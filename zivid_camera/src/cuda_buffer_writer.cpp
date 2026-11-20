// Copyright 2026 Zivid AS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Zivid AS nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <Zivid/Application.h>
#include <Zivid/ComputeDevice.h>
#include <Zivid/Frame2D.h>
#include <Zivid/PointCloud.h>
#include <cuda.h>
#include <cuda_runtime.h>

#include <cuda_buffer/cuda_buffer_api.hpp>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <zivid_camera/cuda_buffer_writer.hpp>

namespace zivid_camera
{
namespace
{
void throwIfCudaError(cudaError_t error, const std::string & operation)
{
  if (error != cudaSuccess) {
    throw std::runtime_error(
      "CUDA error during " + operation + ": " + cudaGetErrorName(error) + " (" +
      cudaGetErrorString(error) + ")");
  }
}

void throwIfSizeMismatch(std::size_t actual, std::size_t expected, const std::string & what)
{
  if (actual != expected) {
    throw std::runtime_error(
      "Internal error: Unexpected size of " + what + ": " + std::to_string(actual) +
      " bytes, expected " + std::to_string(expected) + " bytes");
  }
}

template <typename ZividDataType>
constexpr bool isColorType = std::is_same_v<ZividDataType, Zivid::ColorRGBA> ||
                             std::is_same_v<ZividDataType, Zivid::ColorRGBA_SRGB>;

template <typename ZividDataType>
auto deviceArray(const Zivid::PointCloud & point_cloud, Zivid::CUDAStreamPtr stream)
{
  if constexpr (std::is_same_v<ZividDataType, Zivid::PointXYZW>) {
    return point_cloud.devicePointsXYZW(stream);
  } else if constexpr (std::is_same_v<ZividDataType, Zivid::PointZ>) {
    return point_cloud.devicePointsZ(stream);
  } else if constexpr (std::is_same_v<ZividDataType, Zivid::SNR>) {
    return point_cloud.deviceSNRs(stream);
  } else if constexpr (std::is_same_v<ZividDataType, Zivid::NormalXYZ>) {
    return point_cloud.deviceNormalsXYZ(stream);
  } else {
    static_assert(!sizeof(ZividDataType), "Unsupported Zivid data type");
  }
}

// The Zivid SDK has no composite XYZ+color device format, so interleave the separate XYZ and color
// device arrays into the message buffer with two strided device-to-device copies.
template <typename ZividColorType>
void interleaveXYZAndColor(
  uint8_t * destination, const Zivid::PointCloud & point_cloud, Zivid::CUDAStreamPtr zivid_stream)
{
  constexpr std::size_t xyz_size = sizeof(Zivid::PointXYZ);
  constexpr std::size_t color_size = sizeof(ZividColorType);
  constexpr std::size_t point_step = xyz_size + color_size;
  static_assert(point_step == sizeof(Zivid::PointXYZColorBGRA));

  const auto stream = static_cast<cudaStream_t>(zivid_stream.stream);
  const auto count = point_cloud.size();
  const auto xyz = point_cloud.devicePointsXYZ(zivid_stream);
  const auto color = point_cloud.imageDeviceArray<ZividColorType>(zivid_stream);
  throwIfSizeMismatch(xyz.sizeInBytes(), count * xyz_size, "XYZ device array");
  throwIfSizeMismatch(color.sizeInBytes(), count * color_size, "color device array");

  throwIfCudaError(
    cudaMemcpy2DAsync(
      destination, point_step, xyz.devicePointer(), xyz_size, xyz_size, count,
      cudaMemcpyDeviceToDevice, stream),
    "copying XYZ into point cloud message");
  throwIfCudaError(
    cudaMemcpy2DAsync(
      destination + xyz_size, point_step, color.devicePointer(), color_size, color_size, count,
      cudaMemcpyDeviceToDevice, stream),
    "copying colors into point cloud message");
}
}  // namespace

CudaBufferWriter::CudaBufferWriter(const Zivid::Application & application)
: application_{application}, stream_{nullptr}
{
  const auto compute_device = application_.computeDevice();
  if (compute_device.backend() != Zivid::ComputeBackend::cuda) {
    throw std::runtime_error(
      "CUDA buffers require the CUDA variant of the Zivid SDK (package zivid-cuda), but the "
      "installed Zivid SDK uses the " +
      Zivid::toString(compute_device.backend()) + " backend.");
  }

  // Calling a CUDA runtime function makes the primary context of the current device current on
  // this thread, if no context is current already.
  throwIfCudaError(cudaFree(nullptr), "CUDA runtime initialization");
  CUcontext current_context = nullptr;
  if (cuCtxGetCurrent(&current_context) != CUDA_SUCCESS || current_context == nullptr) {
    throw std::runtime_error("Failed to get the current CUDA context");
  }
  if (compute_device.nativeContext() != current_context) {
    throw std::runtime_error(
      "CUDA buffers require the Zivid SDK to use the CUDA primary context of the current device, "
      "which is the context used by the CUDA buffer backend. The Zivid Application uses a "
      "different CUDA context. If you construct the Zivid::Application yourself, make the primary "
      "context current before, e.g. by calling cudaFree(nullptr).");
  }

  stream_ = compute_device.nativeStreamHandle();
}

template <typename ZividDataType>
void CudaBufferWriter::write(
  rosidl::Buffer<uint8_t> & data, const Zivid::PointCloud & point_cloud) const
{
  const auto stream = static_cast<cudaStream_t>(stream_);
  const Zivid::CUDAStreamPtr zivid_stream{stream_};
  const auto size = point_cloud.size() * sizeof(ZividDataType);

  data = cuda_buffer_backend::allocate_buffer(size);
  auto write_handle = cuda_buffer_backend::from_output_buffer(data, stream);

  if constexpr (isColorType<ZividDataType>) {
    const auto view = application_.computeDevice().createDeviceArrayView<ZividDataType>(
      Zivid::CUDADevicePointer{write_handle.get_ptr()}, point_cloud.width(), point_cloud.height(),
      zivid_stream);
    point_cloud.imageDeviceArray<ZividDataType>(view, zivid_stream);
  } else if constexpr (std::is_same_v<ZividDataType, Zivid::PointXYZColorBGRA>) {
    interleaveXYZAndColor<Zivid::ColorBGRA>(write_handle.get_ptr(), point_cloud, zivid_stream);
  } else if constexpr (std::is_same_v<ZividDataType, Zivid::PointXYZColorBGRA_SRGB>) {
    interleaveXYZAndColor<Zivid::ColorBGRA_SRGB>(write_handle.get_ptr(), point_cloud, zivid_stream);
  } else {
    const auto source = deviceArray<ZividDataType>(point_cloud, zivid_stream);
    throwIfSizeMismatch(source.sizeInBytes(), size, "device array");
    throwIfCudaError(
      cudaMemcpyAsync(
        write_handle.get_ptr(), source.devicePointer(), size, cudaMemcpyDeviceToDevice, stream),
      "copying into message buffer");
  }
}

template <typename ZividColorType>
ImageResolution CudaBufferWriter::write(
  rosidl::Buffer<uint8_t> & data, const Zivid::Frame2D & frame2D) const
{
  static_assert(isColorType<ZividColorType>, "Unsupported Zivid color type");

  const auto stream = static_cast<cudaStream_t>(stream_);
  const auto source = frame2D.imageDeviceArray<ZividColorType>(Zivid::CUDAStreamPtr{stream_});
  const auto shape = source.shape();
  if (shape.size() < 2) {
    throw std::runtime_error(
      "Internal error: Unexpected 2D image device array with " + std::to_string(shape.size()) +
      " dimensions");
  }
  const auto resolution =
    ImageResolution{static_cast<std::size_t>(shape.at(1)), static_cast<std::size_t>(shape.at(0))};
  const auto size = resolution.width * resolution.height * sizeof(ZividColorType);
  throwIfSizeMismatch(source.sizeInBytes(), size, "2D image device array");

  data = cuda_buffer_backend::allocate_buffer(size);
  auto write_handle = cuda_buffer_backend::from_output_buffer(data, stream);
  throwIfCudaError(
    cudaMemcpyAsync(
      write_handle.get_ptr(), source.devicePointer(), size, cudaMemcpyDeviceToDevice, stream),
    "copying 2D image into message buffer");
  return resolution;
}

template void CudaBufferWriter::write<Zivid::PointXYZW>(
  rosidl::Buffer<uint8_t> &, const Zivid::PointCloud &) const;
template void CudaBufferWriter::write<Zivid::PointZ>(
  rosidl::Buffer<uint8_t> &, const Zivid::PointCloud &) const;
template void CudaBufferWriter::write<Zivid::SNR>(
  rosidl::Buffer<uint8_t> &, const Zivid::PointCloud &) const;
template void CudaBufferWriter::write<Zivid::NormalXYZ>(
  rosidl::Buffer<uint8_t> &, const Zivid::PointCloud &) const;
template void CudaBufferWriter::write<Zivid::ColorRGBA>(
  rosidl::Buffer<uint8_t> &, const Zivid::PointCloud &) const;
template void CudaBufferWriter::write<Zivid::ColorRGBA_SRGB>(
  rosidl::Buffer<uint8_t> &, const Zivid::PointCloud &) const;
template void CudaBufferWriter::write<Zivid::PointXYZColorBGRA>(
  rosidl::Buffer<uint8_t> &, const Zivid::PointCloud &) const;
template void CudaBufferWriter::write<Zivid::PointXYZColorBGRA_SRGB>(
  rosidl::Buffer<uint8_t> &, const Zivid::PointCloud &) const;
template ImageResolution CudaBufferWriter::write<Zivid::ColorRGBA>(
  rosidl::Buffer<uint8_t> &, const Zivid::Frame2D &) const;
template ImageResolution CudaBufferWriter::write<Zivid::ColorRGBA_SRGB>(
  rosidl::Buffer<uint8_t> &, const Zivid::Frame2D &) const;
}  // namespace zivid_camera
