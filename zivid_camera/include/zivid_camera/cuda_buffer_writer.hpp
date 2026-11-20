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

#pragma once

// This header is only usable when the driver is built with CUDA buffer support, in which case the
// build system defines ZIVID_CAMERA_WITH_CUDA_BUFFER. See the `use_cuda_buffers` parameter.
#ifdef ZIVID_CAMERA_WITH_CUDA_BUFFER

#include <cstddef>
#include <cstdint>
#include <rosidl_buffer/buffer.hpp>

namespace Zivid
{
class Application;
class Frame2D;
class PointCloud;
}  // namespace Zivid

namespace zivid_camera
{
struct ImageResolution
{
  std::size_t width;
  std::size_t height;
};

// Writes point cloud and image data from the Zivid SDK's GPU memory into CUDA-backed message buffers
// (rosidl::Buffer with the CUDA backend), without copying the data via host memory.
//
// All GPU work is enqueued on the Zivid SDK's own CUDA stream, so it is ordered after the SDK's
// processing. The write is recorded on the buffer when the write completes, and subscribers
// synchronize on it before reading.
class CudaBufferWriter
{
public:
  // Throws if the Zivid SDK does not use the CUDA backend, or does not use the CUDA primary context
  // of the current device, which is the context used by the CUDA buffer backend.
  explicit CudaBufferWriter(const Zivid::Application & application);

  // Supported types: Zivid::PointXYZW, Zivid::PointZ, Zivid::SNR, Zivid::NormalXYZ,
  // Zivid::ColorRGBA, Zivid::ColorRGBA_SRGB, Zivid::PointXYZColorBGRA and
  // Zivid::PointXYZColorBGRA_SRGB. The data layout is identical to PointCloud::copyData of the same
  // type.
  template <typename ZividDataType>
  void write(rosidl::Buffer<uint8_t> & data, const Zivid::PointCloud & point_cloud) const;

  // Supported types: Zivid::ColorRGBA and Zivid::ColorRGBA_SRGB. The data layout is identical to
  // Frame2D::imageRGBA and Frame2D::imageRGBA_SRGB.
  template <typename ZividColorType>
  ImageResolution write(rosidl::Buffer<uint8_t> & data, const Zivid::Frame2D & frame2D) const;

private:
  const Zivid::Application & application_;
  void * stream_;
};
}  // namespace zivid_camera

#endif
