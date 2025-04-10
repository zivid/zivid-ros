// Copyright 2025 Zivid AS
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

#include <Zivid/Calibration/DetectionResult.h>
#include <Zivid/Calibration/DetectionResultFiducialMarkers.h>
#include <Zivid/Calibration/Pose.h>
#include <Zivid/Exception.h>
#include <Zivid/Matrix.h>

#include <exception>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <zivid_interfaces/msg/detection_result_calibration_board.hpp>
#include <zivid_interfaces/msg/detection_result_fiducial_markers.hpp>

namespace zivid_camera
{
template <typename T>
struct DependentFalse : std::false_type
{
};

template <class... Ts>
struct Overloaded : Ts...
{
  using Ts::operator()...;
};

template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

template <typename T, typename U>
T safeCast(U);  // Not implemented on purpose.

template <>
inline int safeCast(size_t value)
{
  if (value > size_t{std::numeric_limits<int>::max()}) {
    throw std::runtime_error("Value is out of range: " + std::to_string(value));
  }
  return static_cast<int>(value);
}

template <>
inline size_t safeCast(int value)
{
  if (value < 0) {
    throw std::runtime_error("Value is out of range: " + std::to_string(value));
  }
  return static_cast<size_t>(value);
}

template <typename ZividDataModel>
auto serializeZividDataModel(const ZividDataModel & dm)
{
  return dm.serialize();
}

template <typename ZividDataModel>
auto deserializeZividDataModel(const std::string & serialized)
{
  return ZividDataModel::fromSerialized(serialized);
}

template <typename Function, typename ResponseSharedPtr, typename Logger>
void runFunctionAndCatchExceptionsForTriggerResponse(
  Function && function, ResponseSharedPtr & response, const Logger & logger,
  const std::string & operation)
{
  try {
    response->success = true;
    function();
  } catch (const std::exception & exception) {
    const auto exception_message = Zivid::toString(exception);
    RCLCPP_ERROR_STREAM(
      logger, operation + " failed with exception: \"" << exception_message << "\"");
    response->success = false;
    response->message = exception_message;
  }
}

template <typename Logger>
[[noreturn]] void logErrorToLoggerAndThrowRuntimeException(
  const Logger & logger, const std::string & message)
{
  RCLCPP_ERROR(logger, "%s", message.c_str());
  throw std::runtime_error(message);
}

inline constexpr double meterToMillimeterFactor = 1000.0;

constexpr float rosLengthToZivid(double value)
{
  return static_cast<float>(meterToMillimeterFactor * value);
}

constexpr double zividLengthToRos(float value)
{
  return static_cast<double>(value) / meterToMillimeterFactor;
}

void ensureIdentityOrThrow(const Zivid::Matrix4x4 & matrix);

Zivid::Calibration::Pose toZividPose(const geometry_msgs::msg::Pose & pose);

Zivid::PointXYZ toZividPoint(const geometry_msgs::msg::Point & point);

geometry_msgs::msg::Point pixelCoordinatesToGeometryMsgPoint(
  const Zivid::PointXY & pixel_coordinates);

geometry_msgs::msg::Transform toGeometryMsgTransform(const Zivid::Matrix4x4 & transform);

geometry_msgs::msg::Pose toGeometryMsgPose(const Zivid::Calibration::Pose & pose);

geometry_msgs::msg::Point toGeometryMsgPoint(const Zivid::PointXYZ & point);

zivid_interfaces::msg::DetectionResultCalibrationBoard toZividMsgDetectionResult(
  const Zivid::Calibration::DetectionResult & detection);

zivid_interfaces::msg::DetectionResultFiducialMarkers toZividMsgDetectionResult(
  const Zivid::Calibration::DetectionResultFiducialMarkers & detection);

}  // namespace zivid_camera
