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

#include <Zivid/Camera.h>

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <string>
#include <zivid_camera/camera_health_controller.hpp>
#include <zivid_camera/camera_health_conversion.hpp>
#include <zivid_camera/utility.hpp>

namespace zivid_camera
{
namespace
{
using diagnostic_msgs::msg::DiagnosticStatus;

constexpr auto health_check_diagnostics_period_param = "health_check_diagnostics_period";

#ifndef ZIVID_CAMERA_HAS_CAMERA_HEALTH
constexpr auto sdk_version_error_message =
  "The camera health check requires Zivid SDK 2.18 or newer. The driver was built against Zivid "
  "SDK " ZIVID_CORE_VERSION ".";
#endif
}  // namespace

CameraHealthController::CameraHealthController(
  rclcpp::Node & zivid_camera_node, Zivid::Camera & camera)
: node_{zivid_camera_node}, camera_{camera}
{
  using namespace std::placeholders;

  check_health_service_ = node_.create_service<zivid_interfaces::srv::CameraHealthCheck>(
    "camera_health/check",
    std::bind(&CameraHealthController::checkHealthServiceHandler, this, _1, _2, _3));

  rcl_interfaces::msg::ParameterDescriptor period_descriptor;
  period_descriptor.description =
    "Period in seconds between camera health checks published on /diagnostics, at least 1. Set "
    "to 0 to disable.";
  period_descriptor.read_only = true;
  const auto period_seconds =
    node_.declare_parameter<double>(health_check_diagnostics_period_param, 10.0, period_descriptor);

  const auto period = [&] {
    try {
      return detail::toDiagnosticsPeriod(period_seconds);
    } catch (const std::invalid_argument & exception) {
      logErrorToLoggerAndThrowRuntimeException(node_.get_logger(), exception.what());
    }
  }();
  if (!period.has_value()) {
    RCLCPP_INFO(node_.get_logger(), "Camera health diagnostics are disabled");
    return;
  }
#ifdef ZIVID_CAMERA_HAS_CAMERA_HEALTH
  diagnostics_publisher_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  diagnostics_timer_ = rclcpp::create_timer(
    &node_, node_.get_clock(), period.value(),
    std::bind(&CameraHealthController::onDiagnosticsTimeout, this));
#else
  static_cast<void>(period);
  RCLCPP_WARN(
    node_.get_logger(), "Camera health diagnostics are not available. %s",
    sdk_version_error_message);
#endif
}

CameraHealthController::~CameraHealthController() = default;

void CameraHealthController::checkHealthServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::CameraHealthCheck::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::CameraHealthCheck::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
#ifdef ZIVID_CAMERA_HAS_CAMERA_HEALTH
      const auto health = camera_.checkHealth();
      response->health = detail::toRosCameraHealth(health);
      response->message = health.toString();
#else
      throw std::runtime_error(sdk_version_error_message);
#endif
    },
    response, node_.get_logger(), "CameraHealthCheck");
}

void CameraHealthController::onDiagnosticsTimeout()
{
  RCLCPP_DEBUG_STREAM(node_.get_logger(), __func__);

#ifdef ZIVID_CAMERA_HAS_CAMERA_HEALTH
  DiagnosticStatus status;
  // Use the fully qualified node name, so that the status name is unique when several drivers in
  // different namespaces publish on the global /diagnostics topic.
  status.name = std::string{node_.get_fully_qualified_name()} + ": Camera health";

  try {
    status.hardware_id = camera_.info().serialNumber().value();
    detail::fillDiagnosticStatus(status, detail::toRosCameraHealth(camera_.checkHealth()));
  } catch (const std::exception & exception) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "Camera health check failed: " + Zivid::toString(exception);
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = node_.get_clock()->now();
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
#else
  throw std::logic_error("Camera health diagnostics require Zivid SDK 2.18 or newer");
#endif
}
}  // namespace zivid_camera
