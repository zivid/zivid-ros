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
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <string>
#include <zivid_camera/camera_health_controller.hpp>
#include <zivid_camera/utility.hpp>
#include <zivid_interfaces/msg/camera_health.hpp>

namespace zivid_camera
{
namespace
{
using CameraHealthMsg = zivid_interfaces::msg::CameraHealth;
using diagnostic_msgs::msg::DiagnosticStatus;

constexpr auto health_check_diagnostics_period_param = "health_check_diagnostics_period";

// Converts the diagnostics period parameter to a timer period. Exactly 0 disables diagnostics and
// is handled by the caller. Any other value must be a positive duration that is representable as a
// timer period, since a zero-duration timer would be ready continuously.
std::chrono::nanoseconds toDiagnosticsPeriod(const rclcpp::Logger & logger, double seconds)
{
  const auto max_seconds = std::chrono::duration<double>{std::chrono::nanoseconds::max()}.count();
  const auto make_message = [&](const std::string & requirement) {
    return std::string{"Parameter "} + health_check_diagnostics_period_param + " must be " +
           requirement + ", got " + std::to_string(seconds);
  };
  if (!std::isfinite(seconds) || seconds < 0.0 || seconds >= max_seconds) {
    logErrorToLoggerAndThrowRuntimeException(
      logger, make_message("0 (disabled) or a positive finite number of seconds"));
  }
  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>{seconds});
  if (period <= std::chrono::nanoseconds::zero()) {
    logErrorToLoggerAndThrowRuntimeException(
      logger, make_message("0 (disabled) or at least 1 nanosecond"));
  }
  return period;
}

#ifdef ZIVID_CAMERA_HAS_CAMERA_HEALTH
std::string statusToString(uint8_t status)
{
  switch (status) {
    case CameraHealthMsg::STATUS_OK:
      return "OK";
    case CameraHealthMsg::STATUS_SUBOPTIMAL:
      return "Suboptimal";
    case CameraHealthMsg::STATUS_ERROR:
      return "Error";
    case CameraHealthMsg::STATUS_UNKNOWN:
      return "Unknown";
    default:
      throw std::runtime_error("Unhandled camera health status: " + std::to_string(status));
  }
}

// The diagnostics levels do not have an "unknown" severity. STALE is the closest match, since it
// signals that the state could not be determined.
uint8_t statusToDiagnosticLevel(uint8_t status)
{
  switch (status) {
    case CameraHealthMsg::STATUS_OK:
      return DiagnosticStatus::OK;
    case CameraHealthMsg::STATUS_SUBOPTIMAL:
      return DiagnosticStatus::WARN;
    case CameraHealthMsg::STATUS_ERROR:
      return DiagnosticStatus::ERROR;
    case CameraHealthMsg::STATUS_UNKNOWN:
      return DiagnosticStatus::STALE;
    default:
      throw std::runtime_error("Unhandled camera health status: " + std::to_string(status));
  }
}

template <typename ZividStatus>
uint8_t toRosStatus(const ZividStatus & status)
{
  using ValueType = typename ZividStatus::ValueType;
  switch (status.value()) {
    case ValueType::ok:
      return CameraHealthMsg::STATUS_OK;
    case ValueType::suboptimal:
      return CameraHealthMsg::STATUS_SUBOPTIMAL;
    case ValueType::error:
      return CameraHealthMsg::STATUS_ERROR;
    case ValueType::unknown:
      return CameraHealthMsg::STATUS_UNKNOWN;
    default:
      throw std::runtime_error("Unhandled Zivid camera health status: " + status.toString());
  }
}

template <typename RosCheck, typename ZividCheck>
RosCheck toRosTemperature(const ZividCheck & check)
{
  RosCheck result;
  result.status = toRosStatus(check.status());
  result.has_value = check.value().hasValue();
  if (result.has_value) {
    result.value_celsius = check.value().value();
  }
  return result;
}

uint8_t toRosFanValue(const Zivid::CameraHealth::Fan::Value & value)
{
  using ValueType = Zivid::CameraHealth::Fan::Value::ValueType;
  switch (value.value()) {
    case ValueType::ok:
      return zivid_interfaces::msg::CameraHealthFan::FAN_OK;
    case ValueType::unexpectedFanStop:
      return zivid_interfaces::msg::CameraHealthFan::FAN_UNEXPECTED_STOP;
    default:
      throw std::runtime_error("Unhandled Zivid camera fan value: " + value.toString());
  }
}

CameraHealthMsg toRosCameraHealth(const Zivid::CameraHealth & health)
{
  CameraHealthMsg msg;
  msg.overall = toRosStatus(health.overall());

  const auto & transfer_speed = health.maxTransferSpeed();
  msg.max_transfer_speed.status = toRosStatus(transfer_speed.status());
  msg.max_transfer_speed.has_value = transfer_speed.value().hasValue();
  if (msg.max_transfer_speed.has_value) {
    msg.max_transfer_speed.value_mbps = transfer_speed.value().value();
  }

  using zivid_interfaces::msg::CameraHealthTemperature;
  msg.temperature_dmd = toRosTemperature<CameraHealthTemperature>(health.temperature().dmd());
  msg.temperature_led = toRosTemperature<CameraHealthTemperature>(health.temperature().led());
  msg.temperature_lens = toRosTemperature<CameraHealthTemperature>(health.temperature().lens());

  const auto & fan = health.fan();
  msg.fan.status = toRosStatus(fan.status());
  msg.fan.has_value = fan.value().hasValue();
  if (msg.fan.has_value) {
    msg.fan.value = toRosFanValue(fan.value());
  }

  const auto & memory = health.memory();
  msg.memory.status = toRosStatus(memory.status());
  msg.memory.has_value = memory.value().hasValue();
  if (msg.memory.has_value) {
    msg.memory.error_count = memory.value().value();
  }

  const auto & infield_verification = health.infieldVerification();
  msg.infield_verification.status = toRosStatus(infield_verification.status());
  msg.infield_verification.has_value = infield_verification.value().hasValue();
  if (msg.infield_verification.has_value) {
    const auto since_epoch = infield_verification.value().value().time_since_epoch();
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
    msg.infield_verification.last_verification.sec = static_cast<int32_t>(seconds.count());
    msg.infield_verification.last_verification.nanosec = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - seconds).count());
  }

  return msg;
}

diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  return key_value;
}

template <typename Check>
void appendCheck(DiagnosticStatus & status, const std::string & name, const Check & check)
{
  status.values.push_back(makeKeyValue(name + " status", statusToString(check.status)));
}

void appendDiagnosticValues(DiagnosticStatus & status, const CameraHealthMsg & health)
{
  appendCheck(status, "Max transfer speed", health.max_transfer_speed);
  if (health.max_transfer_speed.has_value) {
    status.values.push_back(makeKeyValue(
      "Max transfer speed (Mbps)", std::to_string(health.max_transfer_speed.value_mbps)));
  }

  const auto append_temperature = [&](const std::string & name, const auto & temperature) {
    appendCheck(status, name, temperature);
    if (temperature.has_value) {
      status.values.push_back(
        makeKeyValue(name + " (Celsius)", std::to_string(temperature.value_celsius)));
    }
  };
  append_temperature("DMD temperature", health.temperature_dmd);
  append_temperature("LED temperature", health.temperature_led);
  append_temperature("Lens temperature", health.temperature_lens);

  appendCheck(status, "Fan", health.fan);
  if (health.fan.has_value) {
    status.values.push_back(makeKeyValue(
      "Fan", health.fan.value == zivid_interfaces::msg::CameraHealthFan::FAN_OK
               ? "OK"
               : "Unexpected fan stop"));
  }

  appendCheck(status, "Memory", health.memory);
  if (health.memory.has_value) {
    status.values.push_back(
      makeKeyValue("Memory error count", std::to_string(health.memory.error_count)));
  }

  appendCheck(status, "Infield verification", health.infield_verification);
  if (health.infield_verification.has_value) {
    status.values.push_back(makeKeyValue(
      "Last infield verification (seconds since epoch)",
      std::to_string(health.infield_verification.last_verification.sec)));
  }
}
#else
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
    "Period in seconds between camera health checks published on /diagnostics. Set to 0 to "
    "disable.";
  period_descriptor.read_only = true;
  const auto period_seconds =
    node_.declare_parameter<double>(health_check_diagnostics_period_param, 10.0, period_descriptor);

  if (period_seconds == 0.0) {
    RCLCPP_INFO(node_.get_logger(), "Camera health diagnostics are disabled");
    return;
  }
  const auto period = toDiagnosticsPeriod(node_.get_logger(), period_seconds);
#ifdef ZIVID_CAMERA_HAS_CAMERA_HEALTH
  diagnostics_publisher_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  diagnostics_timer_ = rclcpp::create_timer(
    &node_, node_.get_clock(), period,
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
      response->health = toRosCameraHealth(health);
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
    const auto health = toRosCameraHealth(camera_.checkHealth());
    status.level = statusToDiagnosticLevel(health.overall);
    status.message = "Overall: " + statusToString(health.overall);
    appendDiagnosticValues(status, health);
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
