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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <test_zivid_camera.hpp>
#include <zivid_camera/camera_health_controller.hpp>
#include <zivid_camera/camera_health_conversion.hpp>
#include <zivid_interfaces/msg/camera_health.hpp>
#include <zivid_interfaces/srv/camera_health_check.hpp>
#include <zivid_interfaces/srv/camera_info_serial_number.hpp>

namespace
{
using CameraHealthMsg = zivid_interfaces::msg::CameraHealth;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

constexpr auto check_health_service_name = "camera_health/check";
constexpr auto diagnostics_topic_name = "/diagnostics";

template <typename Check>
void verifyCheckIsUnknownWithoutValue(const Check & check)
{
  EXPECT_EQ(check.status, CameraHealthMsg::STATUS_UNKNOWN);
  EXPECT_FALSE(check.has_value);
}

std::string findValue(const DiagnosticStatus & status, const std::string & key)
{
  const auto it = std::find_if(
    status.values.begin(), status.values.end(),
    [&](const auto & key_value) { return key_value.key == key; });
  if (it == status.values.end()) {
    throw std::runtime_error("Diagnostic status has no key '" + key + "'");
  }
  return it->value;
}

#ifdef ZIVID_CAMERA_HAS_CAMERA_HEALTH
bool hasValue(const DiagnosticStatus & status, const std::string & key)
{
  return std::any_of(status.values.begin(), status.values.end(), [&](const auto & key_value) {
    return key_value.key == key;
  });
}

// Captured with CameraHealth::save() from a Zivid 3 XL250. The infield verification is old, so
// it is Suboptimal, and so is the overall status.
constexpr auto camera_health_zivid3_xl250_yaml = R"(__version__: 1
CameraHealth:
  Fan:
    Status: OK
    Value: ok
  InfieldVerification:
    Status: Suboptimal
    Value: 2025-10-27 09:56:55
  MaxTransferSpeed:
    Status: OK
    Value: 10000
  Memory:
    Status: OK
    Value: 0
  Overall: Suboptimal
  Temperature:
    DMD:
      Status: OK
      Value: 29.066404342651367
    LED:
      Status: OK
      Value: 28.834449768066406
    Lens:
      Status: OK
      Value: 28.679813385009766
)";

// Captured with CameraHealth::save() from a Zivid 2+ L110 that has never been verified in the
// field. Every check except the infield verification is OK, but the overall status is Unknown.
constexpr auto camera_health_zivid2plus_l110_yaml = R"(__version__: 1
CameraHealth:
  Fan:
    Status: OK
    Value: ok
  InfieldVerification:
    Status: Unknown
    Value: __not_set__
  MaxTransferSpeed:
    Status: OK
    Value: 10000
  Memory:
    Status: OK
    Value: 0
  Overall: Unknown
  Temperature:
    DMD:
      Status: OK
      Value: 35.338932037353516
    LED:
      Status: OK
      Value: 34.571205139160156
    Lens:
      Status: OK
      Value: 34.850379943847656
)";

// Synthetic, to cover the Error status, the fan stop, a non-zero memory error count and a check
// without value.
constexpr auto camera_health_synthetic_yaml = R"(__version__: 1
CameraHealth:
  Fan:
    Status: Error
    Value: unexpectedFanStop
  InfieldVerification:
    Status: OK
    Value: 2025-11-24 16:00:00
  MaxTransferSpeed:
    Status: Suboptimal
    Value: 1000
  Memory:
    Status: Error
    Value: 3
  Overall: Error
  Temperature:
    DMD:
      Status: Error
      Value: 71.5
    LED:
      Status: Suboptimal
      Value: 55.25
    Lens:
      Status: Unknown
      Value: __not_set__
)";

// Infield verification times in the YAML fixtures, in seconds since the epoch. The SDK reads the
// times as UTC.
constexpr int32_t zivid3_xl250_infield_verification_time = 1761559015;  // 2025-10-27 09:56:55
constexpr int32_t synthetic_infield_verification_time = 1764000000;     // 2025-11-24 16:00:00

template <typename Check>
void verifyCheck(const Check & check, uint8_t expected_status, bool expected_has_value)
{
  EXPECT_EQ(check.status, expected_status);
  EXPECT_EQ(check.has_value, expected_has_value);
}
#endif
}  // namespace

class CameraHealthTest : public ZividNodeTest
{
protected:
  std::shared_ptr<const DiagnosticArray> waitForDiagnostics(
    const SubscriptionWrapper<DiagnosticArray> & subscription)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{10};
    while (subscription.numMessages() == 0 && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      rclcpp::sleep_for(std::chrono::milliseconds{50});
    }
    return subscription.lastMessage();
  }

  // Constructs another driver node that shares the Application of the node under test.
  std::shared_ptr<zivid_camera::ZividCamera> makeCameraNode(
    const std::string & name, const std::string & node_namespace, double diagnostics_period)
  {
    const auto node = ZividCameraNodeWrapper::get();
    const auto shared_application =
      std::shared_ptr<Zivid::Application>(node, &node->zividApplication());
    const auto options =
      rclcpp::NodeOptions{}
        .append_parameter_override(
          "file_camera_path", node->get_parameter("file_camera_path").as_string())
        .append_parameter_override("health_check_diagnostics_period", diagnostics_period);
    return std::make_shared<zivid_camera::ZividCamera>(
      name, node_namespace, options, shared_application);
  }
};

#ifdef ZIVID_CAMERA_HAS_CAMERA_HEALTH

TEST_F(CameraHealthTest, testCheckHealthServiceFileCamera)
{
  const auto response =
    doEmptySrvRequest<zivid_interfaces::srv::CameraHealthCheck>(check_health_service_name);
  ASSERT_TRUE(response->success) << response->message;
  ASSERT_TRUE(containsSubstring(response->message, "CameraHealth"));

  const auto & health = response->health;
  EXPECT_EQ(health.overall, CameraHealthMsg::STATUS_UNKNOWN);
  verifyCheckIsUnknownWithoutValue(health.max_transfer_speed);
  verifyCheckIsUnknownWithoutValue(health.temperature_dmd);
  verifyCheckIsUnknownWithoutValue(health.temperature_led);
  verifyCheckIsUnknownWithoutValue(health.temperature_lens);
  verifyCheckIsUnknownWithoutValue(health.fan);
  verifyCheckIsUnknownWithoutValue(health.memory);
  verifyCheckIsUnknownWithoutValue(health.infield_verification);
}

TEST_F(CameraHealthTest, testHealthDiagnosticsPublishedFileCamera)
{
  const auto diagnostics_sub = subscribe<DiagnosticArray>(diagnostics_topic_name);
  const auto diagnostics = waitForDiagnostics(diagnostics_sub);
  ASSERT_NE(diagnostics, nullptr) << "No message received on " << diagnostics_topic_name;
  ASSERT_EQ(diagnostics->status.size(), 1U);

  const auto serial_number =
    doEmptySrvRequest<zivid_interfaces::srv::CameraInfoSerialNumber>("camera_info/serial_number")
      ->serial_number;

  const auto & status = diagnostics->status.front();
  EXPECT_EQ(status.name, "/zivid_camera: Camera health");
  EXPECT_EQ(status.hardware_id, serial_number);
  EXPECT_EQ(status.level, DiagnosticStatus::STALE);
  EXPECT_EQ(
    status.message,
    "Unknown: Max transfer speed, DMD temperature, LED temperature, Lens temperature, Fan, "
    "Memory, Infield verification");
  EXPECT_EQ(findValue(status, "Overall status"), "Unknown");
  EXPECT_EQ(findValue(status, "Max transfer speed status"), "Unknown");
  EXPECT_EQ(findValue(status, "DMD temperature status"), "Unknown");
  EXPECT_EQ(findValue(status, "LED temperature status"), "Unknown");
  EXPECT_EQ(findValue(status, "Lens temperature status"), "Unknown");
  EXPECT_EQ(findValue(status, "Fan status"), "Unknown");
  EXPECT_EQ(findValue(status, "Memory status"), "Unknown");
  EXPECT_EQ(findValue(status, "Infield verification status"), "Unknown");
  EXPECT_THROW(findValue(status, "Max transfer speed (Mbps)"), std::runtime_error);
}

TEST_F(CameraHealthTest, testHealthDiagnosticsStatusNameIsUniqueAcrossNamespaces)
{
  // Same node name as the node under test, but in a namespace. Both publish on the global
  // /diagnostics topic, so the status names must differ.
  const auto namespaced_node = makeCameraNode("zivid_camera", "left", 0.2);
  executor_.add_node(namespaced_node);

  std::set<std::string> status_names;
  const auto subscription = test_node_->create_subscription<DiagnosticArray>(
    diagnostics_topic_name, rclcpp::SystemDefaultsQoS(),
    [&](const DiagnosticArray::ConstSharedPtr message) {
      for (const auto & status : message->status) {
        status_names.insert(status.name);
      }
    });

  const std::set<std::string> expected_names{
    "/zivid_camera: Camera health", "/left/zivid_camera: Camera health"};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{10};
  while (status_names != expected_names && std::chrono::steady_clock::now() < deadline) {
    executor_.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds{50});
  }
  executor_.remove_node(namespaced_node);

  EXPECT_EQ(status_names, expected_names);
}

// Provides the Zivid Application that the SDK requires for parsing CameraHealth YAML.
class CameraHealthConversionTest : public ZividNodeTest
{
protected:
  static zivid_interfaces::msg::CameraHealth convert(const std::string & yaml)
  {
    return zivid_camera::detail::toRosCameraHealth(Zivid::CameraHealth::fromSerialized(yaml));
  }
};

TEST_F(CameraHealthConversionTest, testConversionRealCameraZivid3XL250)
{
  const auto health = convert(camera_health_zivid3_xl250_yaml);
  EXPECT_EQ(health.overall, CameraHealthMsg::STATUS_SUBOPTIMAL);

  verifyCheck(health.max_transfer_speed, CameraHealthMsg::STATUS_OK, true);
  EXPECT_EQ(health.max_transfer_speed.value_mbps, 10000U);

  verifyCheck(health.temperature_dmd, CameraHealthMsg::STATUS_OK, true);
  EXPECT_DOUBLE_EQ(health.temperature_dmd.value_celsius, 29.066404342651367);
  verifyCheck(health.temperature_led, CameraHealthMsg::STATUS_OK, true);
  EXPECT_DOUBLE_EQ(health.temperature_led.value_celsius, 28.834449768066406);
  verifyCheck(health.temperature_lens, CameraHealthMsg::STATUS_OK, true);
  EXPECT_DOUBLE_EQ(health.temperature_lens.value_celsius, 28.679813385009766);

  verifyCheck(health.fan, CameraHealthMsg::STATUS_OK, true);
  EXPECT_EQ(health.fan.value, zivid_interfaces::msg::CameraHealthFan::FAN_OK);

  verifyCheck(health.memory, CameraHealthMsg::STATUS_OK, true);
  EXPECT_EQ(health.memory.error_count, 0U);

  verifyCheck(health.infield_verification, CameraHealthMsg::STATUS_SUBOPTIMAL, true);
  EXPECT_EQ(
    health.infield_verification.last_verification.sec, zivid3_xl250_infield_verification_time);
  EXPECT_EQ(health.infield_verification.last_verification.nanosec, 0U);
}

TEST_F(CameraHealthConversionTest, testConversionRealCameraZivid2PlusL110)
{
  const auto health = convert(camera_health_zivid2plus_l110_yaml);
  EXPECT_EQ(health.overall, CameraHealthMsg::STATUS_UNKNOWN);
  verifyCheck(health.max_transfer_speed, CameraHealthMsg::STATUS_OK, true);
  verifyCheck(health.temperature_dmd, CameraHealthMsg::STATUS_OK, true);
  EXPECT_DOUBLE_EQ(health.temperature_dmd.value_celsius, 35.338932037353516);
  verifyCheck(health.temperature_led, CameraHealthMsg::STATUS_OK, true);
  verifyCheck(health.temperature_lens, CameraHealthMsg::STATUS_OK, true);
  verifyCheck(health.fan, CameraHealthMsg::STATUS_OK, true);
  verifyCheck(health.memory, CameraHealthMsg::STATUS_OK, true);
  verifyCheck(health.infield_verification, CameraHealthMsg::STATUS_UNKNOWN, false);
}

TEST_F(CameraHealthConversionTest, testConversionSynthetic)
{
  const auto health = convert(camera_health_synthetic_yaml);
  EXPECT_EQ(health.overall, CameraHealthMsg::STATUS_ERROR);

  verifyCheck(health.max_transfer_speed, CameraHealthMsg::STATUS_SUBOPTIMAL, true);
  EXPECT_EQ(health.max_transfer_speed.value_mbps, 1000U);

  verifyCheck(health.temperature_dmd, CameraHealthMsg::STATUS_ERROR, true);
  EXPECT_DOUBLE_EQ(health.temperature_dmd.value_celsius, 71.5);
  verifyCheck(health.temperature_led, CameraHealthMsg::STATUS_SUBOPTIMAL, true);
  EXPECT_DOUBLE_EQ(health.temperature_led.value_celsius, 55.25);
  verifyCheck(health.temperature_lens, CameraHealthMsg::STATUS_UNKNOWN, false);

  verifyCheck(health.fan, CameraHealthMsg::STATUS_ERROR, true);
  EXPECT_EQ(health.fan.value, zivid_interfaces::msg::CameraHealthFan::FAN_UNEXPECTED_STOP);

  verifyCheck(health.memory, CameraHealthMsg::STATUS_ERROR, true);
  EXPECT_EQ(health.memory.error_count, 3U);

  verifyCheck(health.infield_verification, CameraHealthMsg::STATUS_OK, true);
  EXPECT_EQ(health.infield_verification.last_verification.sec, synthetic_infield_verification_time);
}

TEST_F(CameraHealthConversionTest, testDiagnosticStatusSynthetic)
{
  DiagnosticStatus status;
  zivid_camera::detail::fillDiagnosticStatus(status, convert(camera_health_synthetic_yaml));

  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status.message,
    "Error: DMD temperature, Fan, Memory; Suboptimal: Max transfer speed, LED temperature; "
    "Unknown: Lens temperature");
  EXPECT_EQ(findValue(status, "Overall status"), "Error");
  EXPECT_EQ(findValue(status, "Max transfer speed status"), "Suboptimal");
  EXPECT_EQ(findValue(status, "Max transfer speed (Mbps)"), "1000");
  EXPECT_EQ(findValue(status, "DMD temperature status"), "Error");
  EXPECT_EQ(findValue(status, "DMD temperature (Celsius)"), std::to_string(71.5));
  EXPECT_EQ(findValue(status, "Lens temperature status"), "Unknown");
  EXPECT_FALSE(hasValue(status, "Lens temperature (Celsius)"));
  EXPECT_EQ(findValue(status, "Fan status"), "Error");
  EXPECT_EQ(findValue(status, "Fan"), "Unexpected fan stop");
  EXPECT_EQ(findValue(status, "Memory status"), "Error");
  EXPECT_EQ(findValue(status, "Memory error count"), "3");
  EXPECT_EQ(findValue(status, "Infield verification status"), "OK");
  EXPECT_EQ(
    findValue(status, "Last infield verification (seconds since epoch)"),
    std::to_string(synthetic_infield_verification_time));
}

TEST_F(CameraHealthConversionTest, testDiagnosticStatusRealCameras)
{
  DiagnosticStatus xl250_status;
  zivid_camera::detail::fillDiagnosticStatus(
    xl250_status, convert(camera_health_zivid3_xl250_yaml));
  EXPECT_EQ(xl250_status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(xl250_status.message, "Suboptimal: Infield verification");

  // The overall status is Unknown only because the camera was never verified in the field. That
  // must not hide that all other checks are OK.
  DiagnosticStatus l110_status;
  zivid_camera::detail::fillDiagnosticStatus(
    l110_status, convert(camera_health_zivid2plus_l110_yaml));
  EXPECT_EQ(l110_status.level, DiagnosticStatus::OK);
  EXPECT_EQ(l110_status.message, "Unknown: Infield verification");
  EXPECT_EQ(findValue(l110_status, "Overall status"), "Unknown");
}

#else

TEST_F(CameraHealthTest, testCheckHealthServiceUnsupportedSdk)
{
  const auto response =
    doEmptySrvRequest<zivid_interfaces::srv::CameraHealthCheck>(check_health_service_name);
  ASSERT_FALSE(response->success);
  ASSERT_TRUE(containsSubstring(response->message, "requires Zivid SDK 2.18 or newer"));
}

#endif

TEST_F(CameraHealthTest, testHealthDiagnosticsInvalidPeriodThrows)
{
  for (const auto period : {
         -1.0,
         -std::numeric_limits<double>::min(),
         1e-10,  // Positive, but rounds to a zero-duration timer
         std::numeric_limits<double>::quiet_NaN(),
         std::numeric_limits<double>::infinity(),
         1e20,  // Larger than the maximum timer period
       }) {
    SCOPED_TRACE("period " + std::to_string(period));
    EXPECT_THROW(makeCameraNode("zivid_camera_invalid_period", "", period), std::runtime_error);
  }
}

TEST_F(CameraHealthTest, testHealthDiagnosticsSmallPositivePeriodIsAccepted)
{
  EXPECT_NO_THROW(makeCameraNode("zivid_camera_small_period", "", 1e-6));
}

TEST(CameraHealthDiagnosticLevelTest, testDiagnosticLevelFromChecks)
{
  // Overall status and per-check statuses, with the infield verification status set separately.
  const auto diagnose = [](uint8_t overall, uint8_t checks, uint8_t infield_verification) {
    CameraHealthMsg health;
    health.overall = overall;
    health.max_transfer_speed.status = checks;
    health.temperature_dmd.status = checks;
    health.temperature_led.status = checks;
    health.temperature_lens.status = checks;
    health.fan.status = checks;
    health.memory.status = checks;
    health.infield_verification.status = infield_verification;
    DiagnosticStatus status;
    zivid_camera::detail::fillDiagnosticStatus(status, health);
    return status;
  };
  constexpr auto ok = CameraHealthMsg::STATUS_OK;
  constexpr auto suboptimal = CameraHealthMsg::STATUS_SUBOPTIMAL;
  constexpr auto error = CameraHealthMsg::STATUS_ERROR;
  constexpr auto unknown = CameraHealthMsg::STATUS_UNKNOWN;

  EXPECT_EQ(diagnose(ok, ok, ok).level, DiagnosticStatus::OK);
  EXPECT_EQ(diagnose(ok, ok, ok).message, "All checks OK");
  EXPECT_EQ(diagnose(suboptimal, ok, suboptimal).level, DiagnosticStatus::WARN);
  EXPECT_EQ(diagnose(error, suboptimal, error).level, DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnose(error, error, ok).level, DiagnosticStatus::ERROR);

  // Unknown checks do not affect the level when other checks are known, also when the SDK reports
  // an overall Unknown.
  EXPECT_EQ(diagnose(unknown, ok, unknown).level, DiagnosticStatus::OK);
  EXPECT_EQ(diagnose(unknown, suboptimal, unknown).level, DiagnosticStatus::WARN);
  EXPECT_EQ(diagnose(unknown, unknown, error).level, DiagnosticStatus::ERROR);

  // STALE only when all checks are Unknown.
  EXPECT_EQ(diagnose(unknown, unknown, unknown).level, DiagnosticStatus::STALE);
}
