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
#include <zivid_interfaces/msg/camera_health.hpp>
#include <zivid_interfaces/srv/camera_health_check.hpp>
#include <zivid_interfaces/srv/camera_info_serial_number.hpp>

namespace
{
using CameraHealthMsg = zivid_interfaces::msg::CameraHealth;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

constexpr auto check_health_service_name = "check_health";
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
  EXPECT_EQ(status.message, "Overall: Unknown");
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
