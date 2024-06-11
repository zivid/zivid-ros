// Copyright 2024 Zivid AS
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

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

/*
 * This sample shows how to set the settings_file_path parameter of the zivid
 * node, subscribe for the points/xyzrgba topic, and invoke the capture service.
 * When a point cloud is received, a new capture is triggered.
 */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_capture");
  RCLCPP_INFO(node->get_logger(), "Started the sample_capture node");

  auto client = node->create_client<std_srvs::srv::Trigger>("capture");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Client interrupted while waiting for service to appear.");
      return EXIT_FAILURE;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Waiting for the capture service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "Service is available");

  auto param_client =
    std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");
  if (!param_client->service_is_ready()) {
    RCLCPP_ERROR(node->get_logger(), "Param client is not ready");
    return EXIT_FAILURE;
  }
  const auto share_directory =
    ament_index_cpp::get_package_share_directory("zivid_samples");
  const auto path_to_settings_yml =
    share_directory + "/settings/camera_settings.yml";

  RCLCPP_INFO_STREAM(
    node->get_logger(), "Setting settings path to '"
      << path_to_settings_yml << "'");

  param_client->set_parameters(
    {rclcpp::Parameter("settings_file_path", path_to_settings_yml)},
    [&node](auto future) {
      auto results = future.get();
      if (results.size() != 1) {
        RCLCPP_ERROR_STREAM(
          node->get_logger(),
          "Expected 1 result, got " << results.size());
      } else {
        if (results[0].successful) {
          RCLCPP_INFO(node->get_logger(), "Successfully set settings path");
        } else {
          RCLCPP_ERROR(node->get_logger(), "Failed to set settings path");
        }
      }
    });

  auto trigger_capture = [&]() {
      RCLCPP_INFO(node->get_logger(), "Triggering capture");
      client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    };

  auto points_xyzrgba_subscription =
    node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points/xyzrgba", 10,
    [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        node->get_logger(),
        "Received point cloud of width %d and height %d",
        msg->width, msg->height);
      RCLCPP_INFO(
        node->get_logger(),
        "Re-trigger capture");
      trigger_capture();
    });

  trigger_capture();

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
