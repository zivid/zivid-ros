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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

/*
 * This sample shows how to set the settings_2d_yaml parameter of the zivid
 * node, subscribe for the color/image_color topic, and invoke the capture_2d service.
 * When an image is received, a new capture is triggered.
 */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_capture_2d");
  RCLCPP_INFO(node->get_logger(), "Started the sample_capture_2d node");

  auto client = node->create_client<std_srvs::srv::Trigger>("capture_2d");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted while waiting for service to appear.");
      return EXIT_FAILURE;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the capture_2d service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "Service is available");

  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");
  if (!param_client->service_is_ready()) {
    RCLCPP_ERROR(node->get_logger(), "Param client is not ready");
    return EXIT_FAILURE;
  }

  const std::string settings_2d_yaml =
    R"(
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
    - Acquisition:
        Aperture: 2.83
        Brightness: 1.0
        ExposureTime: 10000
        Gain: 2.5
)";

  RCLCPP_INFO_STREAM(
    node->get_logger(), "Settings settings_2d_yaml to '" << settings_2d_yaml << "'");

  param_client->set_parameters(
    {rclcpp::Parameter("settings_2d_yaml", settings_2d_yaml)}, [&node](auto future) {
      auto results = future.get();
      if (results.size() != 1) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Expected 1 result, got " << results.size());
      } else {
        if (results[0].successful) {
          RCLCPP_INFO(node->get_logger(), "Successfully set settings_2d_yaml");
        } else {
          RCLCPP_ERROR(node->get_logger(), "Failed to set settings_2d_yaml");
        }
      }
    });

  auto trigger_capture = [&]() {
      RCLCPP_INFO(node->get_logger(), "Triggering 2d capture");
      client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    };

  auto color_image_color_subscription = node->create_subscription<sensor_msgs::msg::Image>(
    "color/image_color", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        node->get_logger(), "Received image of width %d and height %d", msg->width, msg->height);
      RCLCPP_INFO(node->get_logger(), "Re-trigger 2d capture");
      trigger_capture();
    });

  trigger_capture();

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
