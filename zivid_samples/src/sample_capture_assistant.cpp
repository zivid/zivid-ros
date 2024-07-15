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

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <zivid_interfaces/srv/capture_assistant_suggest_settings.hpp>

#include <exception>

/*
 * This sample shows how to use the capture assistant service to suggest and set the capture
 * settings parameter of the zivid node. Then, it shows how to subscribe to the points/xyzrgba and
 * color/image_color topics, and finally how to invoke the capture service.
 */

void capture_assistant_suggest_settings(const std::shared_ptr<rclcpp::Node> & node)
{
  using zivid_interfaces::srv::CaptureAssistantSuggestSettings;

  auto client = node->create_client<CaptureAssistantSuggestSettings>(
    "capture_assistant/suggest_settings");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted while waiting for service to appear.");
      std::terminate();
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the capture assistant service to appear...");
  }

  auto request = std::make_shared<CaptureAssistantSuggestSettings::Request>();
  request->max_capture_time = rclcpp::Duration::from_seconds(2.0);
  request->ambient_light_frequency =
    CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_NONE;

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call capture assistant service");
    std::terminate();
  }
}

void capture(const std::shared_ptr<rclcpp::Node> & node)
{
  auto capture_client = node->create_client<std_srvs::srv::Trigger>("capture");
  while (!capture_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted while waiting for service to appear.");
      std::terminate();
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the capture service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "Triggering capture");
  auto result = capture_client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());

  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to trigger capture");
    std::terminate();
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_capture_assistant");
  RCLCPP_INFO(node->get_logger(), "Started the sample_capture_assistant node");

  auto points_xyzrgba_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points/xyzrgba", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        node->get_logger(), "Received point cloud of size %d x %d", msg->width,
        msg->height);
    });

  auto color_image_color_subscription = node->create_subscription<sensor_msgs::msg::Image>(
    "color/image_color", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(node->get_logger(), "Received image of size %d x %d", msg->width, msg->height);
    });

  capture_assistant_suggest_settings(node);

  capture(node);

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
