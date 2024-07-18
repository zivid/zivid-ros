#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

/*
 * This sample shows how to set the settings_2d_yaml parameter of the zivid node, subscribe for the
 * color/image_color topic, and invoke the capture_2d service. When an image is received, a new
 * capture is triggered.
 */

void set_settings_2d(const std::shared_ptr<rclcpp::Node> & node)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Setting parameter `settings_2d_yaml`");
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

  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");
  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted while waiting for service to appear.");
      std::terminate();
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  auto result =
    param_client->set_parameters({rclcpp::Parameter("settings_2d_yaml", settings_2d_yaml)});
  if (
    rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to set `settings_2d_yaml` parameter");
    std::terminate();
  }
}

auto create_capture_2d_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>("capture_2d");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted while waiting for service to appear.");
      std::terminate();
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the capture service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "Capture service is available");
  return client;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_capture_2d");
  RCLCPP_INFO(node->get_logger(), "Started the sample_capture_2d node");

  set_settings_2d(node);

  auto capture_2d_client = create_capture_2d_client(node);
  auto trigger_capture = [&]() {
    RCLCPP_INFO(node->get_logger(), "Triggering 2d capture");
    capture_2d_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  };

  auto color_image_color_subscription = node->create_subscription<sensor_msgs::msg::Image>(
    "color/image_color", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(node->get_logger(), "Received image of size %d x %d", msg->width, msg->height);
      trigger_capture();
    });

  trigger_capture();

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
