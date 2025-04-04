#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>

/*
 * This sample shows how to set the settings_file_path parameter of the zivid node, subscribe for
 * the points/xyzrgba topic, and invoke the capture service. When a point cloud is received, a new
 * capture is triggered.
 */

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

void set_settings(const std::shared_ptr<rclcpp::Node> & node)
{
  const auto share_directory = ament_index_cpp::get_package_share_directory("zivid_samples");
  const auto path_to_settings_yml = share_directory + "/settings/camera_settings.yml";
  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "Setting parameter `settings_file_path` to '" << path_to_settings_yml << "'");

  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");
  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  auto result =
    param_client->set_parameters({rclcpp::Parameter("settings_file_path", path_to_settings_yml)});
  if (
    rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set `settings_file_path` parameter");
  }
}

void set_srgb(const std::shared_ptr<rclcpp::Node> & node)
{
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");
  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  auto result = param_client->set_parameters({rclcpp::Parameter("color_space", "srgb")});
  if (
    rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set `color_space` parameter");
  }
}

auto create_capture_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>("capture");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the capture service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "Service is available");
  return client;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_capture_with_settings_from_file");
  RCLCPP_INFO(node->get_logger(), "Started the sample node");
  set_srgb(node);

  auto points_xyzrgba_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points/xyzrgba", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        node->get_logger(), "Received point cloud of size %d x %d", msg->width, msg->height);
    });

  set_settings(node);

  auto client = create_capture_client(node);

  RCLCPP_INFO(node->get_logger(), "Triggering capture");
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
