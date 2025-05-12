#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>

/*
 * This sample shows how to set the settings_file_path parameter of the zivid node, subscribe for
 * the points/xyzrgba topic, and invoke the capture service. When a point cloud is received, a new
 * capture is triggered.
 */

rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr g_capture_client;
std::shared_ptr<rclcpp::Node> g_node;

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

void set_settings(const std::shared_ptr<rclcpp::Node> & node)
{
  const std::string settings_yml = R"(
__version__:
  serializer: 1
  data: 22
Settings:
  Acquisitions:
    - Acquisition:
        Aperture: 5.66
        ExposureTime: 8333
  Processing:
    Filters:
      Outlier:
        Removal:
          Enabled: yes
          Threshold: 5
)";
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");

  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Interrupted while waiting for parameter service.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for parameter service...");
  }

  auto result = param_client->set_parameters({rclcpp::Parameter("settings_yaml", settings_yml)});
  if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(120)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set `settings_yaml` parameter");
  }
}

void set_srgb(const std::shared_ptr<rclcpp::Node> & node)
{
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");

  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Interrupted while waiting for parameter service.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for parameter service...");
  }

  auto result = param_client->set_parameters({rclcpp::Parameter("color_space", "srgb")});
  if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set `color_space` parameter");
  }
}

rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr create_capture_client(const std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>("capture");

  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Interrupted while waiting for capture service.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for capture service...");
  }

  RCLCPP_INFO(node->get_logger(), "Capture service is available.");
  return client;
}

void trigger_capture()
{
  RCLCPP_INFO(g_node->get_logger(), "Triggering capture");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  g_capture_client->async_send_request(request);
}

void on_point_cloud_received(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  RCLCPP_INFO(g_node->get_logger(), "Received point cloud of size %d x %d", msg->width, msg->height);
  trigger_capture();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("sample_capture");

  RCLCPP_INFO(g_node->get_logger(), "Started the sample_capture node");

  set_settings(g_node);
  set_srgb(g_node);

  g_capture_client = create_capture_client(g_node);


  auto sub = g_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points/xyzrgba", 10, on_point_cloud_received);

  trigger_capture();

  RCLCPP_INFO(g_node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(g_node);
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
