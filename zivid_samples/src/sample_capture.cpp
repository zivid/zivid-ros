#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>

/*
 * This sample shows how to set the settings_file_path parameter of the zivid node, subscribe for
 * the points/xyzrgba topic, and invoke the capture service. When a point cloud is received, a new
 * capture is triggered.
 */

class CaptureHandler {
public:
  CaptureHandler(rclcpp::Node::SharedPtr node)
  : node_(node),
    capture_client_(node_->create_client<std_srvs::srv::Trigger>("capture")) 
  {
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points/xyzrgba", 10,
      std::bind(&CaptureHandler::on_point_cloud, this, std::placeholders::_1));

    wait_for_service();
  }

  void wait_for_service()
  {
    while (!capture_client_->wait_for_service(std::chrono::seconds(3))) {
      if (!rclcpp::ok()) {
        fatal_error("Interrupted while waiting for capture service.");
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for capture service...");
    }
    RCLCPP_INFO(node_->get_logger(), "Capture service is available.");
  }

  void trigger_capture() {
    RCLCPP_INFO(node_->get_logger(), "Triggering capture");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    capture_client_->async_send_request(request);
  }

  void on_point_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
    RCLCPP_INFO(node_->get_logger(), "Got point cloud: %d x %d", msg->width, msg->height);
    trigger_capture();
  }

private:
  void fatal_error(const std::string & message) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), message);
    throw std::runtime_error(message);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

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
      throw std::runtime_error("Interrupted while waiting for parameter service.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for parameter service...");
  }

  auto result = param_client->set_parameters({rclcpp::Parameter("settings_yaml", settings_yml)});
  if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(120)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("Failed to set `settings_yaml` parameter");
  }
}

void set_srgb(const std::shared_ptr<rclcpp::Node> & node)
{
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");

  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("Interrupted while waiting for parameter service.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for parameter service...");
  }

  auto result = param_client->set_parameters({rclcpp::Parameter("color_space", "srgb")});
  if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("Failed to set `color_space` parameter");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_capture");

  RCLCPP_INFO(node->get_logger(), "Started the sample_capture node");

  set_settings(node);
  set_srgb(node);

  auto handler = std::make_shared<CaptureHandler>(node);

  handler->trigger_capture();

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
