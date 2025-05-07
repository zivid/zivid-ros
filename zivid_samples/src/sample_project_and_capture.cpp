#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <zivid_interfaces/srv/projection_resolution.hpp>
#include <zivid_interfaces/srv/projection_start.hpp>

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

void set_settings_2d(const std::shared_ptr<rclcpp::Node> & node)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Setting parameter `settings_2d_yaml`");
  const std::string settings_2d_yaml =
    R"(
__version__: 7
Settings2D:
  Acquisitions:
    - Acquisition:
        Aperture: 3.36
        Brightness: 0
        ExposureTime: 20000
        Gain: 4
  Processing:
    Color:
      Balance:
        Blue: 1
        Green: 1
        Red: 1
      Experimental:
        Mode: automatic
      Gamma: 1
  Sampling:
    Color: grayscale
    Pixel: all
)";

  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");
  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  auto result =
    param_client->set_parameters({rclcpp::Parameter("settings_2d_yaml", settings_2d_yaml)});
  if (
    rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set `settings_2d_yaml` parameter");
  }
}

auto create_resolution_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client =
    node->create_client<zivid_interfaces::srv::ProjectionResolution>("projection/resolution");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the projection service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "projection service is available");
  return client;
}

auto create_project_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<zivid_interfaces::srv::ProjectionStart>("projection/start");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the projection service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "projection service is available");
  return client;
}

auto create_capture_2d_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>("projection/capture_2d");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the projection service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "projection service is available");
  return client;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_project");
  RCLCPP_INFO(node->get_logger(), "Started the sample_project node");

  set_settings_2d(node);

  auto resolution_client = create_resolution_client(node);
  auto get_projection_resolution = [&]() {
    auto req = std::make_shared<zivid_interfaces::srv::ProjectionResolution::Request>();
    auto future = resolution_client->async_send_request(req);
    rclcpp::spin_until_future_complete(node, future);
    return future.get();
  };

  const auto resolution = get_projection_resolution();

  auto project_client = create_project_client(node);
  auto start_projection = [&]() {
    RCLCPP_INFO(node->get_logger(), "Starting projection");
    auto req = std::make_shared<zivid_interfaces::srv::ProjectionStart::Request>();
    const auto width = resolution->width;
    const auto height = resolution->height;

    req->data = std::vector<uint8_t>(4 * width * height, 0);
    const auto line_width = 10;

    // Vertical line
    for (size_t i = 0; i < height; i++) {
      for (size_t j = width / 2 - line_width / 2; j < width / 2 + line_width / 2; j++) {
        const auto pos = 4 * (i * width + j);
        req->data[pos] = 255;
        req->data[pos + 1] = 255;
        req->data[pos + 2] = 255;
        req->data[pos + 3] = 255;
      }
    }

    // Horizontal line
    for (size_t i = height / 2 - line_width / 2; i < height / 2 + line_width / 2; i++) {
      for (size_t j = 0; j < width; j++) {
        const auto pos = 4 * (i * width + j);
        req->data[pos] = 255;
        req->data[pos + 1] = 255;
        req->data[pos + 2] = 255;
        req->data[pos + 3] = 255;
      }
    }

    auto future = project_client->async_send_request(req);
    rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(2));
  };

  auto capture_2d_client = create_capture_2d_client(node);
  auto trigger_capture = [&]() {
    RCLCPP_INFO(node->get_logger(), "Triggering 2d capture");
    capture_2d_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  };

  auto color_image_color_subscription = node->create_subscription<sensor_msgs::msg::Image>(
    "color/image_color", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        node->get_logger(), "Received image of size %d x %d with encoding %s", msg->width,
        msg->height, msg->encoding.c_str());
    });

  start_projection();
  trigger_capture();

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
