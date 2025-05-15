#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <zivid_interfaces/srv/camera_info_model_name.hpp>
#include <zivid_interfaces/srv/projection_resolution.hpp>
#include <zivid_interfaces/srv/projection_start.hpp>

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

void set_settings_2d(const std::shared_ptr<rclcpp::Node> & node, std::string_view model_name)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Setting parameter `settings_2d_yaml`");
  std::string settings_2d_yaml =
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
    Color: rgb
    Pixel: all
)";

  for (auto grayscale_model : {"Zivid 2+ MR60", "Zivid 2+ LR110", "Zivid 2+ MR130"}) {
    if (model_name == grayscale_model) {
      const std::string_view color_setting("Color: rgb");
      settings_2d_yaml.replace(
        settings_2d_yaml.find(color_setting), color_setting.size(), "Color: grayscale");
      break;
    }
  }

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

auto create_camera_info_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client =
    node->create_client<zivid_interfaces::srv::CameraInfoModelName>("/camera_info/model_name");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the camera info service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "camera info service is available");
  return client;
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

auto create_projection_start_client(std::shared_ptr<rclcpp::Node> & node)
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

auto create_projection_capture_2d_client(std::shared_ptr<rclcpp::Node> & node)
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

auto create_stop_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>("projection/stop");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the projection service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "projection service is available");
  return client;
}

auto get_model_name(std::shared_ptr<rclcpp::Node> & node)
{
  auto camera_info_client = create_camera_info_client(node);
  auto request = std::make_shared<zivid_interfaces::srv::CameraInfoModelName::Request>();
  auto future = camera_info_client->async_send_request(request);
  rclcpp::spin_until_future_complete(node, future);
  return future.get();
}

auto get_projection_resolution(std::shared_ptr<rclcpp::Node> & node)
{
  auto resolution_client = create_resolution_client(node);
  auto request = std::make_shared<zivid_interfaces::srv::ProjectionResolution::Request>();
  auto future = resolution_client->async_send_request(request);
  rclcpp::spin_until_future_complete(node, future);
  return future.get();
}

std::vector<uint8_t> generate_marker(uint32_t width, uint32_t height)
{
  const auto center_x = width / 2;
  const auto center_y = height / 2;

  std::vector<uint8_t> pixel_data(4 * width * height, 0);

  // Center
  const std::vector<std::pair<int, int>> center{{-1, 0}, {1, 0}, {0, 0}, {0, -1}, {0, 1}};
  for (const auto & [i, j] : center) {
    const auto pos = 4 * ((center_y + i) * width + (center_x + j));
    pixel_data.at(pos + 1) = 255;
  }

  // Horizontal lines
  for (size_t i = center_y - 20; i < center_y - 5; i++) {
    const auto pos = 4 * (i * width + center_x);
    pixel_data.at(pos + 1) = 255;
  }

  for (size_t i = center_y + 5; i < center_y + 20; i++) {
    const auto pos = 4 * (i * width + center_x);
    pixel_data.at(pos + 1) = 255;
  }

  // Vertical lines
  for (size_t j = center_x - 20; j < center_x - 5; j++) {
    const auto pos = 4 * (center_y * width + j);
    pixel_data.at(pos + 1) = 255;
  }

  for (size_t j = center_x + 5; j < center_x + 20; j++) {
    const auto pos = 4 * (center_y * width + j);
    pixel_data.at(pos + 1) = 255;
  }

  return pixel_data;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_project_and_capture");
  RCLCPP_INFO(node->get_logger(), "Started the sample_project_and_capture node");

  const auto response = get_model_name(node);

  set_settings_2d(node, response->model_name);

  const auto resolution = get_projection_resolution(node);

  auto projection_start_client = create_projection_start_client(node);
  auto capture_2d_client = create_projection_capture_2d_client(node);
  auto stop_client = create_stop_client(node);

  auto color_image_color_subscription = node->create_subscription<sensor_msgs::msg::Image>(
    "color/image_color", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(node->get_logger(), "Received image of size %d x %d", msg->width, msg->height);
    });

  RCLCPP_INFO(node->get_logger(), "Starting projection");
  auto request = std::make_shared<zivid_interfaces::srv::ProjectionStart::Request>();
  request->data = generate_marker(resolution->width, resolution->height);

  auto future = projection_start_client->async_send_request(request);
  rclcpp::spin_until_future_complete(node, future);

  RCLCPP_INFO(node->get_logger(), "Triggering 2d capture");
  capture_2d_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

  RCLCPP_INFO(node->get_logger(), "Stopping projection");
  auto stop_future =
    stop_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  rclcpp::spin_until_future_complete(node, stop_future);

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
