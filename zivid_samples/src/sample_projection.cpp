#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <zivid_interfaces/srv/projection_resolution.hpp>
#include <zivid_interfaces/srv/projection_start.hpp>

static const auto read_only_parameter =
  rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
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

auto get_projection_resolution(std::shared_ptr<rclcpp::Node> & node)
{
  auto resolution_client = create_resolution_client(node);
  auto request = std::make_shared<zivid_interfaces::srv::ProjectionResolution::Request>();
  auto future = resolution_client->async_send_request(request);
  rclcpp::spin_until_future_complete(node, future);
  return future.get();
}

std::vector<uint8_t> generate_image(uint32_t width, uint32_t height)
{
  std::vector<uint8_t> pixel_data;
  pixel_data.reserve(width * height);

  for (size_t i = 0; i < height; i++) {
    for (size_t j = 0; j < width; j++) {
      const uint8_t blue = (256 * j) / width;
      const uint8_t red = (256 * (width - j)) / width;
      const uint8_t green = (256 * i) / height;
      pixel_data.emplace_back(blue);
      pixel_data.emplace_back(green);
      pixel_data.emplace_back(red);
      pixel_data.emplace_back(255);
    }
  }
  return pixel_data;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_projection");
  RCLCPP_INFO(node->get_logger(), "Started the sample_projection node");

  const auto image_path =
    node->declare_parameter<std::string>("image_path", "", read_only_parameter);

  auto resolution_client = create_resolution_client(node);
  auto projection_start_client = create_projection_start_client(node);
  auto stop_client = create_stop_client(node);

  RCLCPP_INFO(node->get_logger(), "Starting projection");
  auto request = std::make_shared<zivid_interfaces::srv::ProjectionStart::Request>();

  if (image_path.empty()) {
    const auto resolution = get_projection_resolution(node);
    request->data = generate_image(resolution->width, resolution->height);
  } else {
    request->image_path = image_path;
  }
  projection_start_client->async_send_request(request);

  RCLCPP_INFO(node->get_logger(), "Projecting for 5 seconds");
  rclcpp::spin_until_future_complete(
    node, std::promise<bool>().get_future(), std::chrono::seconds(5));

  RCLCPP_INFO(node->get_logger(), "Stopping projection");
  stop_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
