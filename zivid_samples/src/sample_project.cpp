#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <zivid_interfaces/srv/projection_resolution.hpp>
#include <zivid_interfaces/srv/projection_start.hpp>
#include <zivid_interfaces/srv/projection_stop.hpp>

static const auto read_only_parameter =
  rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
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
  auto client = node->create_client<zivid_interfaces::srv::ProjectionStop>("projection/stop");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the projection service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "projection service is available");
  return client;
}

void generate_image(std::vector<uint8_t> & data, uint32_t width, uint32_t height)
{
  data.reserve(width * height);
  for (size_t i = 0; i < height; i++) {
    for (size_t j = 0; j < width; j++) {
      const uint8_t blue = (256 * j) / width;
      const uint8_t red = (256 * (width - j)) / width;
      const uint8_t green = (256 * i) / height;
      data.emplace_back(blue);
      data.emplace_back(green);
      data.emplace_back(red);
      data.emplace_back(255);
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_project");
  RCLCPP_INFO(node->get_logger(), "Started the sample_project node");

  const auto file_path = node->declare_parameter<std::string>("file_path", "", read_only_parameter);

  auto resolution_client = create_resolution_client(node);
  auto get_projection_resolution = [&]() {
    auto req = std::make_shared<zivid_interfaces::srv::ProjectionResolution::Request>();
    auto future = resolution_client->async_send_request(req);
    rclcpp::spin_until_future_complete(node, future);
    return future.get();
  };

  auto project_client = create_project_client(node);
  auto start_projection = [&]() {
    RCLCPP_INFO(node->get_logger(), "Starting projection");
    auto req = std::make_shared<zivid_interfaces::srv::ProjectionStart::Request>();

    if (file_path.empty()) {
      const auto resolution = get_projection_resolution();
      generate_image(req->data, resolution->width, resolution->height);
    } else {
      req->file_path = file_path;
    }
    project_client->async_send_request(req);
  };

  auto stop_client = create_stop_client(node);
  auto stop_projection = [&]() {
    RCLCPP_INFO(node->get_logger(), "Stopping projection");
    stop_client->async_send_request(
      std::make_shared<zivid_interfaces::srv::ProjectionStop::Request>());
  };

  start_projection();

  RCLCPP_INFO(node->get_logger(), "Projecting image.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  stop_projection();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
