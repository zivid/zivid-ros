#include <Zivid/Application.h>
#include <Zivid/Frame.h>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <zivid_interfaces/srv/capture_and_save.hpp>

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

void set_settings(const std::shared_ptr<rclcpp::Node> & node)
{
  RCLCPP_INFO(node->get_logger(), "Setting parameter 'settings_yaml'");
  const std::string settings_yml =
    R"(
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
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  auto result = param_client->set_parameters({rclcpp::Parameter("settings_yaml", settings_yml)});
  if (
    rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set `settings_yaml` parameter");
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

void capture_and_save(const std::shared_ptr<rclcpp::Node> & node, const std::string & file_path)
{
  using zivid_interfaces::srv::CaptureAndSave;

  auto client = node->create_client<CaptureAndSave>("capture_and_save");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the capture_and_save service to appear...");
  }

  auto request = std::make_shared<CaptureAndSave::Request>();
  request->file_path = file_path;
  RCLCPP_INFO(
    node->get_logger(), "Sending capture and save request with file path: %s",
    request->file_path.c_str());

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the capture_and_save service");
  }

  auto capture_response = result.get();
  if (!capture_response->success) {
    fatal_error(
      node->get_logger(),
      "Capture and save operation was unsuccessful: " + capture_response->message);
  }

  RCLCPP_INFO(node->get_logger(), "Capture and save operation successful");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_with_sdk_capture_and_load_frame");
  RCLCPP_INFO(node->get_logger(), "Started the sample_with_sdk_capture_and_load_frame node");

  set_settings(node);
  set_srgb(node);

  const std::string filename = "zivid_sample_with_sdk_capture_and_load_frame.zdf";
  const std::string file_path = (std::filesystem::temp_directory_path() / filename).string();

  capture_and_save(node, file_path);

  Zivid::Application application;

  Zivid::Frame frame{file_path};
  const auto point_cloud = frame.pointCloud();

  RCLCPP_INFO(
    node->get_logger(), "Loaded frame from '%s'.\nResolution: %zu x %zu\n%s", file_path.c_str(),
    point_cloud.width(), point_cloud.height(), frame.cameraInfo().toString().c_str());

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
