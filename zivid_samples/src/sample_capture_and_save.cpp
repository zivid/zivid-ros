#include <exception>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include <zivid_interfaces/srv/capture_and_save.hpp>

/*
 * This sample shows how to set the settings_yaml parameter of the zivid node, then invoke the
 * capture_and_save service, and read the response from the service call. If successful, the
 * captured frame is stored in a temporary directory.
 */

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
      RCLCPP_ERROR(node->get_logger(), "Client interrupted while waiting for service to appear.");
      std::terminate();
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  auto result = param_client->set_parameters({rclcpp::Parameter("settings_yaml", settings_yml)});
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to set `settings_yaml` parameter");
    std::terminate();
  }
}

void capture_and_save(const std::shared_ptr<rclcpp::Node> & node)
{
  using zivid_interfaces::srv::CaptureAndSave;

  auto client = node->create_client<CaptureAndSave>("capture_and_save");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted while waiting for service to appear.");
      std::terminate();
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the capture_and_save service to appear...");
  }

  const std::string filename = "zivid_sample_capture_and_save.zdf";

  auto request = std::make_shared<CaptureAndSave::Request>();
  request->file_path = (std::filesystem::temp_directory_path() / filename).string();
  RCLCPP_INFO(
    node->get_logger(), "Sending capture and save request with file path: %s",
    request->file_path.c_str());

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call capture_and_save service");
    std::terminate();
  }

  auto capture_response = result.get();
  if (!capture_response->success) {
    RCLCPP_ERROR(
      node->get_logger(), "Capture and save operation was unsuccessful: %s",
      capture_response->message.c_str());
    std::terminate();
  }

  RCLCPP_INFO(node->get_logger(), "Capture and save operation successful");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_capture_and_save");
  RCLCPP_INFO(node->get_logger(), "Started the sample_capture_and_save node");

  set_settings(node);

  capture_and_save(node);

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
