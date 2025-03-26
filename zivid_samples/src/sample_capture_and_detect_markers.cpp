#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <zivid_interfaces/srv/capture_and_detect_markers.hpp>

[[noreturn]] void fatal_error(const rclcpp::Logger & logger, const std::string & message)
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

auto create_detect_markers_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<zivid_interfaces::srv::CaptureAndDetectMarkers>(
    "capture_and_detect_markers");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the detect service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "Service is available");
  return client;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_detect_calibration_board");
  RCLCPP_INFO(node->get_logger(), "Started the sample node");

  set_settings(node);

  auto client = create_detect_markers_client(node);
  RCLCPP_INFO(node->get_logger(), "Triggering %s", client->get_service_name());
  auto request = std::make_shared<zivid_interfaces::srv::CaptureAndDetectMarkers::Request>();
  request->marker_dictionary = "aruco4x4_50";
  request->marker_ids = {1, 2, 3};
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the detect markers service");
  }

  auto result = future.get();
  RCLCPP_INFO(node->get_logger(), "Detect markers results (%s):", client->get_service_name());
  RCLCPP_INFO(node->get_logger(), "  Success: %s", result->success ? "true" : "false");
  RCLCPP_INFO(
    node->get_logger(), "  Message: %s",
    result->message.empty() ? "" : (R"(""")" + result->message + R"(""")").c_str());

  RCLCPP_INFO(node->get_logger(), "  Detected markers:");
  const auto & detection_result = result->detection_result;
  for (const auto & marker : detection_result.detected_markers) {
    RCLCPP_INFO(node->get_logger(), "  - ID: %d", marker.id);
    RCLCPP_INFO(
      node->get_logger(),
      "    Pose: {{ Position in meter: %g, %g, %g }, { Orientation as quaternion: %g, %g, %g, %g "
      "}}",
      marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
      marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z,
      marker.pose.orientation.w);
  }

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
