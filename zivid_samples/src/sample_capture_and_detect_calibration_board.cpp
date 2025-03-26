#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <zivid_interfaces/srv/capture_and_detect_calibration_board.hpp>

[[noreturn]] void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

std::string detection_status_to_string(
  const std::shared_ptr<rclcpp::Node> & node,
  const zivid_interfaces::msg::DetectionResultCalibrationBoard & msg)
{
  using zivid_interfaces::msg::DetectionResultCalibrationBoard;
  switch (msg.status) {
    case DetectionResultCalibrationBoard::STATUS_NOT_SET:
      return "STATUS_NOT_SET";
    case DetectionResultCalibrationBoard::STATUS_OK:
      return "STATUS_OK";
    case DetectionResultCalibrationBoard::STATUS_NO_VALID_FIDUCIAL_MARKER_DETECTED:
      return "STATUS_NO_VALID_FIDUCIAL_MARKER_DETECTED";
    case DetectionResultCalibrationBoard::STATUS_MULTIPLE_VALID_FIDUCIAL_MARKERS_DETECTED:
      return "STATUS_MULTIPLE_VALID_FIDUCIAL_MARKERS_DETECTED";
    case DetectionResultCalibrationBoard::STATUS_BOARD_DETECTION_FAILED:
      return "STATUS_BOARD_DETECTION_FAILED";
    case DetectionResultCalibrationBoard::STATUS_INSUFFICIENT_3D_QUALITY:
      return "STATUS_INSUFFICIENT_3D_QUALITY";
    default:
      fatal_error(node->get_logger(), "Invalid status: " + std::to_string(msg.status));
  }
}

auto create_detect_markers_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<zivid_interfaces::srv::CaptureAndDetectCalibrationBoard>(
    "capture_and_detect_calibration_board");
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

  auto client = create_detect_markers_client(node);
  RCLCPP_INFO(node->get_logger(), "Triggering %s", client->get_service_name());
  auto request =
    std::make_shared<zivid_interfaces::srv::CaptureAndDetectCalibrationBoard::Request>();
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the detect calibration board service");
  }

  auto result = future.get();
  RCLCPP_INFO(
    node->get_logger(), "Detect calibration board results (%s):", client->get_service_name());
  RCLCPP_INFO(node->get_logger(), "  Success: %s", result->success ? "true" : "false");
  RCLCPP_INFO(
    node->get_logger(), "  Message: %s",
    result->message.empty() ? "" : (R"(""")" + result->message + R"(""")").c_str());

  RCLCPP_INFO(node->get_logger(), "  Detection result:");
  const auto & detection_result = result->detection_result;
  RCLCPP_INFO(
    node->get_logger(), "    Status: %s",
    detection_status_to_string(node, detection_result).c_str());
  RCLCPP_INFO(
    node->get_logger(), "    Status description: %s", detection_result.status_description.c_str());
  RCLCPP_INFO(
    node->get_logger(), "    Centroid in meter: %g, %g, %g", detection_result.centroid.x,
    detection_result.centroid.y, detection_result.centroid.z);
  RCLCPP_INFO(
    node->get_logger(),
    "    Pose: {{ Position in meter: %g, %g, %g }, { Orientation as quaternion: %g, %g, %g, %g }}",
    detection_result.pose.position.x, detection_result.pose.position.y,
    detection_result.pose.position.z, detection_result.pose.orientation.x,
    detection_result.pose.orientation.y, detection_result.pose.orientation.z,
    detection_result.pose.orientation.w);

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
