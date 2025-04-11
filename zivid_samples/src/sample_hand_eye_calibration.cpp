#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_calibrate.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_capture.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_load.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_start.hpp>

static const auto read_only_parameter =
  rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);

[[noreturn]] void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

std::vector<int> sanitize_marker_ids(
  const rclcpp::Logger & logger, const std::vector<int64_t> & values)
{
  // We use an integer array with a single value [-1] as a placeholder for an empty array, since
  // an empty integer array cannot be specified directly.
  // See: https://github.com/ros2/rclcpp/issues/1955
  if (values.empty() || (values.size() == 1 && values.at(0) == -1)) {
    return {};
  }

  for (const auto & value : values) {
    if (value < 0) {
      fatal_error(logger, "Negative marker IDs are not allowed.");
    }
    if (value > std::numeric_limits<int>::max()) {
      fatal_error(logger, "Marker ID " + std::to_string(value) + " is too large.");
    }
  }

  return std::vector<int>(values.begin(), values.end());
}

std::string format_transform(const geometry_msgs::msg::Transform & transform)
{
  std::ostringstream ss;
  ss << "{ translation in m: {" << transform.translation.x << ", " << transform.translation.y
     << ", " << transform.translation.z << "}, rotation as quaternion: {" << transform.rotation.x
     << ", " << transform.rotation.y << ", " << transform.rotation.z << ", " << transform.rotation.w
     << "}}";
  return ss.str();
}

template <typename List, typename ToStringFunc>
std::string join_list_to_string(const List & list, ToStringFunc && to_string_func)
{
  return list.empty()
           ? std::string()
           : std::accumulate(
               std::next(std::begin(list)), std::end(list), to_string_func(*std::begin(list)),
               [&](const std::string & str, const auto & entry) {
                 return str + ", " + to_string_func(entry);
               });
}

template <typename Enum>
Enum get_parameter_enum(
  rclcpp::Node & node, const std::string & name, const std::map<std::string, Enum> & name_value_map)
{
  const auto value = node.declare_parameter<std::string>(name, "", read_only_parameter);
  const auto it = name_value_map.find(value);
  if (it == name_value_map.end()) {
    const std::string valid_values =
      join_list_to_string(name_value_map, [](auto && pair) { return pair.first; });
    fatal_error(
      node.get_logger(), "Invalid value for parameter '" + name + "': '" + value +
                           "'. Expected one of: " + valid_values + ".");
  }
  RCLCPP_INFO(node.get_logger(), "%s", ("Using parameter " + name + ":=" + value).c_str());
  return it->second;
}

enum class CalibrationConfiguration
{
  EyeToHand,
  EyeInHand,
};
const std::map<std::string, CalibrationConfiguration> calibration_configuration_map = {
  {"eye_to_hand", CalibrationConfiguration::EyeToHand},
  {"eye_in_hand", CalibrationConfiguration::EyeInHand},
};

template <typename Service>
std::shared_ptr<rclcpp::Client<Service>> create_service_client(
  const rclcpp::Node::SharedPtr & node, const std::string & service_name)
{
  auto client = node->create_client<Service>(service_name);
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(
      node->get_logger(), "Waiting for the %s service to appear...", client->get_service_name());
  }
  return client;
}

void set_settings(const std::shared_ptr<rclcpp::Node> & node)
{
  // The following settings are for exposition only. Please refer to the Zivid knowledge base on how
  // to get good quality data for hand-eye calibration:
  // https://support.zivid.com/en/latest/academy/applications/hand-eye/how-to-get-good-quality-data-on-zivid-calibration-board.html

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

bool request_start(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<rclcpp::Client<zivid_interfaces::srv::HandEyeCalibrationStart>> & client,
  const std::vector<int> marker_ids, const std::string & working_directory)
{
  auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationStart::Request>();

  if (!working_directory.empty()) {
    RCLCPP_INFO(node->get_logger(), "Setting working directory to: %s", working_directory.c_str());
    request->working_directory = working_directory;
  }

  if (marker_ids.empty()) {
    request->calibration_objects.type =
      zivid_interfaces::msg::HandEyeCalibrationObjects::CALIBRATION_BOARD;
  } else {
    request->calibration_objects.type =
      zivid_interfaces::msg::HandEyeCalibrationObjects::FIDUCIAL_MARKERS;
    request->calibration_objects.marker_dictionary = "aruco4x4_50";
    request->calibration_objects.marker_ids = marker_ids;
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the hand-eye calibration start service");
  }
  auto result = future.get();
  RCLCPP_INFO(node->get_logger(), "Hand-eye calibration start (%s):", client->get_service_name());
  RCLCPP_INFO(node->get_logger(), "  Success: %s", result->success ? "true" : "false");
  RCLCPP_INFO(
    node->get_logger(), "  Message: %s",
    result->message.empty() ? "" : (R"(""")" + result->message + R"(""")").c_str());
  return result->success;
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

void print_detection_result_calibration_board(
  const std::shared_ptr<rclcpp::Node> & node,
  const zivid_interfaces::msg::DetectionResultCalibrationBoard & detection_result)
{
  RCLCPP_INFO(node->get_logger(), "  Detection result:");
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
}

void print_detection_result_fiducial_markers(
  const std::shared_ptr<rclcpp::Node> & node,
  const zivid_interfaces::msg::DetectionResultFiducialMarkers & detection_result)
{
  RCLCPP_INFO(node->get_logger(), "  Detected markers:");
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
}

bool request_capture_and_print_response(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<rclcpp::Client<zivid_interfaces::srv::HandEyeCalibrationCapture>> & client,
  const geometry_msgs::msg::Pose & pose)
{
  auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCapture::Request>();
  request->robot_pose = pose;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the hand-eye calibration capture service");
  }
  auto result = future.get();
  RCLCPP_INFO(node->get_logger(), "Hand-eye calibration capture (%s):", client->get_service_name());
  RCLCPP_INFO(node->get_logger(), "  Success: %s", result->success ? "true" : "false");
  RCLCPP_INFO(
    node->get_logger(), "  Message: %s",
    result->message.empty() ? "" : (R"(""")" + result->message + R"(""")").c_str());
  RCLCPP_INFO(node->get_logger(), "  Capture handle: %d", result->capture_handle);

  if (
    result->detection_result_calibration_board.status !=
    zivid_interfaces::msg::DetectionResultCalibrationBoard::STATUS_NOT_SET) {
    print_detection_result_calibration_board(node, result->detection_result_calibration_board);
  }

  if (!result->detection_result_fiducial_markers.detected_markers.empty()) {
    print_detection_result_fiducial_markers(node, result->detection_result_fiducial_markers);
  }
  return result->success;
}

bool request_calibration_and_print_response(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<rclcpp::Client<zivid_interfaces::srv::HandEyeCalibrationCalibrate>> &
    client,
  const CalibrationConfiguration configuration)
{
  RCLCPP_INFO(node->get_logger(), "--- Starting hand-eye calibration ---");

  using SrvRequest = zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request;
  auto request = std::make_shared<SrvRequest>();
  request->configuration =
    (configuration == CalibrationConfiguration::EyeToHand ? SrvRequest::EYE_TO_HAND
                                                          : SrvRequest::EYE_IN_HAND);

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the hand-eye calibration calibrate service");
  }
  auto result = future.get();
  RCLCPP_INFO(node->get_logger(), "Hand-eye calibration capture (%s):", client->get_service_name());
  RCLCPP_INFO(node->get_logger(), "  Success: %s", result->success ? "true" : "false");
  RCLCPP_INFO(
    node->get_logger(), "  Message: %s",
    result->message.empty() ? "" : (R"(""")" + result->message + R"(""")").c_str());
  RCLCPP_INFO(node->get_logger(), "  Transform: %s", format_transform(result->transform).c_str());
  RCLCPP_INFO(
    node->get_logger(), "  Residuals: %s",
    join_list_to_string(
      result->residuals,
      [](const zivid_interfaces::msg::HandEyeCalibrationResidual & residual) {
        return "{ rotation in deg: " + std::to_string(residual.rotation) +
               ", translation in m: " + std::to_string(residual.translation) + " }";
      })
      .c_str());
  return result->success;
}

geometry_msgs::msg::Pose get_simulated_robot_pose(double simulated_time)
{
  // This simulated response is for exposition only. On a real system, this method should be
  // replaced with the actual robot pose.
  const double yaw = simulated_time * 0.1;
  const double translation_x = std::sin(simulated_time);

  tf2::Transform transform;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  transform.setRotation(q);

  transform.setOrigin(tf2::Vector3(translation_x, 0.0, 0.0));

  const geometry_msgs::msg::Transform msg_transform = tf2::toMsg(transform);

  geometry_msgs::msg::Pose pose;
  pose.orientation = msg_transform.rotation;
  pose.position.x = msg_transform.translation.x;
  pose.position.y = msg_transform.translation.y;
  pose.position.z = msg_transform.translation.z;
  return pose;
}

void start_hand_eye_calibration(
  const rclcpp::Node::SharedPtr & node, const std::vector<int> & marker_ids,
  const std::string & working_directory)
{
  auto start_client = create_service_client<zivid_interfaces::srv::HandEyeCalibrationStart>(
    node, "hand_eye_calibration/start");

  RCLCPP_INFO(node->get_logger(), "--- Starting hand-eye calibration ---");

  if (!request_start(node, start_client, marker_ids, working_directory)) {
    fatal_error(node->get_logger(), "Could not start hand-eye calibration. Aborting.");
  }

  RCLCPP_INFO(node->get_logger(), "--- Setting capture settings ---");
  set_settings(node);
}

void perform_captures_for_hand_eye_calibration(const rclcpp::Node::SharedPtr & node)
{
  auto capture_client = create_service_client<zivid_interfaces::srv::HandEyeCalibrationCapture>(
    node, "hand_eye_calibration/capture");

  constexpr int wait_seconds = 5;
  constexpr int num_successful_captures_target = 6;
  RCLCPP_INFO_STREAM(
    node->get_logger(), "--- Starting captures, will proceed until " +
                          std::to_string(num_successful_captures_target) +
                          " captures have completed with a detected calibration object ---");

  bool sleep_before_capture = false;
  for (int num_successful_captures = 0; num_successful_captures < num_successful_captures_target;) {
    if (sleep_before_capture) {
      RCLCPP_INFO_STREAM(
        node->get_logger(), "--- Waiting for " + std::to_string(wait_seconds) +
                              " seconds before taking the next capture ---");
      rclcpp::spin_until_future_complete(
        node, std::promise<bool>().get_future(), std::chrono::seconds(wait_seconds));
    } else {
      sleep_before_capture = true;
    }

    const double simulated_time = 0.5 * static_cast<double>(num_successful_captures);
    const auto robot_pose = get_simulated_robot_pose(simulated_time);

    const bool success = request_capture_and_print_response(node, capture_client, robot_pose);
    if (success) {
      num_successful_captures += 1;
    }
  }

  RCLCPP_INFO(node->get_logger(), "--- Captures complete ---");
}

void perform_hand_eye_calibration(
  const rclcpp::Node::SharedPtr & node, const CalibrationConfiguration configuration)
{
  RCLCPP_INFO(node->get_logger(), "--- Performing hand-eye calibration with captured data ---");
  auto calibrate_client = create_service_client<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
    node, "hand_eye_calibration/calibrate");

  request_calibration_and_print_response(node, calibrate_client, configuration);

  RCLCPP_INFO(node->get_logger(), "--- Calibration complete ---");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sample_hand_eye_calibration");
  set_srgb(node);

  const auto configuration =
    get_parameter_enum(*node, "configuration", calibration_configuration_map);
  const auto marker_ids = sanitize_marker_ids(
    node->get_logger(),
    node->declare_parameter<std::vector<int64_t>>("marker_ids", {}, read_only_parameter));
  const auto working_directory =
    node->declare_parameter<std::string>("working_directory", "", read_only_parameter);

  start_hand_eye_calibration(node, marker_ids, working_directory);

  perform_captures_for_hand_eye_calibration(node);

  perform_hand_eye_calibration(node, configuration);

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
