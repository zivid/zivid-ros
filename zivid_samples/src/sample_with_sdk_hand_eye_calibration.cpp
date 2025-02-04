#include <Zivid/Application.h>
#include <Zivid/Calibration/Detector.h>
#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Calibration/Pose.h>
#include <Zivid/Exception.h>
#include <rclcpp/version.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <filesystem>
#include <map>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <zivid_interfaces/srv/capture_and_save.hpp>

/*
 * This sample shows how to perform hand-eye calibration using the Zivid camera. It implements a
 * node that communicates with the zivid camera driver in a separate node. The hand-eye calibration
 * is performed locally by using the Zivid SDK directly from the sample node. The sample expects
 * files saved on the camera node to be directly accessible from the sample node.
 */

static const auto read_only_parameter =
  rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);

#if RCLCPP_VERSION_MAJOR >= 17
// rclcpp 17.0 changed the QoS signature of `node->create_client` and `node->create_service`.
static const auto default_rclcpp_services_qos = rclcpp::ServicesQoS{};
#else
static const auto default_rclcpp_services_qos = rmw_qos_profile_services_default;
#endif

[[noreturn]] void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

std::string to_string_filled(size_t i)
{
  return (std::stringstream{} << std::setw(2) << std::setfill('0') << i).str();
}

std::vector<int> sanitize_aruco_ids(
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
      fatal_error(logger, "Negative Aruco IDs are not allowed.");
    }
    if (value > std::numeric_limits<int>::max()) {
      fatal_error(logger, "Aruco ID " + std::to_string(value) + " is too large.");
    }
  }

  return {values.begin(), values.end()};
}

Zivid::Matrix4x4 transpose_matrix(const Zivid::Matrix4x4 & value)
{
  Zivid::Matrix4x4 transposed;
  for (size_t i = 0; i < Zivid::Matrix4x4::rows; ++i) {
    for (size_t j = 0; j < Zivid::Matrix4x4::cols; ++j) {
      transposed(i, j) = value(j, i);
    }
  }
  return transposed;
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
  RCLCPP_INFO(node.get_logger(), "%s", ("Got parameter " + name + ": " + value).c_str());
  return it->second;
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

enum class HandEyeCalibrationMode
{
  EyeInHand,
  EyeToHand,
};

enum class HandEyeCalibrationObject
{
  CalibrationBoard,
  ArucoMarkers,
};

const std::map<std::string, HandEyeCalibrationMode> hand_eye_calibration_mode_map = {
  {"eye_in_hand", HandEyeCalibrationMode::EyeInHand},
  {"eye_to_hand", HandEyeCalibrationMode::EyeToHand},
};

class HandEyeCalibrationNode : public rclcpp::Node
{
public:
  HandEyeCalibrationNode()
  : Node("hand_eye_calibration_node"),
    m_save_directory{
      std::filesystem::temp_directory_path() / "zivid_ros_sample_with_sdk_hand_eye_calibration"},
    m_calibration_mode{get_parameter_enum(*this, "configuration", hand_eye_calibration_mode_map)},
    m_aruco_ids{sanitize_aruco_ids(
      get_logger(), declare_parameter<std::vector<int64_t>>("aruco_ids", {}, read_only_parameter))},
    m_calibration_object{
      m_aruco_ids.empty() ? HandEyeCalibrationObject::CalibrationBoard
                          : HandEyeCalibrationObject::ArucoMarkers},
    m_world_frame_id{declare_parameter<std::string>("world_frame_id", "map", read_only_parameter)},
    m_robot_frame_id{
      declare_parameter<std::string>("robot_frame_id", "robot_end_effector", read_only_parameter)},
    m_tf_buffer{std::make_unique<tf2_ros::Buffer>(get_clock())},
    m_tf_listener{std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer, this)},
    m_callback_group{create_callback_group(rclcpp::CallbackGroupType::Reentrant)},
    m_capture_and_save_client{create_client<zivid_interfaces::srv::CaptureAndSave>(
      "capture_and_save", default_rclcpp_services_qos, m_callback_group)},
    m_capture_service{create_service<std_srvs::srv::Trigger>(
      "hand_eye_sample_with_sdk_capture",
      std::bind(
        &HandEyeCalibrationNode::capture_callback, this, std::placeholders::_1,
        std::placeholders::_2),
      default_rclcpp_services_qos, m_callback_group)},
    m_calibrate_service{create_service<std_srvs::srv::Trigger>(
      "hand_eye_sample_with_sdk_calibrate",
      std::bind(
        &HandEyeCalibrationNode::calibrate_callback, this, std::placeholders::_1,
        std::placeholders::_2),
      default_rclcpp_services_qos, m_callback_group)}
  {
    RCLCPP_INFO(get_logger(), "Node %s started", get_name());
    std::filesystem::create_directory(m_save_directory);

    while (!m_capture_and_save_client->wait_for_service(std::chrono::seconds(3))) {
      if (!rclcpp::ok()) {
        fatal_error(get_logger(), "Client interrupted while waiting for service to appear.");
      }
      RCLCPP_INFO(get_logger(), "Waiting for the capture_and_save service to appear...");
    }

    RCLCPP_INFO(
      get_logger(),
      "Hand-eye calibration service started. Configured for %s calibration using %s%s.",
      m_calibration_mode == HandEyeCalibrationMode::EyeToHand ? "eye_to_hand" : "eye_in_hand",
      m_calibration_object == HandEyeCalibrationObject::CalibrationBoard
        ? "a calibration board"
        : "Aruco markers with IDs ",
      join_list_to_string(m_aruco_ids, [](int id) { return std::to_string(id); }).c_str());
  }

  std::string get_instructions() const
  {
    return "  1. Repeat the following for a desired number of captures, e.g. 10-20:\n"
           "    a. Position the robot with a suitable pose for the calibration object.\n"
           "    b. Ensure that the pose of the robot end-effector is reflected by the tf2 frame '" +
           m_robot_frame_id + "' with a valid transform from the world frame '" + m_world_frame_id +
           "'.\n"
           "    c. Call the '" +
           m_capture_service->get_service_name() +
           "' service with a Trigger request to initiate a capture with the current robot pose.\n"
           "  2. Call the '" +
           m_calibrate_service->get_service_name() +
           "' service with a Trigger request to perform the calibration using all the previous "
           "captures.\n"
           "The captures and robot poses will be saved to the directory: " +
           m_save_directory.string();
  }

private:
  struct DetectCalibrationObjectResult
  {
    bool success;
    std::string message;
  };

  void capture_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> & /*request*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    RCLCPP_INFO(get_logger(), "Received %s request", m_capture_service->get_service_name());

    const size_t index = input.size() + 1;
    const std::string file_path_zdf = m_save_directory / ("img" + to_string_filled(index) + ".zdf");
    const std::string file_path_pose =
      m_save_directory / ("pos" + to_string_filled(index) + ".yaml");
    RCLCPP_INFO(get_logger(), "-- Starting hand-eye calibration capture %zu --", index);

    const Zivid::Calibration::Pose pose = read_and_save_robot_transform(file_path_pose);

    run_capture_and_save(file_path_zdf);

    RCLCPP_INFO(get_logger(), "Loading frame from path %s", file_path_zdf.c_str());
    Zivid::Frame frame{file_path_zdf};

    const auto detect_result = detect_calibration_object(pose, frame);
    response->success = detect_result.success;
    response->message = detect_result.message;

    RCLCPP_INFO(
      get_logger(),
      "-- End of hand-eye calibration capture %zu --\n"
      "Add more input captures with a new Trigger request to %s, "
      "or attempt a hand-eye calibration with a Trigger request to %s",
      index, m_capture_service->get_service_name(), m_calibrate_service->get_service_name());
  }

  void calibrate_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> & /*request*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response) const
  {
    RCLCPP_INFO(get_logger(), "Received %s request", m_calibrate_service->get_service_name());
    RCLCPP_INFO(
      get_logger(), "-- Starting hand-eye calibration using %zu capture(s) --", input.size());

    try {
      Zivid::Calibration::HandEyeOutput calibration_output = [&] {
        switch (m_calibration_mode) {
          case HandEyeCalibrationMode::EyeInHand:
            return Zivid::Calibration::calibrateEyeInHand(input);
          case HandEyeCalibrationMode::EyeToHand:
            return Zivid::Calibration::calibrateEyeToHand(input);
          default:
            fatal_error(get_logger(), "Unhandled m_calibration_mode");
        }
      }();
      response->success = calibration_output.valid();
      response->message = calibration_output.toString();
    } catch (const Zivid::Exception & e) {
      response->success = false;
      response->message = "Got exception during hand-eye calibration: " + toString(e);
    }

    RCLCPP_INFO(
      get_logger(), "Calibration %s. %s", response->success ? "succeeded" : "failed",
      response->message.c_str());

    RCLCPP_INFO(
      get_logger(),
      "-- Ended hand-eye calibration using %zu capture(s) --\n"
      "Add more input captures with a new Trigger request to %s, "
      "or try a hand-eye calibration with a Trigger request to %s",
      input.size(), m_capture_service->get_service_name(), m_calibrate_service->get_service_name());
  }

  void run_capture_and_save(const std::string & file_path_zdf) const
  {
    auto capture_and_save_request =
      std::make_shared<zivid_interfaces::srv::CaptureAndSave::Request>();
    capture_and_save_request->file_path = file_path_zdf;
    RCLCPP_INFO(
      get_logger(), "Sending capture_and_save request with file path: %s",
      capture_and_save_request->file_path.c_str());
    auto capture_and_save_response =
      m_capture_and_save_client->async_send_request(capture_and_save_request);

    RCLCPP_INFO(get_logger(), "Waiting for capture_and_save response");
    auto capture_and_save_result = capture_and_save_response.get();
    if (!capture_and_save_result->success) {
      fatal_error(
        get_logger(),
        "Capture and save operation was unsuccessful: " + capture_and_save_result->message);
    }
    RCLCPP_INFO(get_logger(), "Got capture_and_save success response");
  }

  Zivid::Calibration::Pose read_and_save_robot_transform(const std::string & file_path_pose) const
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    try {
      transform_msg =
        m_tf_buffer->lookupTransform(m_world_frame_id, m_robot_frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      fatal_error(
        get_logger(), "Failed to lookup the transform between world frame '" + m_world_frame_id +
                        "' and robot frame '" + m_robot_frame_id + "': " + ex.what());
    }

    tf2::Transform quat_tf;
    tf2::fromMsg(transform_msg.transform, quat_tf);
    std::array<tf2Scalar, 16> tf2_transform = {};
    quat_tf.getOpenGLMatrix(tf2_transform.data());

    const auto transform =
      transpose_matrix(Zivid::Matrix4x4{tf2_transform.begin(), tf2_transform.end()});
    const Zivid::Calibration::Pose pose{transform};
    RCLCPP_INFO(
      get_logger(), "Retrieved robot pose %s.\nSaving robot pose to file: %s",
      pose.toString().c_str(), file_path_pose.c_str());
    transform.save(file_path_pose);
    return pose;
  }

  DetectCalibrationObjectResult detect_calibration_object(
    const Zivid::Calibration::Pose & pose, const Zivid::Frame & frame)
  {
    DetectCalibrationObjectResult result = {};
    switch (m_calibration_object) {
      case HandEyeCalibrationObject::CalibrationBoard: {
        RCLCPP_INFO(get_logger(), "Detecting calibration board");
        const auto detection_result = Zivid::Calibration::detectCalibrationBoard(frame);
        if (detection_result.valid()) {
          input.emplace_back(pose, detection_result);
          result.success = true;
          result.message =
            "Calibration board detected with centroid at " + detection_result.centroid().toString();
          RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
        } else {
          result.success = false;
          result.message = "Calibration board not detected in the capture, please try again";
          RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
        }
      } break;
      case HandEyeCalibrationObject::ArucoMarkers: {
        RCLCPP_INFO(get_logger(), "Detecting aruco markers");
        const auto detection_result = Zivid::Calibration::detectMarkers(
          frame, m_aruco_ids, Zivid::Calibration::MarkerDictionary::aruco4x4_250);
        if (detection_result.valid()) {
          input.emplace_back(pose, detection_result);
          result.success = true;
          result.message = std::to_string(detection_result.detectedMarkers().size()) +
                           " Aruco marker(s) detected: " +
                           join_list_to_string(
                             detection_result.detectedMarkers(),
                             [](const Zivid::Calibration::MarkerShape & marker) {
                               return "{ id: " + std::to_string(marker.id()) + ", corner: " +
                                      marker.cornersInCameraCoordinates().front().toString() + "}";
                             });
          RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
        } else {
          result.success = false;
          result.message = "Calibration board not detected in the capture, please try again";
          RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
        }
      } break;
      default:
        fatal_error(get_logger(), "Unhandled m_calibration_object_type");
    }
    return result;
  }

  std::filesystem::path m_save_directory;
  HandEyeCalibrationMode m_calibration_mode;
  std::vector<int> m_aruco_ids;
  HandEyeCalibrationObject m_calibration_object;
  std::string m_world_frame_id;
  std::string m_robot_frame_id;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  rclcpp::CallbackGroup::SharedPtr m_callback_group;
  rclcpp::Client<zivid_interfaces::srv::CaptureAndSave>::SharedPtr m_capture_and_save_client;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_capture_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_calibrate_service;

  std::vector<Zivid::Calibration::HandEyeInput> input;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HandEyeCalibrationNode>();

  // A Zivid::Application must first be constructed to use any of the Zivid API.
  RCLCPP_INFO(node->get_logger(), "Starting Zivid::Application.");
  Zivid::Application zivid_application;

  set_settings(node);
  RCLCPP_INFO(node->get_logger(), "Hand-eye calibration node initialized.");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Spinning node.. Press Ctrl+C to abort.\n"
    "Instructions:\n"
    "%s",
    node->get_instructions().c_str());

  exec.spin();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
