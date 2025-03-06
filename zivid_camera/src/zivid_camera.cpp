// Copyright 2024 Zivid AS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Zivid AS nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <Zivid/Application.h>
#include <Zivid/Calibration/Detector.h>
#include <Zivid/Camera.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Exception.h>
#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Experimental/Calibration/InfieldCorrection.h>
#include <Zivid/Firmware.h>
#include <Zivid/Frame2D.h>
#include <Zivid/Image.h>
#include <Zivid/Settings2D.h>
#include <Zivid/Version.h>

#include <cstdint>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <zivid_camera/zivid_camera.hpp>

namespace
{
template <typename T>
struct DependentFalse : std::false_type
{
};

std::string infieldCorrectionDistancesToString(
  const std::vector<Zivid::Experimental::Calibration::InfieldCorrectionInput> & dataset)
{
  auto zValues = std::vector<float>(dataset.size());
  std::transform(dataset.begin(), dataset.end(), zValues.begin(), [](const auto & input) {
    return input.detectionResult().centroid().z;
  });
  std::sort(zValues.begin(), zValues.end());

  std::stringstream ss;
  ss << "Measured positions: [";
  for (size_t i = 0; i < zValues.size(); ++i) {
    ss << std::setprecision(1) << std::fixed << zValues.at(i) << "mm";
    if (i < zValues.size() - 1) {
      ss << ", ";
    }
  }
  ss << "].";

  if (!zValues.empty()) {
    ss << " Range covered: " << zValues.back() - zValues.front() << "mm.";
  }

  return ss.str();
}

struct InfieldCorrectionStatistics
{
  size_t numberOfCaptures;
  float currentTruenessError;
  float averageTruenessError;
  float maximumTruenessError;
};

InfieldCorrectionStatistics calculateInfieldCorrectionStatistics(
  const std::vector<Zivid::Experimental::Calibration::InfieldCorrectionInput> & dataset)
{
  if (dataset.empty()) {
    throw std::runtime_error("Empty dataset");
  }
  const auto dimensionTruenessErrors = [&] {
    std::vector<float> result;
    std::transform(
      dataset.begin(), dataset.end(), std::back_inserter(result), [](const auto & input) {
        return Zivid::Experimental::Calibration::verifyCamera(input).localDimensionTrueness();
      });
    return result;
  }();
  const float currentTruenessError = dimensionTruenessErrors.back();
  const float averageTruenessError =
    std::accumulate(cbegin(dimensionTruenessErrors), cend(dimensionTruenessErrors), 0.0F) /
    static_cast<float>(size(dimensionTruenessErrors));
  const float maximumTruenessError =
    *std::max_element(dimensionTruenessErrors.begin(), dimensionTruenessErrors.end());
  return InfieldCorrectionStatistics{
    dataset.size(), currentTruenessError, averageTruenessError, maximumTruenessError};
}

std::string infieldCorrectionEstimateToString(
  const InfieldCorrectionStatistics & statistics,
  const Zivid::Experimental::Calibration::AccuracyEstimate & accuracyEstimate)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "Number of captures: " << statistics.numberOfCaptures << "\n";
  ss << "Camera metrics:";
  ss << "\n  Current trueness error: " << 100.0F * statistics.currentTruenessError << "%.";
  if (statistics.numberOfCaptures > 1) {
    ss << "\n  Average trueness error: " << 100.0F * statistics.averageTruenessError << "%.";
    ss << "\n  Maximum trueness error: " << 100.0F * statistics.maximumTruenessError << "%.";
  }
  ss << "\n";
  ss << "Expected post-correction metrics:";
  ss << "\n  Dimension trueness error: " << 100.0F * accuracyEstimate.dimensionAccuracy()
     << "% or less.";
  ss << "\n  Optimized workspace (depth): " << static_cast<int>(std::round(accuracyEstimate.zMin()))
     << " - " << static_cast<int>(std::round(accuracyEstimate.zMax())) << " mm.";
  return ss.str();
}

sensor_msgs::msg::PointField createPointField(
  std::string name, uint32_t offset, uint8_t datatype, uint32_t count)
{
  sensor_msgs::msg::PointField point_field;
  point_field.name = name;
  point_field.offset = offset;
  point_field.datatype = datatype;
  point_field.count = count;
  return point_field;
}

bool bigEndian()
{
  union {
    uint32_t i;
    char c[4];
  } b = {0x01020304};
  return b.c[0] == 1;
}

template <class T>
void fillCommonMsgFields(
  T & msg, const std_msgs::msg::Header & header, std::size_t width, std::size_t height)
{
  msg.header = header;
  msg.height = static_cast<uint32_t>(height);
  msg.width = static_cast<uint32_t>(width);
  msg.is_bigendian = bigEndian();
}

sensor_msgs::msg::Image::SharedPtr makeImage(
  const std_msgs::msg::Header & header, const std::string & encoding, std::size_t width,
  std::size_t height)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  fillCommonMsgFields(*image, header, width, height);
  image->encoding = encoding;
  const auto bytes_per_pixel = static_cast<std::size_t>(
    sensor_msgs::image_encodings::numChannels(encoding) *
    sensor_msgs::image_encodings::bitDepth(encoding) / 8);
  image->step = static_cast<uint32_t>(bytes_per_pixel * width);
  return image;
}

template <typename ZividDataType>
sensor_msgs::msg::Image::SharedPtr makePointCloudImage(
  const Zivid::PointCloud & point_cloud, const std_msgs::msg::Header & header,
  const std::string & encoding)
{
  auto image = makeImage(header, encoding, point_cloud.width(), point_cloud.height());
  image->data.resize(image->step * image->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType *>(image->data.data()));
  return image;
}

rclcpp::QoS getQoSLatched(bool use_latched_publisher)
{
  rclcpp::QoS qos{rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)};
  if (use_latched_publisher) {
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  }
  return qos;
}

std::string toString(zivid_camera::CameraStatus camera_status)
{
  switch (camera_status) {
    case zivid_camera::CameraStatus::Connected:
      return "Connected";
    case zivid_camera::CameraStatus::Disconnected:
      return "Disconnected";
    case zivid_camera::CameraStatus::Idle:
      return "Idle";
    default:  // NOLINT(clang-diagnostic-covered-switch-default)
      throw std::runtime_error("Enum `camera_status` out of range.");
  }
}

template <typename ZividDataModel>
auto serializeZividDataModel(const ZividDataModel & dm)
{
  return dm.serialize();
}

template <typename ZividDataModel>
auto deserializeZividDataModel(const std::string & serialized)
{
  return ZividDataModel::fromSerialized(serialized);
}

template <typename Function, typename ResponseSharedPtr, typename Logger>
void runFunctionAndCatchExceptions(
  Function && function, ResponseSharedPtr & response, const Logger & logger,
  const std::string & operation)
{
  try {
    response->success = true;
    function();
  } catch (const std::exception & exception) {
    const auto exception_message = Zivid::toString(exception);
    RCLCPP_ERROR_STREAM(
      logger, operation + " failed with exception: \"" << exception_message << "\"");
    response->success = false;
    response->message = exception_message;
  }
}

template <typename Logger>
[[noreturn]] void logErrorToLoggerAndThrowRuntimeException(
  const Logger & logger, const std::string & message)
{
  RCLCPP_ERROR(logger, "%s", message.c_str());
  throw std::runtime_error(message);
}
}  // namespace

namespace zivid_camera
{

namespace ParamNames
{
constexpr auto serial_number = "serial_number";
constexpr auto file_camera_path = "file_camera_path";
constexpr auto frame_id = "frame_id";
}  // namespace ParamNames

template <typename SettingsType>
class CaptureSettingsController
{
public:
  explicit CaptureSettingsController(rclcpp::Node & node)
  : node_(node),
    file_path_param_{baseName() + std::string{"_file_path"}},
    yaml_string_param_{baseName() + std::string{"_yaml"}}
  {
    for (const auto & param : {yaml_string_param_, file_path_param_}) {
      node_.declare_parameter<std::string>(param, "");
    }
  }

  SettingsType currentSettings() const
  {
    if (cached_settings_.has_value()) {
      RCLCPP_DEBUG_STREAM(node_.get_logger(), "Using cached settings");
      return *cached_settings_;
    }

    const auto settings_file_path = node_.get_parameter(file_path_param_).as_string();
    const auto settings_yaml = node_.get_parameter(yaml_string_param_).as_string();

    if (!settings_file_path.empty() && !settings_yaml.empty()) {
      logErrorToLoggerAndThrowRuntimeException(
        node_.get_logger(),
        "Both '" + file_path_param_ + "' and '" + yaml_string_param_ +
          "' parameters are non-empty! Please set only one of these parameters.");
    } else if (settings_file_path.empty() && settings_yaml.empty()) {
      logErrorToLoggerAndThrowRuntimeException(
        node_.get_logger(), "Both '" + file_path_param_ + "' and '" + yaml_string_param_ +
                              "' parameters are empty! Please set one of these parameters.");
    }

    if (!settings_yaml.empty()) {
      RCLCPP_DEBUG_STREAM(node_.get_logger(), "Using settings from yml string");
      cached_settings_ = deserializeZividDataModel<SettingsType>(settings_yaml);
    } else {
      RCLCPP_DEBUG_STREAM(
        node_.get_logger(), "Using settings from file '" << settings_file_path << "'");
      cached_settings_ = SettingsType{settings_file_path};
    }

    return *cached_settings_;
  }

  void setSettings(const SettingsType & settings)
  {
    RCLCPP_DEBUG_STREAM(node_.get_logger(), "Setting settings from " << settings.name << " object");
    node_.set_parameter(rclcpp::Parameter{yaml_string_param_, serializeZividDataModel(settings)});
    node_.set_parameter(rclcpp::Parameter{file_path_param_, ""});
  }

  void onSetParameter(const std::string & parameterName)
  {
    if (
      cached_settings_.has_value() &&
      (parameterName == file_path_param_ || parameterName == yaml_string_param_)) {
      RCLCPP_DEBUG_STREAM(
        node_.get_logger(), "Resetting cached settings due to updated parameter " << parameterName);
      cached_settings_.reset();
    }
  }

private:
  constexpr auto baseName() const
  {
    if constexpr (std::is_same_v<SettingsType, Zivid::Settings>) {
      return "settings";
    } else if constexpr (std::is_same_v<SettingsType, Zivid::Settings2D>) {
      return "settings_2d";
    } else {
      static_assert(DependentFalse<SettingsType>::value, "Unhandled node type");
    }
  }

  rclcpp::Node & node_;
  std::string file_path_param_;
  std::string yaml_string_param_;
  mutable std::optional<SettingsType> cached_settings_;
};

struct InfieldCorrectionState
{
  bool started = false;
  std::vector<Zivid::Experimental::Calibration::InfieldCorrectionInput> dataset;
};

ZividCamera::ZividCamera(const rclcpp::NodeOptions & options)
: rclcpp::Node{"zivid_camera", options},
  zivid_{std::make_unique<Zivid::Application>(
    Zivid::Detail::createApplicationForWrapper(Zivid::Detail::EnvironmentInfo::Wrapper::ros2))},
  set_parameters_callback_handle_{this->add_on_set_parameters_callback(
    std::bind(&ZividCamera::setParametersCallback, this, std::placeholders::_1))}
{
  // Disable buffering on stdout
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  RCLCPP_INFO_STREAM(get_logger(), "Zivid ROS driver");
  RCLCPP_INFO(get_logger(), "The node's namespace is '%s'", get_namespace());
  RCLCPP_INFO_STREAM(get_logger(), "Running Zivid Core version " << ZIVID_CORE_VERSION);

  // The "set parameters" callback references both controllers so the pointers must be initialized
  // before the callback is added. This is because constructing the controllers will cause the "set
  // parameters" event to fire, and we want to handle that first event in order to log it.
  // Therefore, we ensure the controllers are initialized to nullptr before initializing the
  // callback in the initializer list and then construct the controllers here afterward.
  settings_controller_ = std::make_unique<CaptureSettingsController<Zivid::Settings>>(*this);
  settings_2d_controller_ = std::make_unique<CaptureSettingsController<Zivid::Settings2D>>(*this);

  const auto serial_number = declare_parameter<std::string>(ParamNames::serial_number, "");

  frame_id_ = declare_parameter<std::string>(ParamNames::frame_id, "zivid_optical_frame");

  const auto file_camera_path = declare_parameter(ParamNames::file_camera_path, "");
  const bool file_camera_mode = !file_camera_path.empty();

  use_latched_publisher_for_points_xyz_ =
    declare_parameter<bool>("use_latched_publisher_for_points_xyz", false);

  use_latched_publisher_for_points_xyzrgba_ =
    declare_parameter<bool>("use_latched_publisher_for_points_xyzrgba", false);

  use_latched_publisher_for_color_image_ =
    declare_parameter<bool>("use_latched_publisher_for_color_image", false);

  use_latched_publisher_for_depth_image_ =
    declare_parameter<bool>("use_latched_publisher_for_depth_image", false);

  use_latched_publisher_for_snr_image_ =
    declare_parameter<bool>("use_latched_publisher_for_snr_image", false);

  use_latched_publisher_for_normals_xyz_ =
    declare_parameter<bool>("use_latched_publisher_for_normals_xyz", false);

  const bool update_firmware_automatically =
    declare_parameter<bool>("update_firmware_automatically", true);

  camera_ = std::make_unique<Zivid::Camera>([&]() {
    if (file_camera_mode) {
      RCLCPP_INFO(get_logger(), "Creating file camera from file '%s'", file_camera_path.c_str());
      return zivid_->createFileCamera(file_camera_path);
    }
    auto cameras = zivid_->cameras();
    RCLCPP_INFO_STREAM(get_logger(), cameras.size() << " camera(s) found");

    if (cameras.empty()) {
      logErrorAndThrowRuntimeException(
        "No cameras found. Ensure that the camera is connected to your PC.");
    } else if (!serial_number.empty()) {
      RCLCPP_INFO(
        get_logger(), "Searching for camera with serial number '%s' ...", serial_number.c_str());
      for (auto & c : cameras) {
        if (c.info().serialNumber() == Zivid::CameraInfo::SerialNumber(serial_number)) {
          return c;
        }
      }
      logErrorAndThrowRuntimeException(
        "No camera found with serial number '" + serial_number + "'");
    }
    RCLCPP_INFO(get_logger(), "Selecting first available camera");
    for (auto & c : cameras) {
      if (c.state().isAvailable()) {
        return c;
      }
    }
    logErrorAndThrowRuntimeException(
      "No available cameras found! Use ZividListCameras or ZividStudio to see all connected "
      "cameras and their status.");
  }());

  if (!Zivid::Firmware::isUpToDate(*camera_)) {
    if (update_firmware_automatically) {
      RCLCPP_INFO(
        get_logger(),
        "The camera firmware is not up-to-date, and update_firmware_automatically is true, "
        "starting update");
      Zivid::Firmware::update(
        *camera_, [logger = get_logger()](double progress, const std::string & state) {
          RCLCPP_INFO(logger, "  [%.0f%%] %s", progress, state.c_str());
        });
      RCLCPP_INFO(get_logger(), "Firmware update completed");
    } else {
      logErrorAndThrowRuntimeException(
        "The firmware on camera '" + camera_->info().serialNumber().value() +
        "' is not up to date. The launch parameter update_firmware_automatically "
        "is set to false. Please update the firmware on this camera manually.");
    }
  }

  RCLCPP_INFO_STREAM(get_logger(), *camera_);
  if (!file_camera_mode) {
    RCLCPP_INFO_STREAM(
      get_logger(), "Connecting to camera '" << camera_->info().serialNumber() << "'");
    camera_->connect();
  }
  RCLCPP_INFO_STREAM(
    get_logger(), "Connected to camera '" << camera_->info().serialNumber() << "'");
  setCameraStatus(CameraStatus::Connected);

  camera_connection_keepalive_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(10),
    std::bind(&ZividCamera::onCameraConnectionKeepAliveTimeout, this));

  RCLCPP_INFO(get_logger(), "Advertising topics");

  points_xyz_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "points/xyz", getQoSLatched(use_latched_publisher_for_points_xyz_));

  points_xyzrgba_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "points/xyzrgba", getQoSLatched(use_latched_publisher_for_points_xyzrgba_));

  normals_xyz_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "normals/xyz", getQoSLatched(use_latched_publisher_for_normals_xyz_));

  color_image_publisher_ = image_transport::create_camera_publisher(
    this, "color/image_color",
    getQoSLatched(use_latched_publisher_for_color_image_).get_rmw_qos_profile());
  depth_image_publisher_ = image_transport::create_camera_publisher(
    this, "depth/image",
    getQoSLatched(use_latched_publisher_for_depth_image_).get_rmw_qos_profile());
  snr_image_publisher_ = image_transport::create_camera_publisher(
    this, "snr/image", getQoSLatched(use_latched_publisher_for_snr_image_).get_rmw_qos_profile());

  RCLCPP_INFO(get_logger(), "Advertising services");

  using namespace std::placeholders;

  camera_info_model_name_service_ = create_service<zivid_interfaces::srv::CameraInfoModelName>(
    "camera_info/model_name",
    std::bind(&ZividCamera::cameraInfoModelNameServiceHandler, this, _1, _2, _3));

  camera_info_serial_number_service_ =
    create_service<zivid_interfaces::srv::CameraInfoSerialNumber>(
      "camera_info/serial_number",
      std::bind(&ZividCamera::cameraInfoSerialNumberServiceHandler, this, _1, _2, _3));

  is_connected_service_ = create_service<zivid_interfaces::srv::IsConnected>(
    "is_connected", std::bind(&ZividCamera::isConnectedServiceHandler, this, _1, _2, _3));

  capture_service_ = create_service<std_srvs::srv::Trigger>(
    "capture", std::bind(&ZividCamera::captureServiceHandler, this, _1, _2, _3));

  capture_and_save_service_ = create_service<zivid_interfaces::srv::CaptureAndSave>(
    "capture_and_save", std::bind(&ZividCamera::captureAndSaveServiceHandler, this, _1, _2, _3));

  capture_2d_service_ = create_service<std_srvs::srv::Trigger>(
    "capture_2d", std::bind(&ZividCamera::capture2DServiceHandler, this, _1, _2, _3));

  capture_assistant_suggest_settings_service_ =
    create_service<zivid_interfaces::srv::CaptureAssistantSuggestSettings>(
      "capture_assistant/suggest_settings",
      std::bind(&ZividCamera::captureAssistantSuggestSettingsServiceHandler, this, _1, _2, _3));

  infield_correction_state_ = std::make_unique<InfieldCorrectionState>();
  infield_correction_read_ = create_service<std_srvs::srv::Trigger>(
    "infield_correction/read", std::bind(&ZividCamera::infieldCorrectionRead, this, _1, _2, _3));
  infield_correction_reset_ = create_service<std_srvs::srv::Trigger>(
    "infield_correction/reset", std::bind(&ZividCamera::infieldCorrectionReset, this, _1, _2, _3));
  infield_correction_verify_ = create_service<zivid_interfaces::srv::InfieldCorrectionVerify>(
    "infield_correction/verify",
    std::bind(&ZividCamera::infieldCorrectionVerify, this, _1, _2, _3));
  infield_correction_remove_last_capture_ = create_service<std_srvs::srv::Trigger>(
    "infield_correction/remove_last_capture",
    std::bind(&ZividCamera::infieldCorrectionRemoveLastCapture, this, _1, _2, _3));
  infield_correction_start_ = create_service<std_srvs::srv::Trigger>(
    "infield_correction/start", std::bind(&ZividCamera::infieldCorrectionStart, this, _1, _2, _3));
  infield_correction_capture_ = create_service<std_srvs::srv::Trigger>(
    "infield_correction/capture",
    std::bind(&ZividCamera::infieldCorrectionCapture, this, _1, _2, _3));
  infield_correction_compute_ = create_service<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute",
    std::bind(&ZividCamera::infieldCorrectionCompute, this, _1, _2, _3));
  infield_correction_compute_and_write_ =
    create_service<zivid_interfaces::srv::InfieldCorrectionCompute>(
      "infield_correction/compute_and_write",
      std::bind(&ZividCamera::infieldCorrectionComputeAndWrite, this, _1, _2, _3));

  RCLCPP_INFO(get_logger(), "Zivid camera driver is now ready!");
}

ZividCamera::~ZividCamera() = default;

Zivid::Application & ZividCamera::zividApplication() { return *zivid_; }

void ZividCamera::onCameraConnectionKeepAliveTimeout()
{
  RCLCPP_DEBUG_STREAM(get_logger(), __func__);
  try {
    reconnectToCameraIfNecessary();
  } catch (const std::exception & e) {
    RCLCPP_INFO(get_logger(), "%s failed with exception '%s'", __func__, e.what());
  }
}

void ZividCamera::reconnectToCameraIfNecessary()
{
  RCLCPP_DEBUG_STREAM(get_logger(), __func__);

  const auto state = camera_->state();
  if (state.isConnected().value()) {
    setCameraStatus(CameraStatus::Connected);
  } else {
    setCameraStatus(CameraStatus::Disconnected);

    if (state.isAvailable().value()) {
      RCLCPP_INFO_STREAM(
        get_logger(), "The camera '" << camera_->info().serialNumber()
                                     << "' is not connected but is available. Re-connecting ...");
      camera_->connect();
      RCLCPP_INFO(get_logger(), "Successfully reconnected to camera!");
      setCameraStatus(CameraStatus::Connected);
    } else {
      RCLCPP_INFO_STREAM(
        get_logger(),
        "The camera '" << camera_->info().serialNumber() << "' is not connected nor available.");
    }
  }
}

void ZividCamera::setCameraStatus(CameraStatus camera_status)
{
  if (camera_status_ != camera_status) {
    std::stringstream ss;
    ss << "Camera status changed to " << toString(camera_status) << " (was "
       << toString(camera_status_) << ")";
    if (camera_status == CameraStatus::Connected) {
      RCLCPP_INFO_STREAM(get_logger(), ss.str());
    } else {
      RCLCPP_WARN_STREAM(get_logger(), ss.str());
    }
    camera_status_ = camera_status;
  }
}

rcl_interfaces::msg::SetParametersResult ZividCamera::setParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    RCLCPP_DEBUG_STREAM(
      get_logger(), "Set parameter '" << param.get_name() << "' (" << param.get_type_name()
                                      << ") to '" << param.value_to_string() << "'");
    if (settings_controller_) {
      settings_controller_->onSetParameter(param.get_name());
    }
    if (settings_2d_controller_) {
      settings_2d_controller_->onSetParameter(param.get_name());
    }
  }
  return result;
}

void ZividCamera::cameraInfoModelNameServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Request>,
  std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);
  response->model_name = camera_->info().modelName().toString();
}

void ZividCamera::cameraInfoSerialNumberServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Request>,
  std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);
  response->serial_number = camera_->info().serialNumber().toString();
}

void ZividCamera::captureServiceHandler(
  const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      const auto settings = settings_controller_->currentSettings();
      invokeCaptureAndPublishFrame(settings);
    },
    response, get_logger(), "Capture");
}

void ZividCamera::captureAndSaveServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::CaptureAndSave::Request> request,
  std::shared_ptr<zivid_interfaces::srv::CaptureAndSave::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      const auto settings = settings_controller_->currentSettings();
      const auto frame = invokeCaptureAndPublishFrame(settings);
      const auto destination_path = request->file_path;
      RCLCPP_INFO(get_logger(), "Saving frame to '%s'", destination_path.c_str());
      frame.save(destination_path);
    },
    response, get_logger(), "CaptureAndSave");
}

void ZividCamera::capture2DServiceHandler(
  const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  serviceHandlerHandleCameraConnectionLoss();

  runFunctionAndCatchExceptions(
    [&]() {
      const auto settings2D = settings_2d_controller_->currentSettings();
      auto frame2D = camera_->capture(settings2D);
      if (shouldPublishColorImg()) {
        const auto header = makeHeader();
        auto image = frame2D.imageRGBA();
        const auto intrinsics = Zivid::Experimental::Calibration::intrinsics(*camera_);
        const auto camera_info = makeCameraInfo(header, image.width(), image.height(), intrinsics);
        publishColorImage(header, camera_info, image);
      }
    },
    response, get_logger(), "Capture2D");
}

void ZividCamera::captureAssistantSuggestSettingsServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request> request,
  std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  serviceHandlerHandleCameraConnectionLoss();

  runFunctionAndCatchExceptions(
    [&]() {
      using SuggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters;

      const auto max_capture_time =
        rclcpp::Duration{request->max_capture_time}.to_chrono<std::chrono::milliseconds>();
      const auto ambient_light_frequency = [this, &request]() {
        using RosRequestTypes = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request;
        switch (request->ambient_light_frequency) {
          case RosRequestTypes::AMBIENT_LIGHT_FREQUENCY_NONE:
            return SuggestSettingsParameters::AmbientLightFrequency::none;
          case RosRequestTypes::AMBIENT_LIGHT_FREQUENCY_50HZ:
            return SuggestSettingsParameters::AmbientLightFrequency::hz50;
          case RosRequestTypes::AMBIENT_LIGHT_FREQUENCY_60HZ:
            return SuggestSettingsParameters::AmbientLightFrequency::hz60;
          default:
            logErrorAndThrowRuntimeException(
              "Unhandled AMBIENT_LIGHT_FREQUENCY value: " +
              std::to_string(request->ambient_light_frequency));
        }
      }();

      SuggestSettingsParameters suggest_settings_parameters{
        SuggestSettingsParameters::MaxCaptureTime{max_capture_time}, ambient_light_frequency};

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Getting suggested settings using parameters: " << suggest_settings_parameters);

      const auto suggested_settings =
        Zivid::CaptureAssistant::suggestSettings(*camera_, suggest_settings_parameters);

      RCLCPP_INFO_STREAM(
        get_logger(), "CaptureAssistant::suggestSettings returned "
                        << suggested_settings.acquisitions().size() << " acquisitions");

      settings_controller_->setSettings(suggested_settings);
      response->suggested_settings = serializeZividDataModel(suggested_settings);
    },
    response, get_logger(), "CaptureAssistantSuggestSettings");
}

void ZividCamera::serviceHandlerHandleCameraConnectionLoss()
{
  reconnectToCameraIfNecessary();
  if (camera_status_ != CameraStatus::Connected) {
    logErrorAndThrowRuntimeException(
      "Unable to capture since the camera is not connected. Please re-connect the camera and "
      "try again.");
  }
}

void ZividCamera::isConnectedServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::IsConnected::Request>,
  std::shared_ptr<zivid_interfaces::srv::IsConnected::Response> response)
{
  response->is_connected = camera_status_ == CameraStatus::Connected;
}

void ZividCamera::infieldCorrectionRead(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      if (Zivid::Experimental::Calibration::hasCameraCorrection(*camera_)) {
        const auto timestamp =
          Zivid::Experimental::Calibration::cameraCorrectionTimestamp(*camera_);
        const auto time = std::chrono::system_clock::to_time_t(timestamp);
        std::stringstream ss;
        ss << "Timestamp of current camera correction: "
           << std::put_time(std::gmtime(&time), "%FT%TZ");
        response->message = ss.str();
      } else {
        response->message = "This camera has no in-field correction written to it.";
      }
    },
    response, get_logger(), "InfieldCorrectionRead");
}

void ZividCamera::infieldCorrectionReset(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() { Zivid::Experimental::Calibration::resetCameraCorrection(*camera_); }, response,
    get_logger(), "InfieldCorrectionReset");
}

void ZividCamera::infieldCorrectionVerify(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionVerify::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionVerify::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      RCLCPP_DEBUG_STREAM(get_logger(), "Capturing calibration object");
      const auto detectionResult = Zivid::Experimental::Calibration::detectFeaturePoints(*camera_);
      const auto input = Zivid::Experimental::Calibration::InfieldCorrectionInput{detectionResult};
      if (!input.valid()) {
        response->message = "Invalid capture! Feedback: " + input.statusDescription();
        response->success = false;
        return;
      }
      RCLCPP_DEBUG_STREAM(get_logger(), "Successful measurement, starting verification");
      const auto verification = Zivid::Experimental::Calibration::verifyCamera(input);
      const auto centroid = detectionResult.centroid();
      std::stringstream ss;
      ss << "Successful measurement at " << detectionResult.centroid()
         << ". Estimated dimension trueness error at measured position: " << std::setprecision(2)
         << std::fixed << 100.0F * verification.localDimensionTrueness() << "%";
      response->local_dimension_trueness = verification.localDimensionTrueness();
      response->position.x = static_cast<double>(centroid.x);
      response->position.y = static_cast<double>(centroid.y);
      response->position.z = static_cast<double>(centroid.z);
      response->message = ss.str();
    },
    response, get_logger(), "InfieldCorrectionVerify");
}

void ZividCamera::infieldCorrectionRemoveLastCapture(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      if (infield_correction_state_->dataset.empty()) {
        throw std::runtime_error("Infield correction dataset is empty");
      }
      infield_correction_state_->dataset.pop_back();
    },
    response, get_logger(), "InfieldCorrectionRemoveLastCapture");
}

void ZividCamera::infieldCorrectionStart(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      *infield_correction_state_ = {};
      infield_correction_state_->started = true;
    },
    response, get_logger(), "InfieldCorrectionStart");
}

void ZividCamera::infieldCorrectionCapture(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      if (!infield_correction_state_->started) {
        throw std::runtime_error(
          "Infield correction not started. Please call the infield correction start service before "
          "capturing.");
      }
      const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(*camera_);
      const auto input = Zivid::Experimental::Calibration::InfieldCorrectionInput{detectionResult};
      if (input.valid()) {
        auto & dataset = infield_correction_state_->dataset;
        dataset.push_back(input);
        response->message = "Valid detection. Collected " + std::to_string(dataset.size()) +
                            " valid measurement(s) so far. " +
                            infieldCorrectionDistancesToString(dataset);
      } else {
        response->success = false;
        response->message = "Invalid detection. This measurement will not be used. Feedback: " +
                            input.statusDescription();
      }
    },
    response, get_logger(), "InfieldCorrectionCapture");
}

void ZividCamera::infieldCorrectionCompute(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      const auto & dataset = infield_correction_state_->dataset;
      // Set started & number of captures before computing, so that it is reported regardless of any exceptions.
      response->infield_correction_started = infield_correction_state_->started;
      response->number_of_captures = static_cast<int>(dataset.size());
      const auto correction = Zivid::Experimental::Calibration::computeCameraCorrection(dataset);
      const auto accuracyEstimate = correction.accuracyEstimate();
      const auto statistics = calculateInfieldCorrectionStatistics(dataset);
      response->current_trueness_error = statistics.currentTruenessError;
      response->average_trueness_error = statistics.averageTruenessError;
      response->maximum_trueness_error = statistics.maximumTruenessError;
      response->z_min = accuracyEstimate.zMin();
      response->z_max = accuracyEstimate.zMax();
      response->dimension_accuracy = accuracyEstimate.dimensionAccuracy();
      response->message = "Camera correction computed successfully.\n" +
                          infieldCorrectionEstimateToString(statistics, accuracyEstimate);
    },
    response, get_logger(), "InfieldCorrectionCompute");
}

void ZividCamera::infieldCorrectionComputeAndWrite(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      auto & dataset = infield_correction_state_->dataset;
      response->infield_correction_started = infield_correction_state_->started;
      response->number_of_captures = static_cast<int>(dataset.size());
      const auto correction = Zivid::Experimental::Calibration::computeCameraCorrection(dataset);
      RCLCPP_DEBUG(get_logger(), "Writing correction to camera");
      Zivid::Experimental::Calibration::writeCameraCorrection(*camera_, correction);
      const auto accuracyEstimate = correction.accuracyEstimate();
      const auto statistics = calculateInfieldCorrectionStatistics(dataset);
      response->current_trueness_error = statistics.currentTruenessError;
      response->average_trueness_error = statistics.averageTruenessError;
      response->maximum_trueness_error = statistics.maximumTruenessError;
      response->z_min = accuracyEstimate.zMin();
      response->z_max = accuracyEstimate.zMax();
      response->dimension_accuracy = accuracyEstimate.dimensionAccuracy();
      response->message = "Camera correction successfully written to camera.\n" +
                          infieldCorrectionEstimateToString(statistics, accuracyEstimate);
      // Clear the dataset after a successful write, the captures cannot be used again.
      dataset.clear();
    },
    response, get_logger(), "InfieldCorrectionComputeAndWrite");
}

void ZividCamera::publishFrame(const Zivid::Frame & frame)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);
  const bool publish_points_xyz = shouldPublishPointsXYZ();
  const bool publish_points_xyzrgba = shouldPublishPointsXYZRGBA();
  const bool publish_color_img = shouldPublishColorImg();
  const bool publish_depth_img = shouldPublishDepthImg();
  const bool publish_snr_img = shouldPublishSnrImg();
  const bool publish_normals_xyz = shouldPublishNormalsXYZ();

  if (
    publish_points_xyz || publish_points_xyzrgba || publish_color_img || publish_depth_img ||
    publish_snr_img || publish_normals_xyz) {
    const auto header = makeHeader();
    auto point_cloud = frame.pointCloud();

    // Transform point cloud from millimeters (Zivid SDK) to meter (ROS).
    const float scale = 0.001f;
    const auto transformation_matrix =
      Zivid::Matrix4x4{scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, 1};
    point_cloud.transform(transformation_matrix);

    if (publish_points_xyz) {
      publishPointCloudXYZ(header, point_cloud);
    }
    if (publish_points_xyzrgba) {
      publishPointCloudXYZRGBA(header, point_cloud);
    }
    if (publish_color_img || publish_depth_img || publish_snr_img) {
      const auto intrinsics = Zivid::Experimental::Calibration::intrinsics(*camera_);
      const auto camera_info =
        makeCameraInfo(header, point_cloud.width(), point_cloud.height(), intrinsics);

      if (publish_color_img) {
        publishColorImage(header, camera_info, point_cloud);
      }
      if (publish_depth_img) {
        publishDepthImage(header, camera_info, point_cloud);
      }
      if (publish_snr_img) {
        publishSnrImage(header, camera_info, point_cloud);
      }
    }
    if (publish_normals_xyz) {
      publishNormalsXYZ(header, point_cloud);
    }
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      __func__ << ": capture was called, but no subscribers active and 0 messages sent");
  }
}

bool ZividCamera::shouldPublishPointsXYZRGBA() const
{
  return points_xyzrgba_publisher_->get_subscription_count() > 0 ||
         use_latched_publisher_for_points_xyzrgba_;
}

bool ZividCamera::shouldPublishPointsXYZ() const
{
  return points_xyz_publisher_->get_subscription_count() > 0 ||
         use_latched_publisher_for_points_xyz_;
}

bool ZividCamera::shouldPublishColorImg() const
{
  return color_image_publisher_.getNumSubscribers() > 0 || use_latched_publisher_for_color_image_;
}

bool ZividCamera::shouldPublishDepthImg() const
{
  return depth_image_publisher_.getNumSubscribers() > 0 || use_latched_publisher_for_depth_image_;
}

bool ZividCamera::shouldPublishSnrImg() const
{
  return snr_image_publisher_.getNumSubscribers() > 0 || use_latched_publisher_for_snr_image_;
}

bool ZividCamera::shouldPublishNormalsXYZ() const
{
  return normals_xyz_publisher_->get_subscription_count() > 0 ||
         use_latched_publisher_for_normals_xyz_;
}

std_msgs::msg::Header ZividCamera::makeHeader()
{
  std_msgs::msg::Header header;
  header.stamp = get_clock()->now();
  header.frame_id = frame_id_;
  return header;
}

void ZividCamera::publishPointCloudXYZ(
  const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << points_xyz_publisher_->get_topic_name());

  // We are using the Zivid::XYZW type here for compatibility with the pcl::PointXYZ type, which
  // contains a padding float for performance reasons. We could use the "pcl_conversion" utility
  // functions to construct the PointCloud2 message. However, those are observed to add significant
  // overhead due to extra unnecessary copies of the data.
  using ZividDataType = Zivid::PointXYZW;
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(3);
  msg->fields.push_back(createPointField("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->is_dense = false;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType *>(msg->data.data()));
  points_xyz_publisher_->publish(std::move(msg));
}

void ZividCamera::publishPointCloudXYZRGBA(
  const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << points_xyzrgba_publisher_->get_topic_name());

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(4);
  msg->fields.push_back(createPointField("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("rgba", 12, sensor_msgs::msg::PointField::UINT32, 1));
  msg->is_dense = false;

  // Note that the "rgba" field is actually byte order "bgra" on little-endian systems. For this
  // reason we use the Zivid BGRA type.
  using ZividDataType = Zivid::PointXYZColorBGRA;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType *>(msg->data.data()));
  points_xyzrgba_publisher_->publish(std::move(msg));
}

void ZividCamera::publishColorImage(
  const std_msgs::msg::Header & header,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(
    get_logger(), "Publishing " << color_image_publisher_.getTopic() << " from point cloud");
  auto image =
    makePointCloudImage<Zivid::ColorRGBA>(point_cloud, header, sensor_msgs::image_encodings::RGBA8);
  color_image_publisher_.publish(image, camera_info);
}

void ZividCamera::publishColorImage(
  const std_msgs::msg::Header & header,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  const Zivid::Image<Zivid::ColorRGBA> & image)
{
  RCLCPP_INFO_STREAM(
    get_logger(), "Publishing " << color_image_publisher_.getTopic() << " from Image");
  auto msg = makeImage(header, sensor_msgs::image_encodings::RGBA8, image.width(), image.height());
  const auto uint8_ptr_begin = reinterpret_cast<const uint8_t *>(image.data());

#ifdef __clang__
#if __has_warning("-Wunsafe-buffer-usage")
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage"
#endif
#endif
  const auto uint8_ptr_end = reinterpret_cast<const uint8_t *>(image.data() + image.size());
#ifdef __clang__
#if __has_warning("-Wunsafe-buffer-usage")
#pragma clang diagnostic pop
#endif
#endif

  msg->data = std::vector<uint8_t>(uint8_ptr_begin, uint8_ptr_end);
  color_image_publisher_.publish(msg, camera_info);
}

void ZividCamera::publishDepthImage(
  const std_msgs::msg::Header & header,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << depth_image_publisher_.getTopic());
  auto image = makePointCloudImage<Zivid::PointZ>(
    point_cloud, header, sensor_msgs::image_encodings::TYPE_32FC1);
  depth_image_publisher_.publish(image, camera_info);
}

void ZividCamera::publishSnrImage(
  const std_msgs::msg::Header & header,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << snr_image_publisher_.getTopic());
  auto image =
    makePointCloudImage<Zivid::SNR>(point_cloud, header, sensor_msgs::image_encodings::TYPE_32FC1);
  snr_image_publisher_.publish(image, camera_info);
}

void ZividCamera::publishNormalsXYZ(
  const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << normals_xyz_publisher_->get_topic_name());

  using ZividDataType = Zivid::NormalXYZ;
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(3);
  msg->fields.push_back(createPointField("normal_x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("normal_y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("normal_z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->is_dense = false;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType *>(msg->data.data()));
  normals_xyz_publisher_->publish(std::move(msg));
}

sensor_msgs::msg::CameraInfo::ConstSharedPtr ZividCamera::makeCameraInfo(
  const std_msgs::msg::Header & header, std::size_t width, std::size_t height,
  const Zivid::CameraIntrinsics & intrinsics)
{
  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  msg->header = header;
  msg->width = static_cast<uint32_t>(width);
  msg->height = static_cast<uint32_t>(height);
  msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // k1, k2, t1, t2, k3
  const auto distortion = intrinsics.distortion();
  msg->d.resize(5);
  msg->d[0] = distortion.k1().value();
  msg->d[1] = distortion.k2().value();
  msg->d[2] = distortion.p1().value();
  msg->d[3] = distortion.p2().value();
  msg->d[4] = distortion.k3().value();

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  const auto camera_matrix = intrinsics.cameraMatrix();
  msg->k[0] = camera_matrix.fx().value();
  msg->k[2] = camera_matrix.cx().value();
  msg->k[4] = camera_matrix.fy().value();
  msg->k[5] = camera_matrix.cy().value();
  msg->k[8] = 1;

  // R (identity)
  msg->r[0] = 1;
  msg->r[4] = 1;
  msg->r[8] = 1;

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  msg->p[0] = camera_matrix.fx().value();
  msg->p[2] = camera_matrix.cx().value();
  msg->p[5] = camera_matrix.fy().value();
  msg->p[6] = camera_matrix.cy().value();
  msg->p[10] = 1;

  return msg;
}

Zivid::Frame ZividCamera::invokeCaptureAndPublishFrame(const Zivid::Settings & settings)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  serviceHandlerHandleCameraConnectionLoss();

  RCLCPP_INFO(get_logger(), "Capturing with %zd acquisition(s)", settings.acquisitions().size());
  RCLCPP_DEBUG_STREAM(get_logger(), settings);
  const auto frame = camera_->capture(settings);
  publishFrame(frame);
  return frame;
}

void ZividCamera::logErrorAndThrowRuntimeException(const std::string & message)
{
  logErrorToLoggerAndThrowRuntimeException(get_logger(), message);
}

}  // namespace zivid_camera

#include "rclcpp_components/register_node_macro.hpp"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wglobal-constructors"
#pragma clang diagnostic ignored "-Wexit-time-destructors"
#endif

RCLCPP_COMPONENTS_REGISTER_NODE(zivid_camera::ZividCamera)

#ifdef __clang__
#pragma clang diagnostic pop
#endif
