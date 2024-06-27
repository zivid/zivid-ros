#include "zivid_camera.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <dynamic_reconfigure/config_tools.h>

#include <Zivid/Firmware.h>
#include <Zivid/Frame2D.h>
#include <Zivid/Settings2D.h>
#include <Zivid/Version.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Experimental/Calibration.h>

#include <boost/algorithm/string.hpp>
#include <boost/predef.h>

#include <sstream>
#include <thread>
#include <cstdint>

namespace
{
sensor_msgs::PointField createPointField(std::string name, uint32_t offset, uint8_t datatype, uint32_t count)
{
  sensor_msgs::PointField point_field;
  point_field.name = name;
  point_field.offset = offset;
  point_field.datatype = datatype;
  point_field.count = count;
  return point_field;
}

bool big_endian()
{
  return BOOST_ENDIAN_BIG_BYTE;
}

template <class T>
void fillCommonMsgFields(T& msg, const std_msgs::Header& header, std::size_t width, std::size_t height)
{
  msg.header = header;
  msg.height = static_cast<uint32_t>(height);
  msg.width = static_cast<uint32_t>(width);
  msg.is_bigendian = big_endian();
}

sensor_msgs::ImagePtr makeImage(const std_msgs::Header& header, const std::string& encoding, std::size_t width,
                                std::size_t height)
{
  auto image = boost::make_shared<sensor_msgs::Image>();
  fillCommonMsgFields(*image, header, width, height);
  image->encoding = encoding;
  const auto bytes_per_pixel = static_cast<std::size_t>(sensor_msgs::image_encodings::numChannels(encoding) *
                                                        sensor_msgs::image_encodings::bitDepth(encoding) / 8);
  image->step = static_cast<uint32_t>(bytes_per_pixel * width);
  return image;
}

template <typename ZividDataType>
sensor_msgs::ImagePtr makePointCloudImage(const Zivid::PointCloud& point_cloud, const std_msgs::Header& header,
                                          const std::string& encoding)
{
  auto image = makeImage(header, encoding, point_cloud.width(), point_cloud.height());
  image->data.resize(image->step * image->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType*>(image->data.data()));
  return image;
}

std::string toString(zivid_camera::CameraStatus camera_status)
{
  switch (camera_status)
  {
    case zivid_camera::CameraStatus::Connected:
      return "Connected";
    case zivid_camera::CameraStatus::Disconnected:
      return "Disconnected";
    case zivid_camera::CameraStatus::Idle:
      return "Idle";
  }
  return "N/A";
}

Zivid::Application makeZividApplication()
{
#if ZIVID_CORE_VERSION_MAJOR >= 3 || (ZIVID_CORE_VERSION_MAJOR == 2 && ZIVID_CORE_VERSION_MINOR >= 13)
  return Zivid::Detail::createApplicationForWrapper(Zivid::Detail::EnvironmentInfo::Wrapper::ros1);
#else
  return Zivid::Application();
#endif
}
}  // namespace

namespace zivid_camera
{
ZividCamera::ZividCamera(ros::NodeHandle& nh, ros::NodeHandle& priv)
  : nh_(nh)
  , priv_(priv)
  , camera_status_(CameraStatus::Idle)
  , use_latched_publisher_for_points_xyz_(false)
  , use_latched_publisher_for_points_xyzrgba_(false)
  , use_latched_publisher_for_color_image_(false)
  , use_latched_publisher_for_depth_image_(false)
  , use_latched_publisher_for_snr_image_(false)
  , use_latched_publisher_for_normals_xyz_(false)
  , image_transport_(nh_)
  , zivid_(makeZividApplication())
  , header_seq_(0)
{
  ROS_INFO("Zivid ROS driver version %s", ZIVID_ROS_DRIVER_VERSION);

  ROS_INFO("Node's namespace is '%s'", nh_.getNamespace().c_str());
  if (nh_.getNamespace() == "")
  {
    // Require the user to specify the namespace that this node will run in.
    // See REP-135 http://www.ros.org/reps/rep-0135.html
    throw std::runtime_error("Zivid ROS driver started in the global namespace ('/')! This is unsupported. "
                             "Please specify namespace, e.g. using the ROS_NAMESPACE environment variable.");
  }

  ROS_INFO_STREAM("Built towards Zivid Core version " << ZIVID_CORE_VERSION);
  ROS_INFO_STREAM("Running with Zivid Core version " << Zivid::Version::coreLibraryVersion());
  if (Zivid::Version::coreLibraryVersion() != ZIVID_CORE_VERSION)
  {
    throw std::runtime_error("Zivid library mismatch! The running Zivid Core version does not match the "
                             "version this ROS driver was built towards. Hint: Try to clean and re-build your project "
                             "from scratch.");
  }

  std::string serial_number;
  priv_.param<decltype(serial_number)>("serial_number", serial_number, "");

  int max_capture_acquisitions;
  priv_.param<decltype(max_capture_acquisitions)>("max_capture_acquisitions", max_capture_acquisitions, 10);

  priv_.param<decltype(frame_id_)>("frame_id", frame_id_, "zivid_optical_frame");

  std::string file_camera_path;
  priv_.param<decltype(file_camera_path)>("file_camera_path", file_camera_path, "");
  const bool file_camera_mode = !file_camera_path.empty();

  priv_.param<bool>("use_latched_publisher_for_points_xyz", use_latched_publisher_for_points_xyz_, false);
  priv_.param<bool>("use_latched_publisher_for_points_xyzrgba", use_latched_publisher_for_points_xyzrgba_, false);
  priv_.param<bool>("use_latched_publisher_for_color_image", use_latched_publisher_for_color_image_, false);
  priv_.param<bool>("use_latched_publisher_for_depth_image", use_latched_publisher_for_depth_image_, false);
  priv_.param<bool>("use_latched_publisher_for_snr_image", use_latched_publisher_for_snr_image_, false);
  priv_.param<bool>("use_latched_publisher_for_normals_xyz", use_latched_publisher_for_normals_xyz_, false);

  bool update_firmware_automatically = true;
  priv_.param<bool>("update_firmware_automatically", update_firmware_automatically, update_firmware_automatically);

  if (file_camera_mode)
  {
    ROS_INFO("Creating file camera from file '%s'", file_camera_path.c_str());
    camera_ = zivid_.createFileCamera(file_camera_path);
  }
  else
  {
    auto cameras = zivid_.cameras();
    ROS_INFO_STREAM(cameras.size() << " cameras found");
    if (cameras.empty())
    {
      throw std::runtime_error("No cameras found. Ensure that the camera is connected to your PC.");
    }
    else if (serial_number.empty())
    {
      camera_ = [&]() {
        ROS_INFO("Selecting first available camera");
        for (auto& c : cameras)
        {
          if (c.state().isAvailable())
            return c;
        }
        throw std::runtime_error("No available cameras found. Is the camera in use by another process?");
      }();
    }
    else
    {
      if (serial_number.find(":") == 0)
      {
        serial_number = serial_number.substr(1);
      }
      camera_ = [&]() {
        ROS_INFO("Searching for camera with serial number '%s' ...", serial_number.c_str());
        for (auto& c : cameras)
        {
          if (c.info().serialNumber() == Zivid::CameraInfo::SerialNumber(serial_number))
            return c;
        }
        throw std::runtime_error("No camera found with serial number '" + serial_number + "'");
      }();
    }

    if (!Zivid::Firmware::isUpToDate(camera_))
    {
      if (update_firmware_automatically)
      {
        ROS_INFO("The camera firmware is not up-to-date, and update_firmware_automatically is true, starting update");
        Zivid::Firmware::update(camera_, [](double progress, const std::string& state) {
          ROS_INFO("  [%.0f%%] %s", progress, state.c_str());
        });
        ROS_INFO("Firmware update completed");
      }
      else
      {
        ROS_ERROR("The camera firmware is not up-to-date, and update_firmware_automatically is false. Throwing error.");
        throw std::runtime_error("The firmware on camera '" + camera_.info().serialNumber().value() +
                                 "' is not up to date. The launch parameter update_firmware_automatically "
                                 "is set to false. Please update the firmware on this camera manually.");
      }
    }
  }

  ROS_INFO_STREAM(camera_);
  if (!file_camera_mode)
  {
    ROS_INFO_STREAM("Connecting to camera '" << camera_.info().serialNumber() << "'");
    camera_.connect();
  }
  ROS_INFO_STREAM("Connected to camera '" << camera_.info().serialNumber() << "'");
  setCameraStatus(CameraStatus::Connected);

  camera_connection_keepalive_timer_ =
      nh_.createTimer(ros::Duration(10), &ZividCamera::onCameraConnectionKeepAliveTimeout, this);

  capture_settings_controller_ =
      std::make_unique<Capture3DSettingsController>(nh_, camera_, "settings", max_capture_acquisitions);

  // HDR is not supported in 2D mode, but for future-proofing the 2D configuration API is analogous
  // to 3D except there is only 1 acquisition.
  capture_2d_settings_controller_ = std::make_unique<Capture2DSettingsController>(nh_, camera_, "settings_2d", 1);

  ROS_INFO("Advertising topics");
  points_xyz_publisher_ =
      nh_.advertise<sensor_msgs::PointCloud2>("points/xyz", 1, use_latched_publisher_for_points_xyz_);
  points_xyzrgba_publisher_ =
      nh_.advertise<sensor_msgs::PointCloud2>("points/xyzrgba", 1, use_latched_publisher_for_points_xyzrgba_);
  color_image_publisher_ =
      image_transport_.advertiseCamera("color/image_color", 1, use_latched_publisher_for_color_image_);
  depth_image_publisher_ = image_transport_.advertiseCamera("depth/image", 1, use_latched_publisher_for_depth_image_);
  snr_image_publisher_ = image_transport_.advertiseCamera("snr/image", 1, use_latched_publisher_for_snr_image_);
  normals_xyz_publisher_ =
      nh_.advertise<sensor_msgs::PointCloud2>("normals/xyz", 1, use_latched_publisher_for_normals_xyz_);

  ROS_INFO("Advertising services");
  camera_info_model_name_service_ =
      nh_.advertiseService("camera_info/model_name", &ZividCamera::cameraInfoModelNameServiceHandler, this);
  camera_info_serial_number_service_ =
      nh_.advertiseService("camera_info/serial_number", &ZividCamera::cameraInfoSerialNumberServiceHandler, this);
  is_connected_service_ = nh_.advertiseService("is_connected", &ZividCamera::isConnectedServiceHandler, this);
  capture_service_ = nh_.advertiseService("capture", &ZividCamera::captureServiceHandler, this);
  capture_and_save_service_ =
      nh_.advertiseService("capture_and_save", &ZividCamera::captureAndSaveServiceHandler, this);
  capture_2d_service_ = nh_.advertiseService("capture_2d", &ZividCamera::capture2DServiceHandler, this);
  capture_assistant_suggest_settings_service_ = nh_.advertiseService(
      "capture_assistant/suggest_settings", &ZividCamera::captureAssistantSuggestSettingsServiceHandler, this);
  load_settings_from_file_service_ =
      nh_.advertiseService("load_settings_from_file", &ZividCamera::loadSettingsFromFileServiceHandler, this);
  load_settings_2d_from_file_service_ =
      nh_.advertiseService("load_settings_2d_from_file", &ZividCamera::loadSettings2DFromFileServiceHandler, this);

  ROS_INFO("Zivid camera driver is now ready!");
}

void ZividCamera::onCameraConnectionKeepAliveTimeout(const ros::TimerEvent&)
{
  ROS_DEBUG_STREAM(__func__);
  try
  {
    reconnectToCameraIfNecessary();
  }
  catch (const std::exception& e)
  {
    ROS_INFO("%s failed with exception '%s'", __func__, e.what());
  }
}

void ZividCamera::reconnectToCameraIfNecessary()
{
  ROS_DEBUG_STREAM(__func__);

  const auto state = camera_.state();
  if (state.isConnected().value())
  {
    setCameraStatus(CameraStatus::Connected);
  }
  else
  {
    setCameraStatus(CameraStatus::Disconnected);

    // The camera handle needs to be refreshed to ensure we get the correct
    // "available" status. This is a bug in the API.
    auto cameras = zivid_.cameras();
    for (auto& c : cameras)
    {
      if (camera_.info().serialNumber() == c.info().serialNumber())
      {
        camera_ = c;
      }
    }

    if (state.isAvailable().value())
    {
      ROS_INFO_STREAM("The camera '" << camera_.info().serialNumber()
                                     << "' is not connected but is available. Re-connecting ...");
      camera_.connect();
      ROS_INFO("Successfully reconnected to camera!");
      setCameraStatus(CameraStatus::Connected);
    }
    else
    {
      ROS_INFO_STREAM("The camera '" << camera_.info().serialNumber() << "' is not connected nor available.");
    }
  }
}

void ZividCamera::setCameraStatus(CameraStatus camera_status)
{
  if (camera_status_ != camera_status)
  {
    std::stringstream ss;
    ss << "Camera status changed to " << toString(camera_status) << " (was " << toString(camera_status_) << ")";
    if (camera_status == CameraStatus::Connected)
    {
      ROS_INFO_STREAM(ss.str());
    }
    else
    {
      ROS_WARN_STREAM(ss.str());
    }
    camera_status_ = camera_status;
  }
}

bool ZividCamera::cameraInfoModelNameServiceHandler(zivid_camera::CameraInfoModelName::Request&,
                                                    zivid_camera::CameraInfoModelName::Response& res)
{
  res.model_name = camera_.info().modelName().toString();
  return true;
}

bool ZividCamera::cameraInfoSerialNumberServiceHandler(zivid_camera::CameraInfoSerialNumber::Request&,
                                                       zivid_camera::CameraInfoSerialNumber::Response& res)
{
  res.serial_number = camera_.info().serialNumber().toString();
  return true;
}

bool ZividCamera::captureServiceHandler(Capture::Request&, Capture::Response&)
{
  ROS_DEBUG_STREAM(__func__);

  invokeCaptureAndPublishFrame();

  return true;
}

bool ZividCamera::captureAndSaveServiceHandler(CaptureAndSave::Request& req, CaptureAndSave::Response&)
{
  ROS_DEBUG_STREAM(__func__);

  const auto frame = invokeCaptureAndPublishFrame();
  ROS_INFO("Saving frame to '%s'", req.file_path.c_str());
  frame.save(req.file_path);

  return true;
}

bool ZividCamera::capture2DServiceHandler(Capture2D::Request&, Capture2D::Response&)
{
  ROS_DEBUG_STREAM(__func__);

  serviceHandlerHandleCameraConnectionLoss();

  const auto settings2D = capture_2d_settings_controller_->zividSettings();

  if (settings2D.acquisitions().isEmpty())
  {
    throw std::runtime_error("capture_2d called with 0 enabled acquisitions!");
  }

  ROS_DEBUG_STREAM(settings2D);
  auto frame2D = camera_.capture(settings2D);
  if (shouldPublishColorImg())
  {
    const auto header = makeHeader();
    auto image = frame2D.imageRGBA();
    const auto intrinsics = Zivid::Experimental::Calibration::intrinsics(camera_);
    const auto camera_info = makeCameraInfo(header, image.width(), image.height(), intrinsics);
    publishColorImage(header, camera_info, image);
  }
  return true;
}

bool ZividCamera::captureAssistantSuggestSettingsServiceHandler(CaptureAssistantSuggestSettings::Request& req,
                                                                CaptureAssistantSuggestSettings::Response&)
{
  ROS_DEBUG_STREAM(__func__ << ": Request: " << req);

  serviceHandlerHandleCameraConnectionLoss();

  if (capture_settings_controller_->numAcquisitionConfigServers() < 10)
  {
    throw std::runtime_error("To use the CaptureAssistant the launch parameter 'max_capture_acquisitions' "
                             "must be at least 10, since the Capture Assistant may suggest up to 10 acquisitions. "
                             "See README.md for more information.");
  }

  using SuggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters;

  const auto max_capture_time =
      std::chrono::round<std::chrono::milliseconds>(std::chrono::duration<double>{ req.max_capture_time.toSec() });
  const auto ambient_light_frequency = [&req]() {
    switch (req.ambient_light_frequency)
    {
      case CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_NONE:
        return SuggestSettingsParameters::AmbientLightFrequency::none;
      case CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_50HZ:
        return SuggestSettingsParameters::AmbientLightFrequency::hz50;
      case CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_60HZ:
        return SuggestSettingsParameters::AmbientLightFrequency::hz60;
    }
    throw std::runtime_error("Unhandled AMBIENT_LIGHT_FREQUENCY value: " + std::to_string(req.ambient_light_frequency));
  }();

  SuggestSettingsParameters suggest_settings_parameters{ SuggestSettingsParameters::MaxCaptureTime{ max_capture_time },
                                                         ambient_light_frequency };
  ROS_INFO_STREAM("Getting suggested settings using parameters: " << suggest_settings_parameters);
  const auto suggested_settings = Zivid::CaptureAssistant::suggestSettings(camera_, suggest_settings_parameters);
  ROS_INFO_STREAM("CaptureAssistant::suggestSettings returned " << suggested_settings.acquisitions().size()
                                                                << " acquisitions");
  capture_settings_controller_->setZividSettings(suggested_settings);

  return true;
}

bool ZividCamera::loadSettingsFromFileServiceHandler(LoadSettingsFromFile::Request& req,
                                                     LoadSettingsFromFile::Response&)
{
  ROS_DEBUG_STREAM(__func__ << ": Request: " << req);
  capture_settings_controller_->setZividSettings(Zivid::Settings{ req.file_path.c_str() });
  return true;
}

bool ZividCamera::loadSettings2DFromFileServiceHandler(LoadSettings2DFromFile::Request& req,
                                                       LoadSettings2DFromFile::Response&)
{
  ROS_DEBUG_STREAM(__func__ << ": Request: " << req);
  capture_2d_settings_controller_->setZividSettings(Zivid::Settings2D{ req.file_path.c_str() });
  return true;
}

void ZividCamera::serviceHandlerHandleCameraConnectionLoss()
{
  reconnectToCameraIfNecessary();
  if (camera_status_ != CameraStatus::Connected)
  {
    throw std::runtime_error("Unable to capture since the camera is not connected. Please re-connect the camera and "
                             "try again.");
  }
}

bool ZividCamera::isConnectedServiceHandler(IsConnected::Request&, IsConnected::Response& res)
{
  res.is_connected = camera_status_ == CameraStatus::Connected;
  return true;
}

void ZividCamera::publishFrame(const Zivid::Frame& frame)
{
  const bool publish_points_xyz = shouldPublishPointsXYZ();
  const bool publish_points_xyzrgba = shouldPublishPointsXYZRGBA();
  const bool publish_color_img = shouldPublishColorImg();
  const bool publish_depth_img = shouldPublishDepthImg();
  const bool publish_snr_img = shouldPublishSnrImg();
  const bool publish_normals_xyz = shouldPublishNormalsXYZ();

  if (publish_points_xyz || publish_points_xyzrgba || publish_color_img || publish_depth_img || publish_snr_img ||
      publish_normals_xyz)
  {
    const auto header = makeHeader();
    auto point_cloud = frame.pointCloud();

    // Transform point cloud from millimeters (Zivid SDK) to meter (ROS).
    const float scale = 0.001f;
    const auto transformationMatrix = Zivid::Matrix4x4{ scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, 1 };
    point_cloud.transform(transformationMatrix);

    if (publish_points_xyz)
    {
      publishPointCloudXYZ(header, point_cloud);
    }
    if (publish_points_xyzrgba)
    {
      publishPointCloudXYZRGBA(header, point_cloud);
    }
    if (publish_color_img || publish_depth_img || publish_snr_img)
    {
      const auto intrinsics = Zivid::Experimental::Calibration::intrinsics(camera_);
      const auto camera_info = makeCameraInfo(header, point_cloud.width(), point_cloud.height(), intrinsics);

      if (publish_color_img)
      {
        publishColorImage(header, camera_info, point_cloud);
      }
      if (publish_depth_img)
      {
        publishDepthImage(header, camera_info, point_cloud);
      }
      if (publish_snr_img)
      {
        publishSnrImage(header, camera_info, point_cloud);
      }
    }
    if (publish_normals_xyz)
    {
      publishNormalsXYZ(header, point_cloud);
    }
  }
}

bool ZividCamera::shouldPublishPointsXYZRGBA() const
{
  return points_xyzrgba_publisher_.getNumSubscribers() > 0 || use_latched_publisher_for_points_xyzrgba_;
}

bool ZividCamera::shouldPublishPointsXYZ() const
{
  return points_xyz_publisher_.getNumSubscribers() > 0 || use_latched_publisher_for_points_xyz_;
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
  return normals_xyz_publisher_.getNumSubscribers() > 0 || use_latched_publisher_for_normals_xyz_;
}

std_msgs::Header ZividCamera::makeHeader()
{
  std_msgs::Header header;
  header.seq = header_seq_++;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id_;
  return header;
}

void ZividCamera::publishPointCloudXYZ(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud)
{
  ROS_DEBUG_STREAM("Publishing " << points_xyz_publisher_.getTopic());

  // We are using the Zivid::XYZW type here for compatibility with the pcl::PointXYZ type, which contains an
  // padding float for performance reasons. We could use the "pcl_conversion" utility functions to construct
  // the PointCloud2 message. However, those are observed to add significant overhead due to extra unnecssary
  // copies of the data.
  using ZividDataType = Zivid::PointXYZW;
  auto msg = boost::make_shared<sensor_msgs::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(3);
  msg->fields.push_back(createPointField("x", 0, sensor_msgs::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("y", 4, sensor_msgs::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("z", 8, sensor_msgs::PointField::FLOAT32, 1));
  msg->is_dense = false;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType*>(msg->data.data()));
  points_xyz_publisher_.publish(msg);
}

void ZividCamera::publishPointCloudXYZRGBA(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud)
{
  ROS_DEBUG_STREAM("Publishing " << points_xyzrgba_publisher_.getTopic());

  auto msg = boost::make_shared<sensor_msgs::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(4);
  msg->fields.push_back(createPointField("x", 0, sensor_msgs::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("y", 4, sensor_msgs::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("z", 8, sensor_msgs::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("rgba", 12, sensor_msgs::PointField::FLOAT32, 1));
  msg->is_dense = false;

  // Note that the "rgba" field is actually byte order "bgra" on little-endian systems. For this
  // reason we use the Zivid BGRA type.
  using ZividDataType = Zivid::PointXYZColorBGRA;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType*>(msg->data.data()));
  points_xyzrgba_publisher_.publish(msg);
}

void ZividCamera::publishColorImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                                    const Zivid::PointCloud& point_cloud)
{
  ROS_DEBUG_STREAM("Publishing " << color_image_publisher_.getTopic() << " from point cloud");
  auto image = makePointCloudImage<Zivid::ColorRGBA>(point_cloud, header, sensor_msgs::image_encodings::RGBA8);
  color_image_publisher_.publish(image, camera_info);
}

void ZividCamera::publishColorImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                                    const Zivid::Image<Zivid::ColorRGBA>& image)
{
  ROS_DEBUG_STREAM("Publishing " << color_image_publisher_.getTopic() << " from Image");
  auto msg = makeImage(header, sensor_msgs::image_encodings::RGBA8, image.width(), image.height());
  const auto uint8_ptr_begin = reinterpret_cast<const uint8_t*>(image.data());
  const auto uint8_ptr_end = reinterpret_cast<const uint8_t*>(image.data() + image.size());
  msg->data = std::vector<uint8_t>(uint8_ptr_begin, uint8_ptr_end);
  color_image_publisher_.publish(msg, camera_info);
}

void ZividCamera::publishDepthImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                                    const Zivid::PointCloud& point_cloud)
{
  ROS_DEBUG_STREAM("Publishing " << depth_image_publisher_.getTopic());
  auto image = makePointCloudImage<Zivid::PointZ>(point_cloud, header, sensor_msgs::image_encodings::TYPE_32FC1);
  depth_image_publisher_.publish(image, camera_info);
}

void ZividCamera::publishSnrImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                                  const Zivid::PointCloud& point_cloud)
{
  ROS_DEBUG_STREAM("Publishing " << snr_image_publisher_.getTopic());
  auto image = makePointCloudImage<Zivid::SNR>(point_cloud, header, sensor_msgs::image_encodings::TYPE_32FC1);
  snr_image_publisher_.publish(image, camera_info);
}

void ZividCamera::publishNormalsXYZ(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud)
{
  ROS_DEBUG_STREAM("Publishing " << normals_xyz_publisher_.getTopic());

  using ZividDataType = Zivid::NormalXYZ;
  auto msg = boost::make_shared<sensor_msgs::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(3);
  msg->fields.push_back(createPointField("normal_x", 0, sensor_msgs::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("normal_y", 4, sensor_msgs::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("normal_z", 8, sensor_msgs::PointField::FLOAT32, 1));
  msg->is_dense = false;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType*>(msg->data.data()));
  normals_xyz_publisher_.publish(msg);
}

sensor_msgs::CameraInfoConstPtr ZividCamera::makeCameraInfo(const std_msgs::Header& header, std::size_t width,
                                                            std::size_t height,
                                                            const Zivid::CameraIntrinsics& intrinsics)
{
  auto msg = boost::make_shared<sensor_msgs::CameraInfo>();
  msg->header = header;
  msg->width = static_cast<uint32_t>(width);
  msg->height = static_cast<uint32_t>(height);
  msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // k1, k2, t1, t2, k3
  const auto distortion = intrinsics.distortion();
  msg->D.resize(5);
  msg->D[0] = distortion.k1().value();
  msg->D[1] = distortion.k2().value();
  msg->D[2] = distortion.p1().value();
  msg->D[3] = distortion.p2().value();
  msg->D[4] = distortion.k3().value();

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  const auto camera_matrix = intrinsics.cameraMatrix();
  msg->K[0] = camera_matrix.fx().value();
  msg->K[2] = camera_matrix.cx().value();
  msg->K[4] = camera_matrix.fy().value();
  msg->K[5] = camera_matrix.cy().value();
  msg->K[8] = 1;

  // R (identity)
  msg->R[0] = 1;
  msg->R[4] = 1;
  msg->R[8] = 1;

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  msg->P[0] = camera_matrix.fx().value();
  msg->P[2] = camera_matrix.cx().value();
  msg->P[5] = camera_matrix.fy().value();
  msg->P[6] = camera_matrix.cy().value();
  msg->P[10] = 1;

  return msg;
}

Zivid::Frame ZividCamera::invokeCaptureAndPublishFrame()
{
  ROS_DEBUG_STREAM(__func__);

  serviceHandlerHandleCameraConnectionLoss();

  const auto settings = capture_settings_controller_->zividSettings();

  if (settings.acquisitions().isEmpty())
  {
    throw std::runtime_error("capture called with 0 enabled acquisitions!");
  }

  ROS_INFO("Capturing with %zd acquisition(s)", settings.acquisitions().size());
  ROS_DEBUG_STREAM(settings);
  const auto frame = camera_.capture(settings);
  publishFrame(frame);
  return frame;
}

}  // namespace zivid_camera
