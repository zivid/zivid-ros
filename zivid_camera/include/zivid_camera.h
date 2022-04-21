#pragma once

#include "auto_generated_include_wrapper.h"
#include "capture_settings_controller.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/Image.h>

namespace Zivid
{
class Settings;
}

namespace zivid_camera
{
enum class CameraStatus
{
  Idle,
  Connected,
  Disconnected
};

class ZividCamera
{
public:
  ZividCamera(ros::NodeHandle& nh, ros::NodeHandle& priv);

private:
  void onCameraConnectionKeepAliveTimeout(const ros::TimerEvent& event);
  void reconnectToCameraIfNecessary();
  void setCameraStatus(CameraStatus camera_status);
  bool cameraInfoModelNameServiceHandler(CameraInfoModelName::Request& req, CameraInfoModelName::Response& res);
  bool cameraInfoSerialNumberServiceHandler(CameraInfoSerialNumber::Request& req,
                                            CameraInfoSerialNumber::Response& res);
  bool captureServiceHandler(Capture::Request& req, Capture::Response& res);
  bool captureAndSaveServiceHandler(CaptureAndSave::Request& req, CaptureAndSave::Response& res);
  bool capture2DServiceHandler(Capture2D::Request& req, Capture2D::Response& res);
  bool captureAssistantSuggestSettingsServiceHandler(CaptureAssistantSuggestSettings::Request& req,
                                                     CaptureAssistantSuggestSettings::Response& res);
  bool loadSettingsFromFileServiceHandler(LoadSettingsFromFile::Request& req, LoadSettingsFromFile::Response&);
  bool loadSettings2DFromFileServiceHandler(LoadSettings2DFromFile::Request& req, LoadSettings2DFromFile::Response&);
  void serviceHandlerHandleCameraConnectionLoss();
  bool isConnectedServiceHandler(IsConnected::Request& req, IsConnected::Response& res);
  void publishFrame(const Zivid::Frame& frame);
  Zivid::Frame invokeCaptureAndPublishFrame();
  bool shouldPublishPointsXYZ() const;
  bool shouldPublishPointsXYZRGBA() const;
  bool shouldPublishColorImg() const;
  bool shouldPublishDepthImg() const;
  bool shouldPublishSnrImg() const;
  bool shouldPublishNormalsXYZ() const;
  std_msgs::Header makeHeader();
  void publishPointCloudXYZ(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud);
  void publishPointCloudXYZRGBA(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud);
  void publishColorImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                         const Zivid::PointCloud& point_cloud);
  void publishColorImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                         const Zivid::Image<Zivid::ColorRGBA>& image);
  void publishDepthImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                         const Zivid::PointCloud& point_cloud);
  void publishSnrImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                       const Zivid::PointCloud& point_cloud);
  void publishNormalsXYZ(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud);
  sensor_msgs::CameraInfoConstPtr makeCameraInfo(const std_msgs::Header& header, std::size_t width, std::size_t height,
                                                 const Zivid::CameraIntrinsics& intrinsics);

  using Capture3DSettingsController =
      CaptureSettingsController<Zivid::Settings, SettingsConfig, SettingsAcquisitionConfig>;
  using Capture2DSettingsController =
      CaptureSettingsController<Zivid::Settings2D, Settings2DConfig, Settings2DAcquisitionConfig>;

  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Timer camera_connection_keepalive_timer_;
  CameraStatus camera_status_;
  bool use_latched_publisher_for_points_xyz_;
  bool use_latched_publisher_for_points_xyzrgba_;
  bool use_latched_publisher_for_color_image_;
  bool use_latched_publisher_for_depth_image_;
  bool use_latched_publisher_for_snr_image_;
  bool use_latched_publisher_for_normals_xyz_;
  ros::Publisher points_xyz_publisher_;
  ros::Publisher points_xyzrgba_publisher_;
  image_transport::ImageTransport image_transport_;
  image_transport::CameraPublisher color_image_publisher_;
  image_transport::CameraPublisher depth_image_publisher_;
  image_transport::CameraPublisher snr_image_publisher_;
  ros::Publisher normals_xyz_publisher_;
  ros::ServiceServer camera_info_serial_number_service_;
  ros::ServiceServer camera_info_model_name_service_;
  ros::ServiceServer capture_service_;
  ros::ServiceServer capture_and_save_service_;
  ros::ServiceServer capture_2d_service_;
  ros::ServiceServer capture_assistant_suggest_settings_service_;
  ros::ServiceServer load_settings_from_file_service_;
  ros::ServiceServer load_settings_2d_from_file_service_;
  ros::ServiceServer is_connected_service_;
  std::unique_ptr<Capture3DSettingsController> capture_settings_controller_;
  std::unique_ptr<Capture2DSettingsController> capture_2d_settings_controller_;
  Zivid::Application zivid_;
  Zivid::Camera camera_;
  std::string frame_id_;
  unsigned int header_seq_;
};
}  // namespace zivid_camera
