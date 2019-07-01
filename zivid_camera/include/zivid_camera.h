#pragma once

#include "auto_generated_include_wrapper.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>

#include <Zivid/Application.h>
#include <Zivid/Camera.h>

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
  struct DRFrameConfig
  {
    DRFrameConfig(const std::string& _name, ros::NodeHandle& nh)
      : name(_name), dr_server(dr_server_mutex, ros::NodeHandle(nh, name)), config(CaptureFrameConfig::__getDefault__())
    {
    }
    std::string name;
    boost::recursive_mutex dr_server_mutex;
    dynamic_reconfigure::Server<CaptureFrameConfig> dr_server;
    CaptureFrameConfig config;
  };

  void onCameraConnectionKeepAliveTimeout(const ros::TimerEvent& event);
  void reconnectToCameraIfNecessary();
  void setCameraStatus(CameraStatus camera_status);
  void setupCaptureGeneralConfigNode(const Zivid::Settings& camera_settings);
  void setupCaptureFrameConfigNode(int nodeIdx, const Zivid::Settings& camera_settings);
  void onCaptureGeneralConfigChanged(CaptureGeneralConfig& config);
  void onCaptureFrameConfigChanged(CaptureFrameConfig& config, DRFrameConfig& frame_config);
  bool cameraInfoModelNameServiceHandler(CameraInfoModelName::Request& req, CameraInfoModelName::Response& res);
  bool cameraInfoSerialNumberServiceHandler(CameraInfoSerialNumber::Request& req,
                                            CameraInfoSerialNumber::Response& res);
  bool captureServiceHandler(Capture::Request& req, Capture::Response& res);
  bool isConnectedServiceHandler(IsConnected::Request& req, IsConnected::Response& res);

  void publishFrame(Zivid::Frame&& frame);
  sensor_msgs::PointCloud2ConstPtr makePointCloud2(const std_msgs::Header& header,
                                                   const Zivid::PointCloud& point_cloud);
  sensor_msgs::ImageConstPtr makeColorImage(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud);
  sensor_msgs::ImageConstPtr makeDepthImage(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud);
  sensor_msgs::CameraInfoConstPtr makeCameraInfo(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud,
                                                 const Zivid::CameraIntrinsics& intrinsics);

  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Timer camera_connection_keepalive_timer_;
  CameraStatus camera_status_;
  boost::recursive_mutex capture_general_dr_server_mutex_;
  std::unique_ptr<dynamic_reconfigure::Server<CaptureGeneralConfig>> capture_general_dr_server_;
  CaptureGeneralConfig current_capture_general_config_;
  bool use_latched_publisher_for_points_;
  bool use_latched_publisher_for_color_image_;
  bool use_latched_publisher_for_depth_image_;
  ros::Publisher points_publisher_;
  image_transport::ImageTransport image_transport_;
  image_transport::CameraPublisher color_image_publisher_;
  image_transport::CameraPublisher depth_image_publisher_;
  ros::ServiceServer camera_info_serial_number_service_;
  ros::ServiceServer camera_info_model_name_service_;
  ros::ServiceServer capture_service_;
  ros::ServiceServer is_connected_service_;
  std::vector<std::unique_ptr<DRFrameConfig>> frame_configs_;
  Zivid::Application zivid_;
  Zivid::Camera camera_;
  std::string frame_id_;
  unsigned int header_seq_;
};
}  // namespace zivid_camera
