#pragma once

#include "auto_generated_include_wrapper.h"

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
  bool capture2DServiceHandler(Capture::Request& req, Capture::Response& res);
  bool captureAssistantSuggestSettingsServiceHandler(CaptureAssistantSuggestSettings::Request& req,
                                                     CaptureAssistantSuggestSettings::Response& res);
  void serviceHandlerHandleCameraConnectionLoss();
  bool isConnectedServiceHandler(IsConnected::Request& req, IsConnected::Response& res);
  void publishFrame(Zivid::Frame&& frame);
  bool shouldPublishPoints() const;
  bool shouldPublishColorImg() const;
  bool shouldPublishDepthImg() const;
  std_msgs::Header makeHeader();
  void publishPoints(const std_msgs::Header& header, const Zivid::PointCloud& point_cloud);
  void publishColorImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                         const Zivid::PointCloud& point_cloud);
  void publishColorImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                         const Zivid::Image<Zivid::RGBA8>& image);
  void publishDepthImage(const std_msgs::Header& header, const sensor_msgs::CameraInfoConstPtr& camera_info,
                         const Zivid::PointCloud& point_cloud);
  sensor_msgs::CameraInfoConstPtr makeCameraInfo(const std_msgs::Header& header, std::size_t width, std::size_t height,
                                                 const Zivid::CameraIntrinsics& intrinsics);

  template <typename ConfigType_>
  class ConfigDRServer
  {
  public:
    using ConfigType = ConfigType_;
    template <typename ZividSettings>
    ConfigDRServer(const std::string& name, ros::NodeHandle& nh, const ZividSettings& defaultSettings);
    void setConfig(const ConfigType& cfg);
    const ConfigType& config() const
    {
      return config_;
    }
    const std::string& name() const
    {
      return name_;
    }

  private:
    std::string name_;
    boost::recursive_mutex dr_server_mutex_;
    dynamic_reconfigure::Server<ConfigType> dr_server_;
    ConfigType config_;
  };

  using CaptureGeneralConfigDRServer = ConfigDRServer<CaptureGeneralConfig>;
  using CaptureFrameConfigDRServer = ConfigDRServer<CaptureFrameConfig>;
  using Capture2DFrameConfigDRServer = ConfigDRServer<Capture2DFrameConfig>;

  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Timer camera_connection_keepalive_timer_;
  CameraStatus camera_status_;
  std::unique_ptr<CaptureGeneralConfigDRServer> capture_general_config_dr_server_;
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
  ros::ServiceServer capture_2d_service_;
  ros::ServiceServer capture_assistant_suggest_settings_service_;
  ros::ServiceServer is_connected_service_;
  std::vector<std::unique_ptr<CaptureFrameConfigDRServer>> capture_frame_config_dr_servers_;
  std::vector<std::unique_ptr<Capture2DFrameConfigDRServer>> capture_2d_frame_config_dr_servers_;
  Zivid::Application zivid_;
  Zivid::Camera camera_;
  std::string frame_id_;
  unsigned int header_seq_;
};
}  // namespace zivid_camera
