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

#pragma once

#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <zivid_camera/visibility.hpp>
#include <zivid_interfaces/srv/camera_info_model_name.hpp>
#include <zivid_interfaces/srv/camera_info_serial_number.hpp>
#include <zivid_interfaces/srv/capture_and_save.hpp>
#include <zivid_interfaces/srv/capture_assistant_suggest_settings.hpp>
#include <zivid_interfaces/srv/is_connected.hpp>

namespace Zivid
{
class Application;
class Camera;
class CameraIntrinsics;
struct ColorRGBA;
struct ColorSRGB;
using ColorRGBA_SRGB = ColorSRGB;
class Frame;
template <typename T>
class Image;
class PointCloud;
class Settings2D;
class Settings;
}  // namespace Zivid

namespace zivid_camera
{
enum class CameraStatus
{
  Idle,
  Connected,
  Disconnected
};
enum class ColorSpace
{
  sRGB,
  LinearRGB,
};
template <typename SettingsType>
class CaptureSettingsController;
class DetectorController;
class HandEyeCalibrationController;
class InfieldCorrectionController;
class ControllerInterface;

class ZividCamera : public rclcpp::Node
{
public:
  ZIVID_CAMERA_ROS_PUBLIC ZividCamera(const rclcpp::NodeOptions & options);
  ~ZividCamera() override;
  ZIVID_CAMERA_ROS_PUBLIC Zivid::Application & zividApplication();

private:
  void onCameraConnectionKeepAliveTimeout();
  void reconnectToCameraIfNecessary();
  void setCameraStatus(CameraStatus camera_status);
  rcl_interfaces::msg::SetParametersResult setParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  void cameraInfoModelNameServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Response> response);
  void cameraInfoSerialNumberServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Response> response);
  void captureServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void captureAndSaveServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CaptureAndSave::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CaptureAndSave::Response> response);
  void capture2DServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void captureAssistantSuggestSettingsServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Response> response);
  void serviceHandlerHandleCameraConnectionLoss();
  void isConnectedServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::IsConnected::Request> request,
    std::shared_ptr<zivid_interfaces::srv::IsConnected::Response> response);
  void publishFrame(const Zivid::Frame & frame);
  Zivid::Frame invokeCaptureAndPublishFrame(const Zivid::Settings & settings);
  bool shouldPublishPointsXYZ() const;
  bool shouldPublishPointsXYZRGBA() const;
  bool shouldPublishColorImg() const;
  bool shouldPublishDepthImg() const;
  bool shouldPublishSnrImg() const;
  bool shouldPublishNormalsXYZ() const;
  std_msgs::msg::Header makeHeader();
  void publishPointCloudXYZ(
    const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud);
  void publishPointCloudXYZRGBA(
    const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud,
    ColorSpace color_space);
  void publishColorImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::PointCloud & point_cloud, ColorSpace color_space);
  void publishColorImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::Image<Zivid::ColorRGBA> & image);
  void publishColorImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::Image<Zivid::ColorRGBA_SRGB> & image);
  void publishDepthImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::PointCloud & point_cloud);
  void publishSnrImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::PointCloud & point_cloud);
  void publishNormalsXYZ(
    const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud);
  sensor_msgs::msg::CameraInfo::ConstSharedPtr makeCameraInfo(
    const std_msgs::msg::Header & header, std::size_t width, std::size_t height,
    const Zivid::CameraIntrinsics & intrinsics);
  [[noreturn]] void logErrorAndThrowRuntimeException(const std::string & message);
  ColorSpace colorSpace() const;

  friend class ControllerInterface;

  std::map<std::string, ColorSpace> color_space_name_value_map_;
  rclcpp::TimerBase::SharedPtr camera_connection_keepalive_timer_;
  bool use_latched_publisher_for_points_xyz_{false};
  bool use_latched_publisher_for_points_xyzrgba_{false};
  bool use_latched_publisher_for_color_image_{false};
  bool use_latched_publisher_for_depth_image_{false};
  bool use_latched_publisher_for_snr_image_{false};
  bool use_latched_publisher_for_normals_xyz_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_xyz_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_xyzrgba_publisher_;
  image_transport::CameraPublisher color_image_publisher_;
  image_transport::CameraPublisher depth_image_publisher_;
  image_transport::CameraPublisher snr_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr normals_xyz_publisher_;
  rclcpp::Service<zivid_interfaces::srv::CameraInfoSerialNumber>::SharedPtr
    camera_info_serial_number_service_;
  rclcpp::Service<zivid_interfaces::srv::CameraInfoModelName>::SharedPtr
    camera_info_model_name_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::Service<zivid_interfaces::srv::CaptureAndSave>::SharedPtr capture_and_save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_2d_service_;
  rclcpp::Service<zivid_interfaces::srv::CaptureAssistantSuggestSettings>::SharedPtr
    capture_assistant_suggest_settings_service_;
  rclcpp::Service<zivid_interfaces::srv::IsConnected>::SharedPtr is_connected_service_;

  std::unique_ptr<DetectorController> detector_controller_;
  std::unique_ptr<InfieldCorrectionController> infield_correction_controller_;
  std::unique_ptr<HandEyeCalibrationController> hand_eye_calibration_controller_;

  std::unique_ptr<Zivid::Application> zivid_;
  CameraStatus camera_status_{CameraStatus::Idle};
  std::unique_ptr<CaptureSettingsController<Zivid::Settings>> settings_controller_;
  std::unique_ptr<CaptureSettingsController<Zivid::Settings2D>> settings_2d_controller_;
  // The callback must be declared after the settings controllers since the callback references
  // both controllers. Otherwise, the callback could run before the controllers are initialized,
  // which is undefined behavior.
  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;
  std::unique_ptr<Zivid::Camera> camera_;
  std::string frame_id_;
};
}  // namespace zivid_camera
