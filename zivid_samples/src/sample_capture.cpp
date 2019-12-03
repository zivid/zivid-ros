#include <zivid_camera/CaptureFrameConfig.h>
#include <zivid_camera/CaptureGeneralConfig.h>
#include <zivid_camera/Capture.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

namespace
{
const ros::Duration default_wait_duration{ 30 };

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

void on_points(const sensor_msgs::PointCloud2ConstPtr&)
{
  ROS_INFO("PointCloud received");
  capture();
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture_cpp");
  ros::NodeHandle n;

  ROS_INFO("Starting sample_capture.cpp");

  CHECK(ros::service::waitForService("/zivid_camera/capture", default_wait_duration));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto points_sub = n.subscribe("/zivid_camera/points", 1, on_points);

  ROS_INFO("Enabling the reflection filter");
  dynamic_reconfigure::Client<zivid_camera::CaptureGeneralConfig> capture_general_client("/zivid_camera/"
                                                                                         "capture/general/");
  zivid_camera::CaptureGeneralConfig config;
  CHECK(capture_general_client.getDefaultConfiguration(config, default_wait_duration));
  config.filters_reflection_enabled = true;
  CHECK(capture_general_client.setConfiguration(config));

  ROS_INFO("Enabling and configuring the first frame");
  dynamic_reconfigure::Client<zivid_camera::CaptureFrameConfig> frame_0_client("/zivid_camera/capture/frame_0/");

  zivid_camera::CaptureFrameConfig frame_0_cfg;
  CHECK(frame_0_client.getDefaultConfiguration(frame_0_cfg, default_wait_duration));

  frame_0_cfg.enabled = true;
  frame_0_cfg.iris = 21;
  frame_0_cfg.exposure_time = 20000;
  CHECK(frame_0_client.setConfiguration(frame_0_cfg));

  capture();

  ros::waitForShutdown();

  return 0;
}