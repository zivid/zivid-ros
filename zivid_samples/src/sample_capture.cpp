#include <zivid_camera/SettingsAcquisitionConfig.h>
#include <zivid_camera/SettingsConfig.h>
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

  auto points_sub = n.subscribe("/zivid_camera/points/xyzrgba", 1, on_points);

  dynamic_reconfigure::Client<zivid_camera::SettingsConfig> settings_client("/zivid_camera/"
                                                                            "settings/");

  // To initialize the settings_config object we need to load the default configuration from the server.
  // The default values of settings depends on which Zivid camera model is connected.
  zivid_camera::SettingsConfig settings_config;
  CHECK(settings_client.getDefaultConfiguration(settings_config, default_wait_duration));

  ROS_INFO("Enabling the reflection removal filter");
  settings_config.processing_filters_reflection_removal_enabled = true;
  CHECK(settings_client.setConfiguration(settings_config));

  dynamic_reconfigure::Client<zivid_camera::SettingsAcquisitionConfig> acquisition_0_client("/zivid_camera/settings/"
                                                                                            "acquisition_0/");

  // To initialize the acquisition_0_config object we need to load the default configuration from the server.
  // The default values of settings depends on which Zivid camera model is connected.
  zivid_camera::SettingsAcquisitionConfig acquisition_0_config;
  CHECK(acquisition_0_client.getDefaultConfiguration(acquisition_0_config, default_wait_duration));

  ROS_INFO("Enabling and configuring the first acquisition");
  acquisition_0_config.enabled = true;
  acquisition_0_config.aperture = 5.66;
  acquisition_0_config.exposure_time = 20000;
  CHECK(acquisition_0_client.setConfiguration(acquisition_0_config));

  ROS_INFO("Calling capture");

  capture();

  ros::waitForShutdown();

  return 0;
}