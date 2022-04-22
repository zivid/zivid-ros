#include <zivid_camera/SettingsAcquisitionConfig.h>
#include <zivid_camera/SettingsConfig.h>
#include <zivid_camera/CaptureAndSaveFrame.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
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

void capture_and_save_frame()
{
  ROS_INFO("Calling capture_and_save_frame service");
  zivid_camera::CaptureAndSaveFrame capture_and_save_frame;
  std::string file_path = "/tmp/capture_cpp.zdf";
  capture_and_save_frame.request.file_path = file_path;
  CHECK(ros::service::call("/zivid_camera/capture_and_save_frame", capture_and_save_frame));
}

void enable_diagnostics()
{
  dynamic_reconfigure::Client<zivid_camera::SettingsConfig> settings_client("/zivid_camera/"
                                                                            "settings/");

  // To initialize the settings_config object we need to load the default configuration from the server.
  // The default values of settings depends on which Zivid camera model is connected.
  zivid_camera::SettingsConfig settings_config;
  CHECK(settings_client.getDefaultConfiguration(settings_config, default_wait_duration));

  ROS_INFO("Enabling the diagnostics mode");
  settings_config.diagnostics_enabled = true;
  CHECK(settings_client.setConfiguration(settings_config));
}

void enable_first_acquisition()
{
  dynamic_reconfigure::Client<zivid_camera::SettingsAcquisitionConfig> acquisition_0_client("/zivid_camera/settings/"
                                                                                            "acquisition_0/");

  // To initialize the acquisition_0_config object we need to load the default configuration from the server.
  // The default values of settings depends on which Zivid camera model is connected.
  zivid_camera::SettingsAcquisitionConfig acquisition_0_config;
  CHECK(acquisition_0_client.getDefaultConfiguration(acquisition_0_config, default_wait_duration));

  ROS_INFO("Enabling the first acquisition");
  acquisition_0_config.enabled = true;
  CHECK(acquisition_0_client.setConfiguration(acquisition_0_config));
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture_and_save_frame_cpp");
  ros::NodeHandle n;

  ROS_INFO("Starting sample_capture_and_save_frame.cpp");

  CHECK(ros::service::waitForService("/zivid_camera/capture_and_save_frame", default_wait_duration));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  enable_diagnostics();

  enable_first_acquisition();

  ROS_INFO("Calling capture_and_save_frame");

  capture_and_save_frame();

  ros::waitForShutdown();

  return 0;
}