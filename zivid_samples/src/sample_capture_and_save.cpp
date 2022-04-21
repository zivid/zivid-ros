#include <zivid_camera/SettingsAcquisitionConfig.h>
#include <zivid_camera/SettingsConfig.h>
#include <zivid_camera/CaptureAndSave.h>
#include <Zivid/Version.h>
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

void capture_and_save()
{
  ROS_INFO("Calling capture_and_save service");
  zivid_camera::CaptureAndSave capture_and_save;
  std::string file_path = "/tmp/capture_cpp.zdf";
  capture_and_save.request.file_path = file_path;
  CHECK(ros::service::call("/zivid_camera/capture_and_save", capture_and_save));
  ROS_INFO("Your .zdf file is now available here: %s", file_path.c_str());
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

void enable_diagnostics()
{
  dynamic_reconfigure::Client<zivid_camera::SettingsConfig> settings_client("/zivid_camera/"
                                                                            "settings/");
  zivid_camera::SettingsConfig settings_config;
  CHECK(settings_client.getDefaultConfiguration(settings_config, default_wait_duration));

  ROS_INFO("Enabling diagnostics mode");
  settings_config.diagnostics_enabled = true;
  CHECK(settings_client.setConfiguration(settings_config));
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture_and_save_cpp");
  ros::NodeHandle n;

  ROS_INFO("Starting sample_capture_and_save.cpp");

  CHECK(ros::service::waitForService("/zivid_camera/capture_and_save", default_wait_duration));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  enable_first_acquisition();

  enable_diagnostics();

  capture_and_save();

  ros::waitForShutdown();

  return 0;
}