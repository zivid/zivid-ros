#include <zivid_camera/Settings2DAcquisitionConfig.h>
#include <zivid_camera/Capture2D.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/LoadSettingsFromFile.h>
#include <zivid_camera/LoadSettings2DFromFile.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <ros/package.h>


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

void loadSettings()
{
  std::string samples_path = ros::package::getPath("zivid_samples");
  std::string settings_path = samples_path + "/settings/m130_settings.yml";
  zivid_camera::LoadSettingsFromFile load_settings_from_file;
  load_settings_from_file.request.file_path = settings_path;
  CHECK(ros::service::call("/zivid_camera/load_settings_from_file", load_settings_from_file));
}

void capture3d(const ros::TimerEvent& event)
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

void load2dSettings()
{
  std::string samples_path = ros::package::getPath("zivid_samples");
  std::string settings_path = samples_path + "/settings/m130_2d_settings.yml";
  zivid_camera::LoadSettings2DFromFile load_settings_from_file;
  load_settings_from_file.request.file_path = settings_path;
  CHECK(ros::service::call("/zivid_camera/load_settings_2d_from_file", load_settings_from_file));
}

void capture()
{
  ROS_INFO("Calling capture_2d service");
  zivid_camera::Capture2D capture_2d;
  CHECK(ros::service::call("/zivid_camera/capture_2d", capture_2d));
}

void on_image_color(const sensor_msgs::ImageConstPtr& image)
{
  ROS_INFO("2D color image received");
  // Trigger 2d capture when a color image is received
  capture();
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_2d_capture_with_periodic_3d_capture");
  ros::NodeHandle n;

  ros::Timer timer = n.createTimer(ros::Duration(3.0), capture3d);
  CHECK(ros::service::waitForService("/zivid_camera/capture_2d", default_wait_duration));
  CHECK(ros::service::waitForService("/zivid_camera/capture", default_wait_duration));

  // Note that color both from 2d capture and 3d capture comes on this topic
  // We should change that
  auto image_color_sub = n.subscribe("/zivid_camera/color/image_color", 1, on_image_color);

  loadSettings();
  load2dSettings();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  capture();

  ros::waitForShutdown();

  return 0;
}