#include <zivid_camera/Capture.h>
#include <zivid_camera/LoadSettingsFromFile.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
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

bool waitForPublisher(const ros::Subscriber& sub, const ros::Duration& timeout)
{
  const auto start = ros::Time::now();
  while ((ros::Time::now() - start) < timeout)
  {
    if (sub.getNumPublishers() > 0)
    {
      return true;
    }
    ros::Duration(0.1).sleep();
  }
  return false;
}

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

void onPoints(const sensor_msgs::PointCloud2ConstPtr&)
{
  ROS_INFO("Point cloud received");
}

void onImageColor(const sensor_msgs::ImageConstPtr&)
{
  ROS_INFO("Color image received");
}

void onAcquisitionDone(const std_msgs::HeaderConstPtr&)
{
  ROS_INFO("Acquisition done");
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture_with_settings_from_yml_cpp");
  ros::NodeHandle n;

  ROS_INFO("Starting sample_capture_with_settings_from_yml.cpp");

  CHECK(ros::service::waitForService("/zivid_camera/capture", default_wait_duration));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto acquisition_done_sub = n.subscribe("/zivid_camera/acquisition_done", 1, onAcquisitionDone);
  CHECK(waitForPublisher(acquisition_done_sub, default_wait_duration));
  auto image_color_sub = n.subscribe("/zivid_camera/color/image_color", 1, onImageColor);
  CHECK(waitForPublisher(image_color_sub, default_wait_duration));
  auto points_sub = n.subscribe("/zivid_camera/points/xyz", 1, onPoints);
  CHECK(waitForPublisher(points_sub, default_wait_duration));

  std::string samples_path = ros::package::getPath("zivid_samples");
  std::string settings_path = samples_path + "/settings/camera_settings.yml";
  ROS_INFO("Loading settings from: %s", settings_path.c_str());
  zivid_camera::LoadSettingsFromFile load_settings_from_file;
  load_settings_from_file.request.file_path = settings_path;
  CHECK(ros::service::call("/zivid_camera/load_settings_from_file", load_settings_from_file));

  ROS_INFO("Calling capture");

  capture();

  ros::waitForShutdown();

  return 0;
}