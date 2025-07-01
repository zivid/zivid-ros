#include <zivid_camera/Capture.h>
#include <zivid_camera/LoadSettingsFromFile.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
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

auto waitForPublisher = [](const ros::Subscriber& sub, ros::Duration timeout) {
  ros::Time start = ros::Time::now();
  ros::Rate rate(10);
  while ((ros::Time::now() - start) < timeout)
  {
    if (sub.getNumPublishers() > 0)
    {
      return true;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return false;
};

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

void on_points(const sensor_msgs::PointCloud2ConstPtr&)
{
  ROS_INFO("PointCloud received");
}

void on_image_color(const sensor_msgs::ImageConstPtr&)
{
  ROS_INFO("2D color image received");
}

void on_acquisition_done(const std_msgs::HeaderConstPtr& header)
{
  ROS_INFO("Acquisition done: %s", header->frame_id.c_str());
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_capture_with_settings_from_yml_cpp");
  ros::NodeHandle n;

  ROS_INFO("Starting sample_capture_with_settings_from_yml.cpp");

  CHECK(ros::service::waitForService("/zivid_camera/capture", default_wait_duration));

  ros::AsyncSpinner spinner(3);
  spinner.start();

  auto points_sub = n.subscribe("/zivid_camera/points/xyzrgba", 1, on_points);
  CHECK(waitForPublisher(points_sub, default_wait_duration));
  auto image_color_sub = n.subscribe("/zivid_camera/color/image_color", 1, on_image_color);
  CHECK(waitForPublisher(image_color_sub, default_wait_duration));
  auto acquisition_done_sub = n.subscribe("/zivid_camera/acquisition_done", 1, on_acquisition_done);
  CHECK(waitForPublisher(acquisition_done_sub, default_wait_duration));

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