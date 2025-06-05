#include <zivid_camera/Capture.h>
#include <zivid_camera/LoadSettingsFromFile.h>
#include <string>
#include <vector>
#include <array>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
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

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

struct ToString
{
  template <typename T>
  std::string operator()(const T& v) const
  {
    return std::to_string(v);
  }
};

template <typename Container, typename ToStringFunc>
std::string join_list_to_string(const Container& container, ToStringFunc to_string_func)
{
  std::string result;
  for (typename Container::const_iterator it = container.begin(); it != container.end(); ++it)
  {
    if (it != container.begin())
    {
      result += ", ";
    }
    result += to_string_func(*it);
  }
  return result;
}

void on_color_camera_info(const sensor_msgs::CameraInfo& msg)
{
  ROS_INFO("Color CameraInfo received");
  ToString to_string;

  ROS_INFO_STREAM("Received color camera info:\n"
                  << "  width: " << msg.width << "\n"
                  << "  height: " << msg.height << "\n"
                  << "  distortion_model: " << msg.distortion_model << "\n"
                  << "  D: [" << join_list_to_string(msg.D, to_string) << "]\n"
                  << "  K: [" << join_list_to_string(msg.K, to_string) << "]\n"
                  << "  R: [" << join_list_to_string(msg.R, to_string) << "]\n"
                  << "  P: [" << join_list_to_string(msg.P, to_string) << "]");
}

void on_depth_camera_info(const sensor_msgs::CameraInfo& msg)
{
  ROS_INFO("Depth CameraInfo received");
  ToString to_string;

  ROS_INFO_STREAM("Received depth camera info:\n"
                  << "  width: " << msg.width << "\n"
                  << "  height: " << msg.height << "\n"
                  << "  distortion_model: " << msg.distortion_model << "\n"
                  << "  D: [" << join_list_to_string(msg.D, to_string) << "]\n"
                  << "  K: [" << join_list_to_string(msg.K, to_string) << "]\n"
                  << "  R: [" << join_list_to_string(msg.R, to_string) << "]\n"
                  << "  P: [" << join_list_to_string(msg.P, to_string) << "]");
}

void set_intrinsics_source(const std::string& value)
{
  // Use the private namespace of the zivid_camera node
  ros::NodeHandle nh("/zivid_camera/zivid_camera");

  std::string param_name = "intrinsics_source";  // Node-private parameter
  ROS_INFO_STREAM("Setting intrinsics source to: " << value);

  nh.setParam(param_name, value);

  std::string read_value;
  if (nh.getParam(param_name, read_value))
  {
    if (read_value != value)
    {
      ROS_WARN("Expected '%s' but got '%s'", value.c_str(), read_value.c_str());
    }
    else
    {
      ROS_INFO("Param set successfully, value: %s", read_value.c_str());
    }
  }
  else
  {
    ROS_WARN("Failed to read back param");
  }
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_intrinsics_cpp");
  ros::NodeHandle nh;

  ROS_INFO("Starting sample_intrinsics.cpp");

  CHECK(ros::service::waitForService("/zivid_camera/capture", default_wait_duration));
  auto color_camera_info_subscription = nh.subscribe("/zivid_camera/color/camera_info", 1, on_color_camera_info);
  auto depth_camera_info_subscription = nh.subscribe("/zivid_camera/depth/camera_info", 1, on_depth_camera_info);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string samples_path = ros::package::getPath("zivid_samples");
  std::string settings_path = samples_path + "/settings/camera_settings.yml";
  ROS_INFO("Loading settings from: %s", settings_path.c_str());
  zivid_camera::LoadSettingsFromFile load_settings_from_file;
  load_settings_from_file.request.file_path = settings_path;
  CHECK(ros::service::call("/zivid_camera/load_settings_from_file", load_settings_from_file));

  set_intrinsics_source("camera");
  capture();

  set_intrinsics_source("frame");
  capture();

  ros::waitForShutdown();

  return 0;
}