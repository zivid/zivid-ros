#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/CaptureAssistantSuggestSettings.h>

#define CHECK(cmd)                                      \
  do {                                                  \
    if (!cmd) {                                         \
      throw std::runtime_error{"\"" #cmd "\" failed!"}; \
    }                                                   \
  } while (false)

namespace
{
const ros::Duration default_wait_duration{30};
constexpr auto ca_suggest_settings_service_name =
  "/zivid_camera/capture_assistant/suggest_settings";

void capture_assistant_suggest_settings()
{
  zivid_camera::CaptureAssistantSuggestSettings cass;
  cass.request.max_capture_time = ros::Duration{1.20};
  cass.request.ambient_light_frequency =
    zivid_camera::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_NONE;

  ROS_INFO_STREAM(
    "Calling " << ca_suggest_settings_service_name
               << " with max capture time = " << cass.request.max_capture_time << " sec");
  CHECK(ros::service::call(ca_suggest_settings_service_name, cass));
}

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

void on_points(const sensor_msgs::PointCloud2ConstPtr &) { ROS_INFO("PointCloud received"); }

void on_image_color(const sensor_msgs::ImageConstPtr &) { ROS_INFO("2D color image received"); }

}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "sample_capture_assistant_cpp");
  ros::NodeHandle n;

  ROS_INFO("Starting sample_capture_assistant.cpp");

  CHECK(ros::service::waitForService(ca_suggest_settings_service_name, default_wait_duration));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto points_sub = n.subscribe("/zivid_camera/points/xyzrgba", 1, on_points);
  auto image_color_sub = n.subscribe("/zivid_camera/color/image_color", 1, on_image_color);

  capture_assistant_suggest_settings();

  capture();

  ros::waitForShutdown();

  return 0;
}