#ifdef __clang__
#pragma clang diagnostic push
// Errors to ignore for this entire file
#pragma clang diagnostic ignored "-Wglobal-constructors"  // error triggered by gtest fixtures
#endif

#include <zivid_camera/CameraInfoSerialNumber.h>
#include <zivid_camera/CameraInfoModelName.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/Capture2D.h>
#include <zivid_camera/CaptureAssistantSuggestSettings.h>
#include <zivid_camera/LoadSettingsFromFile.h>
#include <zivid_camera/LoadSettings2DFromFile.h>
#include <zivid_camera/SettingsAcquisitionConfig.h>
#include <zivid_camera/Settings2DAcquisitionConfig.h>
#include <zivid_camera/SettingsConfig.h>
#include <zivid_camera/Settings2DConfig.h>
#include <zivid_camera/IsConnected.h>

#include <Zivid/Application.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Frame.h>
#include <Zivid/Camera.h>
#include <Zivid/Version.h>

#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "gtest_include_wrapper.h"

#include <boost/filesystem.hpp>

#include <ros/ros.h>

using SecondsD = std::chrono::duration<double>;

namespace
{
template <typename T>
struct DependentFalse : std::false_type
{
};

template <class Rep, class Period>
ros::Duration toRosDuration(const std::chrono::duration<Rep, Period>& d)
{
  return ros::Duration{ std::chrono::duration_cast<SecondsD>(d).count() };
}

std::string testDataDir()
{
  return (boost::filesystem::path{ __FILE__ }.parent_path() / "data").string();
}

}  // namespace

class ZividNodeTestBase : public testing::Test
{
protected:
  ZividNodeTestBase()
  {
    const auto test_name = testing::UnitTest::GetInstance()->current_test_info()->name();
    std::cerr << "Start of test " << test_name << "\n";
    printLine();
  }

  ~ZividNodeTestBase() override
  {
    const auto test_name = testing::UnitTest::GetInstance()->current_test_info()->name();
    printLine();
    std::cerr << "End of test " << test_name << "\n";
  }

  void printLine()
  {
    std::cerr << std::string(80, '-') << "\n";
  }
};

class ZividNodeTest : public ZividNodeTestBase
{
protected:
  ros::NodeHandle nh_;

  const ros::Duration node_ready_wait_duration{ 20 };
  const ros::Duration short_wait_duration{ 0.5 };
  const ros::Duration medium_wait_duration{ 1.0 };
  const ros::Duration dr_get_max_wait_duration{ 5 };
  static constexpr auto capture_service_name = "/zivid_camera/capture";
  static constexpr auto capture_2d_service_name = "/zivid_camera/capture_2d";
  static constexpr auto capture_assistant_suggest_settings_service_name = "/zivid_camera/capture_assistant/"
                                                                          "suggest_settings";
  static constexpr auto load_settings_from_file_service_name = "/zivid_camera/load_settings_from_file";
  static constexpr auto load_settings_2d_from_file_service_name = "/zivid_camera/load_settings_2d_from_file";

  static constexpr auto color_camera_info_topic_name = "/zivid_camera/color/camera_info";
  static constexpr auto color_image_color_topic_name = "/zivid_camera/color/image_color";
  static constexpr auto depth_camera_info_topic_name = "/zivid_camera/depth/camera_info";
  static constexpr auto depth_image_topic_name = "/zivid_camera/depth/image";
  static constexpr auto snr_camera_info_topic_name = "/zivid_camera/depth/camera_info";
  static constexpr auto snr_image_topic_name = "/zivid_camera/snr/image";
  static constexpr auto points_xyz_topic_name = "/zivid_camera/points/xyz";
  static constexpr auto points_xyzrgba_topic_name = "/zivid_camera/points/xyzrgba";
  static constexpr auto normals_xyz_topic_name = "/zivid_camera/normals/xyz";
  static constexpr size_t num_settings_acquisition_dr_servers = 10;
  static constexpr size_t num_settings_2d_acquisition_dr_servers = 1;
  static constexpr auto file_camera_path = "/usr/share/Zivid/data/FileCameraZividOne.zfc";

  template <typename Type>
  class SubscriptionWrapper
  {
  public:
    static SubscriptionWrapper<Type> make(ros::NodeHandle& nh, const std::string& name)
    {
      auto w = SubscriptionWrapper<Type>();
      boost::function<void(const boost::shared_ptr<const Type>&)> cb = [impl = w.impl_.get()](const auto& v) mutable {
        impl->num_messages_++;
        impl->last_message_ = *v;
      };
      w.impl_->subscriber_ = nh.subscribe<Type>(name, 1, cb);
      return w;
    }

    const std::optional<Type>& lastMessage() const
    {
      return impl_->last_message_;
    }

    std::size_t numMessages() const
    {
      return impl_->num_messages_;
    }

  private:
    SubscriptionWrapper() : impl_(std::make_unique<Impl>())
    {
    }
    struct Impl
    {
      Impl() : num_messages_(0)
      {
      }
      ros::Subscriber subscriber_;
      std::optional<Type> last_message_;
      std::size_t num_messages_;
    };
    std::unique_ptr<Impl> impl_;
  };

  void waitForReady()
  {
    ASSERT_TRUE(ros::service::waitForService(capture_service_name, node_ready_wait_duration));
  }

  void enableFirst3DAcquisition()
  {
    dynamic_reconfigure::Client<zivid_camera::SettingsAcquisitionConfig> acquisition_0_client("/zivid_camera/settings/"
                                                                                              "acquisition_0/");
    zivid_camera::SettingsAcquisitionConfig acquisition_0_cfg;
    ASSERT_TRUE(acquisition_0_client.getDefaultConfiguration(acquisition_0_cfg, dr_get_max_wait_duration));
    acquisition_0_cfg.enabled = true;
    ASSERT_TRUE(acquisition_0_client.setConfiguration(acquisition_0_cfg));
  }

  void enableFirst2DAcquisition()
  {
    dynamic_reconfigure::Client<zivid_camera::Settings2DAcquisitionConfig> acquisition_0_client("/zivid_camera/"
                                                                                                "settings_2d/"
                                                                                                "acquisition_0/");
    zivid_camera::Settings2DAcquisitionConfig cfg;
    ASSERT_TRUE(acquisition_0_client.getDefaultConfiguration(cfg, dr_get_max_wait_duration));
    cfg.enabled = true;
    ASSERT_TRUE(acquisition_0_client.setConfiguration(cfg));
  }

  void enableFirst3DAcquisitionAndCapture()
  {
    enableFirst3DAcquisition();
    zivid_camera::Capture capture;
    ASSERT_TRUE(ros::service::call(capture_service_name, capture));
    short_wait_duration.sleep();
  }

  template <class Type>
  SubscriptionWrapper<Type> subscribe(const std::string& name)
  {
    return SubscriptionWrapper<Type>::make(nh_, name);
  }

  template <class A, class B>
  void assertArrayFloatEq(const A& actual, const B& expected) const
  {
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < actual.size(); i++)
    {
      ASSERT_FLOAT_EQ(actual[i], expected[i]);
    }
  }

  void assertSensorMsgsPointCloud2Meta(const sensor_msgs::PointCloud2& point_cloud, std::size_t width,
                                       std::size_t height, std::size_t point_step)
  {
    ASSERT_EQ(point_cloud.width, width);
    ASSERT_EQ(point_cloud.height, height);
    ASSERT_EQ(point_cloud.point_step, point_step);
    ASSERT_EQ(point_cloud.row_step, point_cloud.width * point_cloud.point_step);
    ASSERT_EQ(point_cloud.is_dense, false);
    ASSERT_EQ(point_cloud.data.size(), point_cloud.width * point_cloud.height * point_cloud.point_step);
  }

  void assertSensorMsgsImageMeta(const sensor_msgs::Image& image, std::size_t width, std::size_t height,
                                 std::size_t bytes_per_pixel, const std::string& encoding)
  {
    ASSERT_EQ(image.width, width);
    ASSERT_EQ(image.height, height);
    ASSERT_EQ(image.step, bytes_per_pixel * image.width);
    ASSERT_EQ(image.data.size(), image.step * image.height);
    ASSERT_EQ(image.encoding, encoding);
    ASSERT_EQ(image.is_bigendian, false);
  }

  void assertSensorMsgsImageContents(const sensor_msgs::Image& image,
                                     const Zivid::Array2D<Zivid::ColorRGBA>& expected_rgba)
  {
    ASSERT_EQ(image.width, expected_rgba.width());
    ASSERT_EQ(image.height, expected_rgba.height());
    for (std::size_t i = 0; i < expected_rgba.size(); i++)
    {
      const auto expectedPixel = expected_rgba(i);
      const auto index = i * 4U;
      ASSERT_EQ(image.data[index], expectedPixel.r);
      ASSERT_EQ(image.data[index + 1], expectedPixel.g);
      ASSERT_EQ(image.data[index + 2], expectedPixel.b);
      ASSERT_EQ(image.data[index + 3], expectedPixel.a);
      ASSERT_EQ(expectedPixel.a, 255);
    }
  }

  template <typename FieldType>
  void assertPointCloud2Field(const FieldType& field, const std::string& name, uint32_t offset, uint32_t datatype,
                              uint32_t count)
  {
    ASSERT_EQ(field.name, name);
    ASSERT_EQ(field.offset, offset);
    ASSERT_EQ(field.datatype, datatype);
    ASSERT_EQ(field.count, count);
  }

  void assertCameraInfoForFileCamera(const sensor_msgs::CameraInfo& ci) const
  {
    ASSERT_EQ(ci.width, 1920U);
    ASSERT_EQ(ci.height, 1200U);
    ASSERT_EQ(ci.distortion_model, "plumb_bob");

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    assertArrayFloatEq(
        ci.K, std::array<double, 9>{ 2759.12329102, 0, 958.78460693, 0, 2758.73681641, 634.94018555, 0, 0, 1 });

    // R = I
    assertArrayFloatEq(ci.R, std::array<double, 9>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 });

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    assertArrayFloatEq(ci.P, std::array<double, 12>{ 2759.12329102, 0, 958.78460693, 0, 0, 2758.73681641, 634.94018555,
                                                     0, 0, 0, 1, 0 });
  }
};

class TestWithFileCamera : public ZividNodeTest
{
protected:
  TestWithFileCamera() : camera_(zivid_.createFileCamera(file_camera_path))
  {
    waitForReady();
  }
  Zivid::Application zivid_;
  Zivid::Camera camera_;
};

TEST_F(ZividNodeTest, testServiceCameraInfoModelName)
{
  waitForReady();
  zivid_camera::CameraInfoModelName model_name;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/model_name", model_name));
  ASSERT_EQ(model_name.response.model_name, std::string("FileCamera-") + ZIVID_CORE_VERSION);
}

TEST_F(ZividNodeTest, testServiceCameraInfoSerialNumber)
{
  waitForReady();
  zivid_camera::CameraInfoSerialNumber serial_number;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/serial_number", serial_number));
  ASSERT_EQ(serial_number.response.serial_number, "F1");
}

TEST_F(ZividNodeTest, testServiceIsConnected)
{
  waitForReady();
  zivid_camera::IsConnected is_connected;
  ASSERT_TRUE(ros::service::call("/zivid_camera/is_connected", is_connected));
  ASSERT_EQ(is_connected.response.is_connected, true);
}

TEST_F(ZividNodeTest, testCapturePublishesTopics)
{
  waitForReady();

  auto color_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name);
  auto color_image_color_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);
  auto depth_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(depth_camera_info_topic_name);
  auto depth_image_sub = subscribe<sensor_msgs::Image>(depth_image_topic_name);
  auto snr_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(snr_camera_info_topic_name);
  auto snr_image_sub = subscribe<sensor_msgs::Image>(snr_image_topic_name);
  auto points_xyz_sub = subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name);
  auto points_xyzrgba_sub = subscribe<sensor_msgs::PointCloud2>(points_xyzrgba_topic_name);
  auto normals_xyz_sub = subscribe<sensor_msgs::PointCloud2>(normals_xyz_topic_name);

  auto assert_num_topics_received = [&](std::size_t numTopics) {
    ASSERT_EQ(color_camera_info_sub.numMessages(), numTopics);
    ASSERT_EQ(color_image_color_sub.numMessages(), numTopics);
    ASSERT_EQ(depth_camera_info_sub.numMessages(), numTopics);
    ASSERT_EQ(depth_image_sub.numMessages(), numTopics);
    ASSERT_EQ(snr_camera_info_sub.numMessages(), numTopics);
    ASSERT_EQ(snr_image_sub.numMessages(), numTopics);
    ASSERT_EQ(points_xyz_sub.numMessages(), numTopics);
    ASSERT_EQ(points_xyzrgba_sub.numMessages(), numTopics);
    ASSERT_EQ(normals_xyz_sub.numMessages(), numTopics);
  };

  medium_wait_duration.sleep();
  assert_num_topics_received(0);

  zivid_camera::Capture capture;
  // Capture fails when no acquisitions are enabled
  ASSERT_FALSE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(0);

  enableFirst3DAcquisition();

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(1);

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(2);

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(3);

  short_wait_duration.sleep();
  assert_num_topics_received(3);
}

class CaptureOutputTest : public TestWithFileCamera
{
protected:
  Zivid::PointCloud captureViaSDKDefaultSettings()
  {
    return camera_.capture(Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } })
        .pointCloud();
  }

  Zivid::Frame2D capture2DViaSDKDefaultSettings()
  {
    return camera_.capture(Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } });
  }

  Zivid::PointCloud captureViaSDK(const Zivid::Settings& settings)
  {
    return camera_.capture(settings).pointCloud();
  }

  void compareFloat(float a, float b, float delta = 1e-6f) const
  {
    if (std::isnan(a))
    {
      ASSERT_TRUE(std::isnan(b));
    }
    else
    {
      ASSERT_NEAR(a, b, delta);
    }
  }

  void comparePointCoordinate(float ros_coordinate, float sdk_coordinate) const
  {
    if (std::isnan(ros_coordinate))
    {
      ASSERT_TRUE(std::isnan(sdk_coordinate));
    }
    else
    {
      // Output from the SDK is millimeters. In the ROS driver a transform is applied to convert
      // the ROS points to meters.
      const float delta = 0.000001f;
      ASSERT_NEAR(ros_coordinate, sdk_coordinate / 1000, delta);
    }
  }
};

TEST_F(CaptureOutputTest, testCapturePointsXYZGBA)
{
  auto points_sub = subscribe<sensor_msgs::PointCloud2>(points_xyzrgba_topic_name);

  enableFirst3DAcquisitionAndCapture();

  const auto& last_pc2 = points_sub.lastMessage();
  ASSERT_TRUE(last_pc2.has_value());
  assertSensorMsgsPointCloud2Meta(*last_pc2, 1920U, 1200U, 16U);
  ASSERT_EQ(last_pc2->fields.size(), 4U);
  assertPointCloud2Field(last_pc2->fields[0], "x", 0, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(last_pc2->fields[1], "y", 4, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(last_pc2->fields[2], "z", 8, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(last_pc2->fields[3], "rgba", 12, sensor_msgs::PointField::FLOAT32, 1);

  const auto point_cloud = captureViaSDKDefaultSettings();
  const auto expected_xyzrgba = point_cloud.copyData<Zivid::PointXYZColorRGBA>();
  ASSERT_EQ(last_pc2->width, expected_xyzrgba.width());
  ASSERT_EQ(last_pc2->height, expected_xyzrgba.height());
  for (std::size_t i = 0; i < expected_xyzrgba.size(); i++)
  {
    const uint8_t* point_ptr = &last_pc2->data[i * last_pc2->point_step];
    const float x = *reinterpret_cast<const float*>(&point_ptr[0]);
    const float y = *reinterpret_cast<const float*>(&point_ptr[4]);
    const float z = *reinterpret_cast<const float*>(&point_ptr[8]);
    const uint32_t argb = *reinterpret_cast<const uint32_t*>(&point_ptr[12]);
    const auto& expected = expected_xyzrgba(i);

    comparePointCoordinate(x, expected.point.x);
    comparePointCoordinate(y, expected.point.y);
    comparePointCoordinate(z, expected.point.z);
    const auto expected_argb = static_cast<uint32_t>(expected.color.a << 24) |
                               static_cast<uint32_t>(expected.color.r << 16) |
                               static_cast<uint32_t>(expected.color.g << 8) | static_cast<uint32_t>(expected.color.b);
    ASSERT_EQ(argb, expected_argb);
  }
}

TEST_F(CaptureOutputTest, testCapturePointsXYZ)
{
  auto points_sub = subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name);

  enableFirst3DAcquisitionAndCapture();

  const auto& point_cloud = points_sub.lastMessage();
  ASSERT_TRUE(point_cloud.has_value());
  assertSensorMsgsPointCloud2Meta(*point_cloud, 1920U, 1200U,
                                  16U);  // 3x4 bytes for xyz + 4 bytes padding (w) = 16 bytes total
  ASSERT_EQ(point_cloud->fields.size(), 3U);
  assertPointCloud2Field(point_cloud->fields[0], "x", 0, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(point_cloud->fields[1], "y", 4, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(point_cloud->fields[2], "z", 8, sensor_msgs::PointField::FLOAT32, 1);

  auto point_cloud_sdk = captureViaSDKDefaultSettings();
  auto expected_xyz = point_cloud_sdk.copyData<Zivid::PointXYZ>();
  ASSERT_EQ(point_cloud->width, expected_xyz.width());
  ASSERT_EQ(point_cloud->height, expected_xyz.height());
  for (std::size_t i = 0; i < expected_xyz.size(); i++)
  {
    const uint8_t* point_ptr = &point_cloud->data[i * point_cloud->point_step];
    const float x = *reinterpret_cast<const float*>(&point_ptr[0]);
    const float y = *reinterpret_cast<const float*>(&point_ptr[4]);
    const float z = *reinterpret_cast<const float*>(&point_ptr[8]);

    const auto expected = expected_xyz(i);
    comparePointCoordinate(x, expected.x);
    comparePointCoordinate(y, expected.y);
    comparePointCoordinate(z, expected.z);
  }
}

#if (ZIVID_CORE_VERSION_MAJOR >= 2 && ZIVID_CORE_VERSION_MINOR >= 9) || ZIVID_CORE_VERSION_MAJOR > 2
TEST_F(CaptureOutputTest, testCapturePointsXYZWithROI)
{
  auto points_sub = subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name);
  dynamic_reconfigure::Client<zivid_camera::SettingsConfig> settings_client("/zivid_camera/settings/");
  zivid_camera::SettingsConfig configOriginal;
  ASSERT_TRUE(settings_client.getDefaultConfiguration(configOriginal, dr_get_max_wait_duration));
  auto config = configOriginal;
  config.region_of_interest_box_enabled = true;
  config.region_of_interest_box_point_o_x = 1;
  config.region_of_interest_box_point_o_y = 0;
  config.region_of_interest_box_point_o_z = 619;
  config.region_of_interest_box_point_a_x = 111;
  config.region_of_interest_box_point_a_y = 2;
  config.region_of_interest_box_point_a_z = 620;
  config.region_of_interest_box_point_b_x = 3;
  config.region_of_interest_box_point_b_y = 110;
  config.region_of_interest_box_point_b_z = 621;
  config.region_of_interest_box_extents_min = -10;
  config.region_of_interest_box_extents_max = 10;
  ASSERT_TRUE(settings_client.setConfiguration(config));

  enableFirst3DAcquisitionAndCapture();
  const auto& point_cloud = points_sub.lastMessage();
  ASSERT_TRUE(point_cloud.has_value());

  const auto point_cloud_sdk = captureViaSDK(Zivid::Settings{
      Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
      Zivid::Settings::RegionOfInterest::Box{
          Zivid::Settings::RegionOfInterest::Box::Enabled::yes,
          Zivid::Settings::RegionOfInterest::Box::PointO{ 1, 0, 619 },
          Zivid::Settings::RegionOfInterest::Box::PointA{ 111, 2, 620 },
          Zivid::Settings::RegionOfInterest::Box::PointB{ 3, 110, 621 },
          Zivid::Settings::RegionOfInterest::Box::Extents{ -10, 10 },
      },
  });

  auto expected = point_cloud_sdk.copyData<Zivid::PointXYZ>();
  const auto numExpectedNaNZ = [&] {
    size_t count = 0;
    for (size_t i = 0; i < expected.size(); ++i)
    {
      count += std::isnan(expected(i).z);
    }
    return count;
  }();
  ASSERT_EQ(numExpectedNaNZ, 2154265U);

  for (size_t i = 0; i < expected.size(); ++i)
  {
    const uint8_t* point_ptr = &point_cloud->data[i * point_cloud->point_step];
    const float x = *reinterpret_cast<const float*>(&point_ptr[0]);
    const float y = *reinterpret_cast<const float*>(&point_ptr[4]);
    const float z = *reinterpret_cast<const float*>(&point_ptr[8]);
    comparePointCoordinate(x, expected(i).x);
    comparePointCoordinate(y, expected(i).y);
    comparePointCoordinate(z, expected(i).z);
  }

  ASSERT_TRUE(settings_client.setConfiguration(configOriginal));
}
#endif

TEST_F(CaptureOutputTest, testCapture3DColorImage)
{
  auto color_image_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);

  enableFirst3DAcquisitionAndCapture();
  const auto image = color_image_sub.lastMessage();
  ASSERT_TRUE(image.has_value());
  const std::size_t bytes_per_pixel = 4U;
  assertSensorMsgsImageMeta(*image, 1920U, 1200U, bytes_per_pixel, "rgba8");

  const auto point_cloud = captureViaSDKDefaultSettings();
  const auto expected_rgba = point_cloud.copyData<Zivid::ColorRGBA>();
  assertSensorMsgsImageContents(*image, expected_rgba);
}

TEST_F(CaptureOutputTest, testCaptureDepthImage)
{
  auto depth_image_sub = subscribe<sensor_msgs::Image>(depth_image_topic_name);

  enableFirst3DAcquisitionAndCapture();

  const auto image = depth_image_sub.lastMessage();
  ASSERT_TRUE(image.has_value());
  const std::size_t bytes_per_pixel = 4U;
  assertSensorMsgsImageMeta(*image, 1920U, 1200U, bytes_per_pixel, "32FC1");

  const auto point_cloud = captureViaSDKDefaultSettings();
  const auto expected_z = point_cloud.copyData<Zivid::PointZ>();
  ASSERT_EQ(image->width, expected_z.width());
  ASSERT_EQ(image->height, expected_z.height());
  for (std::size_t i = 0; i < expected_z.size(); i++)
  {
    const auto expected = expected_z(i);
    const float z = *reinterpret_cast<const float*>(image->data.data() + i * bytes_per_pixel);
    comparePointCoordinate(z, expected.z);
  }
}

TEST_F(CaptureOutputTest, testCaptureSNRImage)
{
  auto snr_image_sub = subscribe<sensor_msgs::Image>(snr_image_topic_name);

  enableFirst3DAcquisitionAndCapture();

  const auto image = snr_image_sub.lastMessage();
  ASSERT_TRUE(image.has_value());
  const std::size_t bytes_per_pixel = 4U;
  assertSensorMsgsImageMeta(*image, 1920U, 1200U, bytes_per_pixel, "32FC1");

  const auto point_cloud = captureViaSDKDefaultSettings();
  const auto expected_snr = point_cloud.copyData<Zivid::SNR>();
  ASSERT_EQ(image->width, expected_snr.width());
  ASSERT_EQ(image->height, expected_snr.height());
  for (std::size_t i = 0; i < expected_snr.size(); i++)
  {
    const auto expected = expected_snr(i);
    const float snr = *reinterpret_cast<const float*>(image->data.data() + i * bytes_per_pixel);
    ASSERT_EQ(snr, expected.value);
  }
}

TEST_F(CaptureOutputTest, testCaptureNormals)
{
  auto normals_sub = subscribe<sensor_msgs::PointCloud2>(normals_xyz_topic_name);

  enableFirst3DAcquisitionAndCapture();

  const auto& point_cloud = normals_sub.lastMessage();
  ASSERT_TRUE(point_cloud.has_value());
  assertSensorMsgsPointCloud2Meta(*point_cloud, 1920U, 1200U,
                                  3U * sizeof(float));  // 12 bytes total
  ASSERT_EQ(point_cloud->fields.size(), 3U);
  assertPointCloud2Field(point_cloud->fields[0], "normal_x", 0, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(point_cloud->fields[1], "normal_y", 4, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(point_cloud->fields[2], "normal_z", 8, sensor_msgs::PointField::FLOAT32, 1);

  auto point_cloud_sdk = captureViaSDKDefaultSettings();
  auto expected_normal_xyz_before_transform = point_cloud_sdk.copyData<Zivid::NormalXYZ>();
  ASSERT_EQ(point_cloud->width, expected_normal_xyz_before_transform.width());
  ASSERT_EQ(point_cloud->height, expected_normal_xyz_before_transform.height());
  ASSERT_EQ(point_cloud->width * point_cloud->height, expected_normal_xyz_before_transform.size());

  // Transform from mm to m (like is done internally in Zivid driver)
  const float scale = 0.001f;
  point_cloud_sdk.transform(Zivid::Matrix4x4{ scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, 1 });
  auto expected_normal_xyz_after_transform = point_cloud_sdk.copyData<Zivid::NormalXYZ>();
  ASSERT_EQ(expected_normal_xyz_after_transform.size(), expected_normal_xyz_before_transform.size());

  for (std::size_t i = 0; i < expected_normal_xyz_before_transform.size(); i++)
  {
    const uint8_t* cloud_ptr = &point_cloud->data[i * point_cloud->point_step];
    const float normal_x = *reinterpret_cast<const float*>(&cloud_ptr[0]);
    const float normal_y = *reinterpret_cast<const float*>(&cloud_ptr[4]);
    const float normal_z = *reinterpret_cast<const float*>(&cloud_ptr[8]);

    const auto& expected_sdk_before_transform = expected_normal_xyz_before_transform(i);
    // We do a transform in the ROS driver to scale from mm to meters. However, `expected_normal_xyz`
    // are calculated without transform, so we need a slightly higher delta to compare.
    constexpr float delta = 0.001f;
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_x, expected_sdk_before_transform.x, delta));
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_y, expected_sdk_before_transform.y, delta));
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_z, expected_sdk_before_transform.z, delta));

    const auto& expected_sdk_after_transform = expected_normal_xyz_after_transform(i);
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_x, expected_sdk_after_transform.x));
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_y, expected_sdk_after_transform.y));
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_z, expected_sdk_after_transform.z));
  }
}

TEST_F(TestWithFileCamera, testSettingsEngine)
{
  waitForReady();
  enableFirst3DAcquisition();
  auto points_sub = subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name);

  dynamic_reconfigure::Client<zivid_camera::SettingsConfig> settings_client("/zivid_camera/settings/");
  zivid_camera::SettingsConfig settings_cfg;
  ASSERT_TRUE(settings_client.getDefaultConfiguration(settings_cfg, dr_get_max_wait_duration));
  ASSERT_EQ(settings_cfg.experimental_engine, zivid_camera::Settings_ExperimentalEnginePhase);
  settings_cfg.experimental_engine = zivid_camera::Settings_ExperimentalEngineStripe;
  settings_cfg.processing_filters_reflection_removal_enabled = true;
  settings_cfg.processing_filters_experimental_contrast_distortion_correction_enabled = true;
  ASSERT_TRUE(settings_client.setConfiguration(settings_cfg));

  zivid_camera::Capture capture;
  // Capture fails here because file camera does not support Stripe engine
  ASSERT_FALSE(ros::service::call(capture_service_name, capture));
  ASSERT_EQ(points_sub.numMessages(), 0U);

  settings_cfg.experimental_engine = zivid_camera::Settings_ExperimentalEnginePhase;
  ASSERT_TRUE(settings_client.setConfiguration(settings_cfg));
  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  ASSERT_EQ(points_sub.numMessages(), 1U);
}

TEST_F(ZividNodeTest, testCaptureCameraInfo)
{
  waitForReady();

  auto color_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name);
  auto depth_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(depth_camera_info_topic_name);
  auto snr_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(snr_camera_info_topic_name);

  enableFirst3DAcquisitionAndCapture();

  ASSERT_EQ(color_camera_info_sub.numMessages(), 1U);
  ASSERT_EQ(depth_camera_info_sub.numMessages(), 1U);
  ASSERT_EQ(snr_camera_info_sub.numMessages(), 1U);

  assertCameraInfoForFileCamera(*color_camera_info_sub.lastMessage());
  assertCameraInfoForFileCamera(*depth_camera_info_sub.lastMessage());
  assertCameraInfoForFileCamera(*snr_camera_info_sub.lastMessage());
}

TEST_F(CaptureOutputTest, testCapture2D)
{
  auto color_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name);
  auto color_image_color_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);

  auto assert_num_topics_received = [&](std::size_t num_topics) {
    ASSERT_EQ(color_camera_info_sub.numMessages(), num_topics);
    ASSERT_EQ(color_image_color_sub.numMessages(), num_topics);
  };

  short_wait_duration.sleep();
  assert_num_topics_received(0);

  // Capture fails when no acquisitions are enabled
  zivid_camera::Capture2D capture;
  ASSERT_FALSE(ros::service::call(capture_2d_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(0);

  enableFirst2DAcquisition();
  ASSERT_TRUE(ros::service::call(capture_2d_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(1);

  auto verify_image_and_camera_info = [this](const auto& img, const auto& info) {
    assertCameraInfoForFileCamera(info);
    assertSensorMsgsImageMeta(img, 1920U, 1200U, 4U, "rgba8");
    assertSensorMsgsImageContents(img, capture2DViaSDKDefaultSettings().imageRGBA());
  };

  verify_image_and_camera_info(*color_image_color_sub.lastMessage(), *color_camera_info_sub.lastMessage());

  short_wait_duration.sleep();
  assert_num_topics_received(1);

  ASSERT_TRUE(ros::service::call(capture_2d_service_name, capture));
  short_wait_duration.sleep();
  assert_num_topics_received(2);
  verify_image_and_camera_info(*color_image_color_sub.lastMessage(), *color_camera_info_sub.lastMessage());
}

class DynamicReconfigureMinMaxDefaultTest : public TestWithFileCamera
{
protected:
  template <typename Setting>
  typename Setting::ValueType sdkDefaultValue()
  {
    return Zivid::Experimental::SettingsInfo::defaultValue<Setting>(camera_.info()).value();
  }

  template <typename Setting>
  auto sdkValidRange()
  {
    return Zivid::Experimental::SettingsInfo::validRange<Setting>(camera_.info());
  }

  template <typename ZividSettingsType, typename ConfigType>
  void testGeneralMinMaxDefault(const std::string& service_name)
  {
    ASSERT_TRUE(ros::service::waitForService(service_name + "/set_parameters", short_wait_duration));

    dynamic_reconfigure::Client<ConfigType> client(service_name);

    ConfigType default_cfg;
    ASSERT_TRUE(client.getDefaultConfiguration(default_cfg, dr_get_max_wait_duration));
    ConfigType min_cfg;
    ASSERT_TRUE(client.getMinConfiguration(min_cfg, dr_get_max_wait_duration));
    ConfigType max_cfg;
    ASSERT_TRUE(client.getMaxConfiguration(max_cfg, dr_get_max_wait_duration));

    using BlueBalance = typename ZividSettingsType::Processing::Color::Balance::Blue;
    ASSERT_EQ(default_cfg.processing_color_balance_blue, sdkDefaultValue<BlueBalance>());
    ASSERT_EQ(min_cfg.processing_color_balance_blue, sdkValidRange<BlueBalance>().min());
    ASSERT_EQ(max_cfg.processing_color_balance_blue, sdkValidRange<BlueBalance>().max());

    using GreenBalance = typename ZividSettingsType::Processing::Color::Balance::Green;
    ASSERT_EQ(default_cfg.processing_color_balance_green, sdkDefaultValue<GreenBalance>());
    ASSERT_EQ(min_cfg.processing_color_balance_green, sdkValidRange<GreenBalance>().min());
    ASSERT_EQ(max_cfg.processing_color_balance_green, sdkValidRange<GreenBalance>().max());

    using RedBalance = typename ZividSettingsType::Processing::Color::Balance::Red;
    ASSERT_EQ(default_cfg.processing_color_balance_red, sdkDefaultValue<RedBalance>());
    ASSERT_EQ(min_cfg.processing_color_balance_red, sdkValidRange<RedBalance>().min());
    ASSERT_EQ(max_cfg.processing_color_balance_red, sdkValidRange<RedBalance>().max());

    if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings>)
    {
      using OutlierRemovalThreshold = typename ZividSettingsType::Processing::Filters::Outlier::Removal::Threshold;
      ASSERT_EQ(default_cfg.processing_filters_outlier_removal_threshold, sdkDefaultValue<OutlierRemovalThreshold>());
      ASSERT_EQ(min_cfg.processing_filters_outlier_removal_threshold, sdkValidRange<OutlierRemovalThreshold>().min());
      ASSERT_EQ(max_cfg.processing_filters_outlier_removal_threshold, sdkValidRange<OutlierRemovalThreshold>().max());
    }
  }

  template <typename ZividSettingsType, typename ConfigType>
  void testAcquisitionMinMaxDefault(const std::string& prefix, std::size_t num_acquisition_servers)
  {
    for (std::size_t i = 0; i < num_acquisition_servers; i++)
    {
      ASSERT_TRUE(ros::service::waitForService(prefix + "acquisition_" + std::to_string(i) + "/set_parameters",
                                               short_wait_duration));

      dynamic_reconfigure::Client<ConfigType> client(prefix + "acquisition_" + std::to_string(i) + "/");
      ConfigType default_cfg;
      ASSERT_TRUE(client.getDefaultConfiguration(default_cfg, dr_get_max_wait_duration));
      ConfigType min_cfg;
      ASSERT_TRUE(client.getMinConfiguration(min_cfg, dr_get_max_wait_duration));
      ConfigType max_cfg;
      ASSERT_TRUE(client.getMaxConfiguration(max_cfg, dr_get_max_wait_duration));

      ASSERT_EQ(default_cfg.enabled, false);

      using Aperture = typename ZividSettingsType::Acquisition::Aperture;
      ASSERT_EQ(default_cfg.aperture, sdkDefaultValue<Aperture>());
      ASSERT_EQ(min_cfg.aperture, sdkValidRange<Aperture>().min());
      ASSERT_EQ(max_cfg.aperture, sdkValidRange<Aperture>().max());

      using Brightness = typename ZividSettingsType::Acquisition::Brightness;
      ASSERT_EQ(default_cfg.brightness, sdkDefaultValue<Brightness>());
      ASSERT_EQ(min_cfg.brightness, sdkValidRange<Brightness>().min());
      ASSERT_EQ(max_cfg.brightness, sdkValidRange<Brightness>().max());

      using ExposureTime = typename ZividSettingsType::Acquisition::ExposureTime;
      ASSERT_EQ(default_cfg.exposure_time, sdkDefaultValue<ExposureTime>().count());
      ASSERT_EQ(min_cfg.exposure_time, sdkValidRange<ExposureTime>().min().count());
      ASSERT_EQ(max_cfg.exposure_time, sdkValidRange<ExposureTime>().max().count());

      using Gain = typename ZividSettingsType::Acquisition::Gain;
      ASSERT_EQ(default_cfg.gain, sdkDefaultValue<Gain>());
      ASSERT_EQ(min_cfg.gain, sdkValidRange<Gain>().min());
      ASSERT_EQ(max_cfg.gain, sdkValidRange<Gain>().max());
    }
    ASSERT_FALSE(ros::service::waitForService(
        prefix + "acquisition_" + std::to_string(num_acquisition_servers) + "/set_parameters", short_wait_duration));
  }
};

TEST_F(DynamicReconfigureMinMaxDefaultTest, testDynamicReconfigureSettingsMinMaxDefaultValue)
{
  // Test the default, min and max configuration of the file camera used in the test suite. This
  // file camera is of model "Zivid One".
  waitForReady();

  testGeneralMinMaxDefault<Zivid::Settings, zivid_camera::SettingsConfig>("/zivid_camera/settings/");
  testAcquisitionMinMaxDefault<Zivid::Settings, zivid_camera::SettingsAcquisitionConfig>("/zivid_camera/settings/", 10);

  testGeneralMinMaxDefault<Zivid::Settings2D, zivid_camera::Settings2DConfig>("/zivid_camera/settings_2d/");
  testAcquisitionMinMaxDefault<Zivid::Settings2D, zivid_camera::Settings2DAcquisitionConfig>("/zivid_camera/"
                                                                                             "settings_2d/",
                                                                                             1);
}

class TestWithSettingsClients : public TestWithFileCamera
{
protected:
  TestWithSettingsClients()
    : settings_client_("/zivid_camera/settings"), settings_2d_client_("/zivid_camera/settings_2d")
  {
    settings_acquisition_clients_.reserve(num_settings_acquisition_dr_servers);
    for (std::size_t i = 0; i < num_settings_acquisition_dr_servers; i++)
    {
      using Client = dynamic_reconfigure::Client<zivid_camera::SettingsAcquisitionConfig>;
      settings_acquisition_clients_.emplace_back(
          std::make_unique<Client>("/zivid_camera/settings/acquisition_" + std::to_string(i)));
    }
    settings_2d_acquisition_clients_.reserve(num_settings_2d_acquisition_dr_servers);
    for (std::size_t i = 0; i < num_settings_2d_acquisition_dr_servers; i++)
    {
      using Client = dynamic_reconfigure::Client<zivid_camera::Settings2DAcquisitionConfig>;
      settings_2d_acquisition_clients_.emplace_back(
          std::make_unique<Client>("/zivid_camera/settings_2d/acquisition_" + std::to_string(i)));
    }
  }

  template <typename ZividSettingsType>
  auto settingsConfig()
  {
    if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings>)
    {
      zivid_camera::SettingsConfig cfg;
      EXPECT_TRUE(settings_client_.getCurrentConfiguration(cfg, dr_get_max_wait_duration));
      return cfg;
    }
    else if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings2D>)
    {
      zivid_camera::Settings2DConfig cfg;
      EXPECT_TRUE(settings_2d_client_.getCurrentConfiguration(cfg, dr_get_max_wait_duration));
      return cfg;
    }
    else
    {
      static_assert(DependentFalse<ZividSettingsType>::value, "Unsupported ZividSettingsType");
    }
  }

  template <typename ZividSettingsType>
  auto settingsAcquisitionConfig(std::size_t i) const
  {
    if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings>)
    {
      zivid_camera::SettingsAcquisitionConfig cfg;
      EXPECT_TRUE(settings_acquisition_clients_[i]->getCurrentConfiguration(cfg, dr_get_max_wait_duration));
      return cfg;
    }
    else if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings2D>)
    {
      zivid_camera::Settings2DAcquisitionConfig cfg;
      EXPECT_TRUE(settings_2d_acquisition_clients_[i]->getCurrentConfiguration(cfg, dr_get_max_wait_duration));
      return cfg;
    }
    else
    {
      static_assert(DependentFalse<ZividSettingsType>::value, "Unsupported ZividSettingsType");
    }
  }

  template <typename ZividSettingsType>
  auto numEnabledAcquisitions() const
  {
    std::size_t enabled_acquisitions = 0;
    for (std::size_t i = 0; i < maxAllowedAcquisitions<ZividSettingsType>(); i++)
    {
      if (settingsAcquisitionConfig<ZividSettingsType>(i).enabled)
      {
        enabled_acquisitions++;
      }
    }
    return enabled_acquisitions;
  }

  template <typename ZividSettingsType>
  std::size_t maxAllowedAcquisitions() const
  {
    if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings>)
    {
      return num_settings_acquisition_dr_servers;
    }
    else if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings2D>)
    {
      return num_settings_2d_acquisition_dr_servers;
    }
    else
    {
      static_assert(DependentFalse<ZividSettingsType>::value, "Unsupported ZividSettingsType");
    }
  }

  template <typename ZividSettingsType>
  void compareSettingsWithNodeState(const ZividSettingsType& settings)
  {
    compareSettingsConfigWithSettings(settings, settingsConfig<ZividSettingsType>());

    const auto& acquisitions = settings.acquisitions();
    ASSERT_EQ(acquisitions.size(), numEnabledAcquisitions<ZividSettingsType>());

    for (std::size_t i = 0; i < acquisitions.size(); i++)
    {
      compareSettingsAcquisitionConfigWithSettings(acquisitions[i], settingsAcquisitionConfig<ZividSettingsType>(i));
    }
    for (std::size_t i = acquisitions.size(); i < maxAllowedAcquisitions<ZividSettingsType>(); i++)
    {
      ASSERT_EQ(false, settingsAcquisitionConfig<ZividSettingsType>(i).enabled);
    }
  }

private:
  template <typename ZividSettingsAcquisitionType, typename CfgType>
  void compareSettingsAcquisitionConfigWithSettings(const ZividSettingsAcquisitionType& a, const CfgType& cfg) const
  {
    ASSERT_EQ(true, cfg.enabled);
    ASSERT_EQ(a.aperture().value(), cfg.aperture);
    ASSERT_EQ(a.brightness().value(), cfg.brightness);
    ASSERT_EQ(a.exposureTime().value().count(), cfg.exposure_time);
    ASSERT_EQ(a.gain().value(), cfg.gain);
  }

  void compareSettingsConfigWithSettings(const Zivid::Settings& s, const zivid_camera::SettingsConfig& cfg) const
  {
    const auto& color = s.processing().color();
    ASSERT_EQ(color.balance().blue().value(), cfg.processing_color_balance_blue);
    ASSERT_EQ(color.balance().green().value(), cfg.processing_color_balance_green);
    ASSERT_EQ(color.balance().red().value(), cfg.processing_color_balance_red);
    ASSERT_EQ(color.gamma().value(), cfg.processing_color_gamma);

    const auto& filters = s.processing().filters();
    ASSERT_EQ(filters.noise().removal().isEnabled().value(), cfg.processing_filters_noise_removal_enabled);
    ASSERT_EQ(filters.noise().removal().threshold().value(), cfg.processing_filters_noise_removal_threshold);
    ASSERT_EQ(filters.smoothing().gaussian().isEnabled().value(), cfg.processing_filters_smoothing_gaussian_enabled);
    ASSERT_EQ(filters.smoothing().gaussian().sigma().value(), cfg.processing_filters_smoothing_gaussian_sigma);
    ASSERT_EQ(filters.outlier().removal().isEnabled().value(), cfg.processing_filters_outlier_removal_enabled);
    ASSERT_EQ(filters.outlier().removal().threshold().value(), cfg.processing_filters_outlier_removal_threshold);
    ASSERT_EQ(filters.reflection().removal().isEnabled().value(), cfg.processing_filters_reflection_removal_enabled);
  }

  void compareSettingsConfigWithSettings(const Zivid::Settings2D& s, const zivid_camera::Settings2DConfig& cfg) const
  {
    const auto& color = s.processing().color();
    ASSERT_EQ(color.balance().blue().value(), cfg.processing_color_balance_blue);
    ASSERT_EQ(color.balance().green().value(), cfg.processing_color_balance_green);
    ASSERT_EQ(color.balance().red().value(), cfg.processing_color_balance_red);
    ASSERT_EQ(color.gamma().value(), cfg.processing_color_gamma);
  }

private:
  dynamic_reconfigure::Client<zivid_camera::SettingsConfig> settings_client_;
  std::vector<std::unique_ptr<dynamic_reconfigure::Client<zivid_camera::SettingsAcquisitionConfig>>>
      settings_acquisition_clients_;
  dynamic_reconfigure::Client<zivid_camera::Settings2DConfig> settings_2d_client_;
  std::vector<std::unique_ptr<dynamic_reconfigure::Client<zivid_camera::Settings2DAcquisitionConfig>>>
      settings_2d_acquisition_clients_;
};

class LoadSettingsTest : public TestWithSettingsClients
{
protected:
  template <typename CmdType, typename ZividSettingsType>
  void testLoadSettingsFromFile(const std::string& serviceName, const std::vector<std::string>& fileNames)
  {
    ASSERT_TRUE(ros::service::waitForService(serviceName, short_wait_duration));

    auto testLoadSettings = [&](const auto& fileName) {
      CmdType cmd;
      cmd.request.file_path = testDataDir() + "/settings/" + fileName;
      ASSERT_TRUE(ros::service::call(serviceName, cmd));
      short_wait_duration.sleep();

      auto expectedSettings = ZividSettingsType{ cmd.request.file_path };
      recursivelyFillInUnsetWithCameraDefault(expectedSettings, camera_.info());
      compareSettingsWithNodeState(expectedSettings);
    };

    for (const auto& fileName : fileNames)
    {
      testLoadSettings(fileName);
    }
  }

  template <typename CmdType>
  void testLoadInvalidSettingsGivesError(const std::string& serviceName)
  {
    ASSERT_TRUE(ros::service::waitForService(serviceName, short_wait_duration));
    CmdType cmd;
    cmd.request.file_path = testDataDir() + "/settings/invalid_file.yml";
    ASSERT_TRUE(boost::filesystem::exists(cmd.request.file_path));
    ASSERT_FALSE(ros::service::call(serviceName, cmd));
  }

  template <typename CmdType>
  void testLoadNonExistentFileGivesError(const std::string& serviceName)
  {
    ASSERT_TRUE(ros::service::waitForService(serviceName, short_wait_duration));
    CmdType cmd;
    cmd.request.file_path = "/tmp/foo/bar";
    ASSERT_FALSE(boost::filesystem::exists(cmd.request.file_path));
    ASSERT_FALSE(ros::service::call(serviceName, cmd));
  }

private:
  template <typename Node>
  static void recursivelyFillInUnsetWithCameraDefault(Node& node, const Zivid::CameraInfo& cameraInfo)
  {
    if constexpr (Node::nodeType == Zivid::DataModel::NodeType::group ||
                  Node::nodeType == Zivid::DataModel::NodeType::leafDataModelList)
    {
      node.forEach([&cameraInfo](auto& child) { recursivelyFillInUnsetWithCameraDefault(child, cameraInfo); });
    }
    else if (!node.hasValue())
    {
      static_assert(Node::nodeType == Zivid::DataModel::NodeType::leafValue);
      node = Zivid::Experimental::SettingsInfo::defaultValue<Node>(cameraInfo);
    }
  }
};

TEST_F(LoadSettingsTest, testLoadSettingsFromFile)
{
  testLoadSettingsFromFile<zivid_camera::LoadSettingsFromFile, Zivid::Settings>(
      load_settings_from_file_service_name,
      { "3d/single.yml", "3d/hdr.yml", "3d/hdr_with_not_set_values.yml", "3d/single.yml" });
}

TEST_F(LoadSettingsTest, testLoadSettings2DFromFile)
{
  testLoadSettingsFromFile<zivid_camera::LoadSettings2DFromFile, Zivid::Settings2D>(
      load_settings_2d_from_file_service_name,
      { "2d/single_1.yml", "2d/single_2.yml", "2d/single_with_not_set_values.yml", "2d/single_1.yml" });
}

TEST_F(LoadSettingsTest, testLoadSettingsFromInvalidFileGivesError)
{
  testLoadInvalidSettingsGivesError<zivid_camera::LoadSettingsFromFile>(load_settings_from_file_service_name);
}

TEST_F(LoadSettingsTest, testLoadSettings2DFromInvalidFileGivesError)
{
  testLoadInvalidSettingsGivesError<zivid_camera::LoadSettings2DFromFile>(load_settings_2d_from_file_service_name);
}

TEST_F(LoadSettingsTest, testLoadSettingsFromNonExistentFileGivesError)
{
  testLoadNonExistentFileGivesError<zivid_camera::LoadSettingsFromFile>(load_settings_from_file_service_name);
}

TEST_F(LoadSettingsTest, testLoadSettings2DFromNonExistentFileGivesError)
{
  testLoadNonExistentFileGivesError<zivid_camera::LoadSettings2DFromFile>(load_settings_2d_from_file_service_name);
}

class ZividCATest : public TestWithSettingsClients
{
protected:
  Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency toAPIAmbientLightFrequency(
      zivid_camera::CaptureAssistantSuggestSettings::Request::_ambient_light_frequency_type ambient_light_frequency)
  {
    using AmbientLightFrequency = Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency;
    using Request = zivid_camera::CaptureAssistantSuggestSettings::Request;
    switch (ambient_light_frequency)
    {
      case Request::AMBIENT_LIGHT_FREQUENCY_NONE:
        return AmbientLightFrequency::none;
      case Request::AMBIENT_LIGHT_FREQUENCY_50HZ:
        return AmbientLightFrequency::hz50;
      case Request::AMBIENT_LIGHT_FREQUENCY_60HZ:
        return AmbientLightFrequency::hz60;
    }
    throw std::runtime_error("Could not convert value " + std::to_string(ambient_light_frequency) + " to API enum.");
  }

  void performSuggestSettingsAndCompareWithCppAPI(
      ros::Duration max_capture_time,
      zivid_camera::CaptureAssistantSuggestSettings::Request::_ambient_light_frequency_type ambient_light_frequency)
  {
    zivid_camera::CaptureAssistantSuggestSettings srv;
    srv.request.max_capture_time = max_capture_time;
    srv.request.ambient_light_frequency = ambient_light_frequency;
    ASSERT_TRUE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
    short_wait_duration.sleep();

    Zivid::CaptureAssistant::SuggestSettingsParameters suggest_settings_parameters{
      Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{
          std::chrono::round<std::chrono::milliseconds>(SecondsD{ max_capture_time.toSec() }) },
      toAPIAmbientLightFrequency(ambient_light_frequency)
    };
    const auto api_settings = Zivid::CaptureAssistant::suggestSettings(camera_, suggest_settings_parameters);
    compareSettingsWithNodeState(api_settings);
  }
};

TEST_F(ZividCATest, testCaptureAssistantServiceAvailable)
{
  ASSERT_TRUE(ros::service::waitForService(capture_assistant_suggest_settings_service_name, short_wait_duration));
}

TEST_F(ZividCATest, testDifferentMaxCaptureTimeAndAmbientLightFrequency)
{
  using Request = zivid_camera::CaptureAssistantSuggestSettings::Request;
  for (double max_capture_time : { 0.2, 1.2, 10.0 })
  {
    for (auto ambient_light_frequency : { Request::AMBIENT_LIGHT_FREQUENCY_NONE, Request::AMBIENT_LIGHT_FREQUENCY_50HZ,
                                          Request::AMBIENT_LIGHT_FREQUENCY_60HZ })
    {
      performSuggestSettingsAndCompareWithCppAPI(ros::Duration{ max_capture_time }, ambient_light_frequency);
    }
  }
}

TEST_F(ZividCATest, testGoingFromMultipleAcquisitionsTo1Acquisition)
{
  using Request = zivid_camera::CaptureAssistantSuggestSettings::Request;
  performSuggestSettingsAndCompareWithCppAPI(ros::Duration{ 10.0 }, Request::AMBIENT_LIGHT_FREQUENCY_NONE);
  ASSERT_GT(numEnabledAcquisitions<Zivid::Settings>(), 1U);

  performSuggestSettingsAndCompareWithCppAPI(ros::Duration{ 0.2 }, Request::AMBIENT_LIGHT_FREQUENCY_NONE);
  ASSERT_EQ(numEnabledAcquisitions<Zivid::Settings>(), 1U);
}

TEST_F(ZividCATest, testCaptureAssistantWithInvalidMaxCaptureTimeFails)
{
  zivid_camera::CaptureAssistantSuggestSettings srv;
  srv.request.max_capture_time = ros::Duration{ 0.0 };
  ASSERT_FALSE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));

  const auto valid_range = Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime::validRange();
  const auto small_delta = std::chrono::milliseconds{ 1 };
  srv.request.max_capture_time = toRosDuration(valid_range.min() - small_delta);
  ASSERT_FALSE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
  srv.request.max_capture_time = toRosDuration(valid_range.max() + small_delta);
  ASSERT_FALSE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));

  srv.request.max_capture_time = toRosDuration(valid_range.max());
  ASSERT_TRUE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
}

TEST_F(ZividCATest, testCaptureAssistantDefaultAmbientLightFrequencyWorks)
{
  zivid_camera::CaptureAssistantSuggestSettings srv;
  srv.request.max_capture_time = ros::Duration{ 1.0 };
  ASSERT_TRUE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
}

TEST_F(ZividCATest, testCaptureAssistantInvalidAmbientLightFrequencyFails)
{
  zivid_camera::CaptureAssistantSuggestSettings srv;
  srv.request.max_capture_time = ros::Duration{ 1.0 };
  srv.request.ambient_light_frequency = 255;
  ASSERT_FALSE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));

  srv.request.ambient_light_frequency =
      zivid_camera::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_NONE;
  ASSERT_TRUE(ros::service::call(capture_assistant_suggest_settings_service_name, srv));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_zivid_camera");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
