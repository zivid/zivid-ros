#ifdef __clang__
#pragma clang diagnostic push
// Errors to ignore for this entire file
#pragma clang diagnostic ignored "-Wglobal-constructors"  // error triggered by gtest fixtures
#endif

#include <zivid_camera/CameraInfoSerialNumber.h>
#include <zivid_camera/CameraInfoModelName.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/CaptureAndSave.h>
#include <zivid_camera/Capture2D.h>
#include <zivid_camera/CaptureAssistantSuggestSettings.h>
#include <zivid_camera/LoadSettingsFromFile.h>
#include <zivid_camera/LoadSettings2DFromFile.h>
#include <zivid_camera/IsConnected.h>

#include <Zivid/Application.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Frame.h>
#include <Zivid/Camera.h>
#include <Zivid/Version.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "gtest_include_wrapper.h"

#include <boost/filesystem.hpp>

#include <ros/ros.h>

using SecondsD = std::chrono::duration<double>;

namespace
{
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

  const ros::Duration node_ready_wait_duration{ 45 };
  const ros::Duration short_wait_duration{ 0.5 };
  const ros::Duration medium_wait_duration{ 1.0 };
  static constexpr auto capture_service_name = "/zivid_camera/capture";
  static constexpr auto capture_and_save_service_name = "/zivid_camera/capture_and_save";
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
  static constexpr auto file_camera_path = "/usr/share/Zivid/data/FileCameraZivid2M70.zfc";

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

  template <typename ZividSettingsType>
  static std::string saveToTemporaryFile(const ZividSettingsType& settings)
  {
    const auto path = (boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("zivid_camera_test_"
                                                                                                 "%%%%%%%%.yml"))
                          .string();
    settings.save(path);
    return path;
  }

  void loadSettingsFromFile(const std::string& path)
  {
    zivid_camera::LoadSettingsFromFile load_settings;
    load_settings.request.file_path = path;
    ASSERT_TRUE(ros::service::call(load_settings_from_file_service_name, load_settings));
  }

  void loadSettings2DFromFile(const std::string& path)
  {
    zivid_camera::LoadSettings2DFromFile load_settings;
    load_settings.request.file_path = path;
    ASSERT_TRUE(ros::service::call(load_settings_2d_from_file_service_name, load_settings));
  }

  void loadSettings(const Zivid::Settings& settings)
  {
    const auto path = saveToTemporaryFile(settings);
    loadSettingsFromFile(path);
    boost::filesystem::remove(path);
  }

  void loadSettings2D(const Zivid::Settings2D& settings)
  {
    const auto path = saveToTemporaryFile(settings);
    loadSettings2DFromFile(path);
    boost::filesystem::remove(path);
  }

  void enableFirst3DAcquisition()
  {
    loadSettings(Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } });
  }

  void enableFirst2DAcquisition()
  {
    loadSettings2D(Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } });
  }

  void enableFirst3DAcquisitionAndCapture()
  {
    enableFirst3DAcquisition();
    medium_wait_duration.sleep();
    zivid_camera::Capture capture;
    ASSERT_TRUE(ros::service::call(capture_service_name, capture));
    short_wait_duration.sleep();
  }

  template <class Type>
  SubscriptionWrapper<Type> subscribe(const std::string& name)
  {
    return SubscriptionWrapper<Type>::make(nh_, name);
  }

  class AllCaptureTopicsSubscriber
  {
  public:
    AllCaptureTopicsSubscriber(ZividNodeTest& nodeTest)
      : color_camera_info_sub_(nodeTest.subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name))
      , color_image_color_sub_(nodeTest.subscribe<sensor_msgs::Image>(color_image_color_topic_name))
      , depth_camera_info_sub_(nodeTest.subscribe<sensor_msgs::CameraInfo>(depth_camera_info_topic_name))
      , depth_image_sub_(nodeTest.subscribe<sensor_msgs::Image>(depth_image_topic_name))
      , snr_camera_info_sub_(nodeTest.subscribe<sensor_msgs::CameraInfo>(snr_camera_info_topic_name))
      , snr_image_sub_(nodeTest.subscribe<sensor_msgs::Image>(snr_image_topic_name))
      , points_xyz_sub_(nodeTest.subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name))
      , points_xyzrgba_sub_(nodeTest.subscribe<sensor_msgs::PointCloud2>(points_xyzrgba_topic_name))
      , normals_xyz_sub_(nodeTest.subscribe<sensor_msgs::PointCloud2>(normals_xyz_topic_name))
    {
    }

    SubscriptionWrapper<sensor_msgs::CameraInfo> color_camera_info_sub_;
    SubscriptionWrapper<sensor_msgs::Image> color_image_color_sub_;
    SubscriptionWrapper<sensor_msgs::CameraInfo> depth_camera_info_sub_;
    SubscriptionWrapper<sensor_msgs::Image> depth_image_sub_;
    SubscriptionWrapper<sensor_msgs::CameraInfo> snr_camera_info_sub_;
    SubscriptionWrapper<sensor_msgs::Image> snr_image_sub_;
    SubscriptionWrapper<sensor_msgs::PointCloud2> points_xyz_sub_;
    SubscriptionWrapper<sensor_msgs::PointCloud2> points_xyzrgba_sub_;
    SubscriptionWrapper<sensor_msgs::PointCloud2> normals_xyz_sub_;

    void assert_num_topics_received(std::size_t numTopics)
    {
      ASSERT_EQ(color_camera_info_sub_.numMessages(), numTopics);
      ASSERT_EQ(color_image_color_sub_.numMessages(), numTopics);
      ASSERT_EQ(depth_camera_info_sub_.numMessages(), numTopics);
      ASSERT_EQ(depth_image_sub_.numMessages(), numTopics);
      ASSERT_EQ(snr_camera_info_sub_.numMessages(), numTopics);
      ASSERT_EQ(snr_image_sub_.numMessages(), numTopics);
      ASSERT_EQ(points_xyz_sub_.numMessages(), numTopics);
      ASSERT_EQ(points_xyzrgba_sub_.numMessages(), numTopics);
      ASSERT_EQ(normals_xyz_sub_.numMessages(), numTopics);
    }
  };

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
    ASSERT_EQ(ci.width, 1944U);
    ASSERT_EQ(ci.height, 1200U);
    ASSERT_EQ(ci.distortion_model, "plumb_bob");

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    assertArrayFloatEq(ci.K, std::array<double, 9>{ 1781.448, 0, 990.49268, 0, 1781.5297, 585.81781, 0, 0, 1 });

    // R = I
    assertArrayFloatEq(ci.R, std::array<double, 9>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 });

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    assertArrayFloatEq(ci.P,
                       std::array<double, 12>{ 1781.448, 0, 990.49268, 0, 0, 1781.5297, 585.81781, 0, 0, 0, 1, 0 });
  }
};

class TestWithFileCamera : public ZividNodeTest
{
protected:
  TestWithFileCamera() : camera_(zivid_.createFileCamera(file_camera_path))
  {
  }
  Zivid::Application zivid_;
  Zivid::Camera camera_;
};

TEST_F(ZividNodeTest, testWaitForNodeReady)
{
  // This test must be the first test that is run
  ASSERT_TRUE(ros::service::waitForService(capture_service_name, node_ready_wait_duration));
}

TEST_F(ZividNodeTest, testServiceCameraInfoModelName)
{
  zivid_camera::CameraInfoModelName model_name;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/model_name", model_name));
  ASSERT_EQ(model_name.response.model_name, "FileCamera");
}

TEST_F(ZividNodeTest, testServiceCameraInfoSerialNumber)
{
  zivid_camera::CameraInfoSerialNumber serial_number;
  ASSERT_TRUE(ros::service::call("/zivid_camera/camera_info/serial_number", serial_number));
  ASSERT_EQ(serial_number.response.serial_number, "F1");
}

TEST_F(ZividNodeTest, testServiceIsConnected)
{
  zivid_camera::IsConnected is_connected;
  ASSERT_TRUE(ros::service::call("/zivid_camera/is_connected", is_connected));
  ASSERT_EQ(is_connected.response.is_connected, true);
}

TEST_F(ZividNodeTest, testCapturePublishesTopics)
{
  AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);

  medium_wait_duration.sleep();
  all_capture_topics_subscriber.assert_num_topics_received(0);

  zivid_camera::Capture capture;
  // Capture fails when no acquisitions are enabled
  ASSERT_FALSE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  all_capture_topics_subscriber.assert_num_topics_received(0);

  enableFirst3DAcquisition();

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  all_capture_topics_subscriber.assert_num_topics_received(1);

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  all_capture_topics_subscriber.assert_num_topics_received(2);

  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  all_capture_topics_subscriber.assert_num_topics_received(3);

  short_wait_duration.sleep();
  all_capture_topics_subscriber.assert_num_topics_received(3);
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
  assertSensorMsgsPointCloud2Meta(*last_pc2, 1944U, 1200U, 16U);
  ASSERT_EQ(last_pc2->fields.size(), 4U);
  assertPointCloud2Field(last_pc2->fields[0], "x", 0, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(last_pc2->fields[1], "y", 4, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(last_pc2->fields[2], "z", 8, sensor_msgs::PointField::FLOAT32, 1);
  assertPointCloud2Field(last_pc2->fields[3], "rgba", 12, sensor_msgs::PointField::UINT32, 1);

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
  assertSensorMsgsPointCloud2Meta(*point_cloud, 1944U, 1200U,
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

TEST_F(CaptureOutputTest, testCapturePointsXYZWithROI)
{
  auto points_sub = subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name);
  const auto settings = Zivid::Settings{
    Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
    Zivid::Settings::RegionOfInterest::Box{
        Zivid::Settings::RegionOfInterest::Box::Enabled::yes,
        Zivid::Settings::RegionOfInterest::Box::PointO{ -370.5, -288, 886 },
        Zivid::Settings::RegionOfInterest::Box::PointA{ -354, 191.5, 673 },
        Zivid::Settings::RegionOfInterest::Box::PointB{ 420, -250, 876.5 },
        Zivid::Settings::RegionOfInterest::Box::Extents{ -2, 200.5 },
    },
  };
  loadSettings(settings);
  medium_wait_duration.sleep();

  zivid_camera::Capture capture;
  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();

  const auto& point_cloud = points_sub.lastMessage();
  ASSERT_TRUE(point_cloud.has_value());

  const auto point_cloud_sdk = captureViaSDK(settings);

  auto expected = point_cloud_sdk.copyData<Zivid::PointXYZ>();
  const auto numNanZ = [&] {
    size_t count = 0;
    for (size_t i = 0; i < expected.size(); ++i)
    {
      count += std::isnan(expected(i).z);
    }
    return count;
  }();
  // Verify that we have some number of points left (to verify that the ROI box did
  // not set everything to NaN)
  ASSERT_GT(numNanZ, 500000);
  ASSERT_LT(numNanZ, expected.size() - 500000);

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
}

TEST_F(CaptureOutputTest, testCapture3DColorImage)
{
  auto color_image_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);

  enableFirst3DAcquisitionAndCapture();
  const auto image = color_image_sub.lastMessage();
  ASSERT_TRUE(image.has_value());
  const std::size_t bytes_per_pixel = 4U;
  assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "rgba8");

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
  assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "32FC1");

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
  assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "32FC1");

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
  assertSensorMsgsPointCloud2Meta(*point_cloud, 1944U, 1200U,
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
    constexpr float delta = 0.15f;
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
  auto points_sub = subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name);
  medium_wait_duration.sleep();

  auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } };
  settings.set(Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes);
  settings.set(Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes);

  settings.set(Zivid::Settings::Engine::stripe);
  loadSettings(settings);
  zivid_camera::Capture capture;
  // Capture fails here because file camera does not support Stripe engine
  ASSERT_FALSE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  ASSERT_EQ(points_sub.numMessages(), 0U);

  settings.set(Zivid::Settings::Engine::phase);
  loadSettings(settings);
  ASSERT_TRUE(ros::service::call(capture_service_name, capture));
  short_wait_duration.sleep();
  ASSERT_EQ(points_sub.numMessages(), 1U);
}

TEST_F(ZividNodeTest, testCaptureCameraInfo)
{
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
    assertSensorMsgsImageMeta(img, 1944U, 1200U, 4U, "rgba8");
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

class CaptureWithSettings2DTest : public CaptureOutputTest
{
protected:
  CaptureWithSettings2DTest()
    : settings_(Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                Zivid::Settings::Color{
                    Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } })
  {
  }

  void TearDown() override
  {
    enableFirst3DAcquisition();
  }

  void loadSettingsWithColorAndCapture()
  {
    loadSettings(settings_);
    medium_wait_duration.sleep();
    zivid_camera::Capture capture;
    ASSERT_TRUE(ros::service::call(capture_service_name, capture));
    short_wait_duration.sleep();
  }

  Zivid::Settings settings_;
};

TEST_F(CaptureWithSettings2DTest, testCaptureColorImageFromSettings2D)
{
  auto color_camera_info_sub = subscribe<sensor_msgs::CameraInfo>(color_camera_info_topic_name);
  auto color_image_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);
  auto points_sub = subscribe<sensor_msgs::PointCloud2>(points_xyz_topic_name);

  loadSettingsWithColorAndCapture();

  ASSERT_EQ(color_image_sub.numMessages(), 1U);
  ASSERT_EQ(color_camera_info_sub.numMessages(), 1U);
  ASSERT_EQ(points_sub.numMessages(), 1U);

  const auto& image = *color_image_sub.lastMessage();
  const auto expected_frame2D = camera_.capture2D(settings_);
  assertSensorMsgsImageMeta(image, expected_frame2D.imageRGBA().width(), expected_frame2D.imageRGBA().height(), 4U,
                            "rgba8");
  assertSensorMsgsImageContents(image, expected_frame2D.imageRGBA());

  const auto& camera_info = *color_camera_info_sub.lastMessage();
  const auto expected_camera_matrix =
      Zivid::Experimental::Calibration::intrinsics(camera_, settings_.color().value()).cameraMatrix();
  ASSERT_FLOAT_EQ(camera_info.K[0], expected_camera_matrix.fx().value());
  ASSERT_FLOAT_EQ(camera_info.K[2], expected_camera_matrix.cx().value());
  ASSERT_FLOAT_EQ(camera_info.K[4], expected_camera_matrix.fy().value());
  ASSERT_FLOAT_EQ(camera_info.K[5], expected_camera_matrix.cy().value());

  const auto& points = *points_sub.lastMessage();
  ASSERT_EQ(image.header.stamp, points.header.stamp);
}

TEST_F(CaptureWithSettings2DTest, testCapturePointsXYZRGBAKeepsColorsFromSettings2D)
{
  auto color_image_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);
  auto points_sub = subscribe<sensor_msgs::PointCloud2>(points_xyzrgba_topic_name);

  loadSettingsWithColorAndCapture();

  ASSERT_EQ(color_image_sub.numMessages(), 1U);
  ASSERT_EQ(points_sub.numMessages(), 1U);

  const auto& points = *points_sub.lastMessage();
  const auto expected_colors = camera_.capture2D3D(settings_).pointCloud().copyColorsBGRA();
  ASSERT_EQ(points.width, expected_colors.width());
  ASSERT_EQ(points.height, expected_colors.height());
  for (std::size_t i = 0; i < expected_colors.size(); i++)
  {
    const uint32_t argb = *reinterpret_cast<const uint32_t*>(&points.data[i * points.point_step + 12]);
    const auto& expected = expected_colors(i);
    const auto expected_argb = static_cast<uint32_t>(expected.a << 24) | static_cast<uint32_t>(expected.r << 16) |
                               static_cast<uint32_t>(expected.g << 8) | static_cast<uint32_t>(expected.b);
    ASSERT_EQ(argb, expected_argb);
  }
}

TEST_F(CaptureWithSettings2DTest, testCaptureAndSaveKeepsColorsFromSettings2D)
{
  auto color_image_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);
  loadSettings(settings_);
  medium_wait_duration.sleep();

  const auto file_path = (boost::filesystem::temp_directory_path() / "zivid_camera_test_with_color.zdf").string();
  zivid_camera::CaptureAndSave capture_and_save;
  capture_and_save.request.file_path = file_path;
  ASSERT_TRUE(ros::service::call(capture_and_save_service_name, capture_and_save));
  short_wait_duration.sleep();

  ASSERT_EQ(color_image_sub.numMessages(), 1U);
  ASSERT_TRUE(Zivid::Frame{ file_path }.frame2D().has_value());
}

class CaptureAndSaveTest : public TestWithFileCamera
{
protected:
  void capture_and_save_to_path(const std::string& file_path, bool expect_success)
  {
    AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);
    medium_wait_duration.sleep();

    enableFirst3DAcquisition();
    zivid_camera::CaptureAndSave capture_and_save;
    capture_and_save.request.file_path = file_path;
    all_capture_topics_subscriber.assert_num_topics_received(0);
    if (expect_success)
    {
      ASSERT_TRUE(ros::service::call(capture_and_save_service_name, capture_and_save));
      ASSERT_TRUE(boost::filesystem::exists(file_path));
    }
    else
    {
      ASSERT_FALSE(ros::service::call(capture_and_save_service_name, capture_and_save));
      ASSERT_FALSE(boost::filesystem::exists(file_path));
    }
    medium_wait_duration.sleep();
    all_capture_topics_subscriber.assert_num_topics_received(1);
  }
};

TEST_F(CaptureAndSaveTest, testCaptureAndSaveNoPath)
{
  capture_and_save_to_path("", false);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSaveInvalidPath)
{
  capture_and_save_to_path("invalid_path", false);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSaveInvalidExtension)
{
  capture_and_save_to_path("/tmp/invalid_extension.wrong", false);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSaveZDF)
{
  capture_and_save_to_path("/tmp/valid.zdf", true);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSavePLY)
{
  capture_and_save_to_path("/tmp/valid.ply", true);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSavePCD)
{
  capture_and_save_to_path("/tmp/valid.pcd", true);
}

class TestWithNodeSettings : public TestWithFileCamera
{
protected:
  Zivid::Settings nodeSettings()
  {
    const auto path = (boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("zivid_camera_test_%%%"
                                                                                                 "%%%%%.zdf"))
                          .string();
    zivid_camera::CaptureAndSave capture_and_save;
    capture_and_save.request.file_path = path;
    EXPECT_TRUE(ros::service::call(capture_and_save_service_name, capture_and_save));
    const auto settings = Zivid::Frame{ path }.settings();
    boost::filesystem::remove(path);
    return settings;
  }

  std::size_t numNodeAcquisitions()
  {
    return nodeSettings().acquisitions().size();
  }

  void compareSettingsWithNodeState(const Zivid::Settings& expected)
  {
    const auto actual = nodeSettings();
    ASSERT_EQ(actual.acquisitions().size(), expected.acquisitions().size());
    for (std::size_t i = 0; i < expected.acquisitions().size(); i++)
    {
      const auto& a = actual.acquisitions().at(i);
      const auto& e = expected.acquisitions().at(i);
      ASSERT_EQ(a.aperture(), e.aperture());
      ASSERT_EQ(a.brightness(), e.brightness());
      ASSERT_EQ(a.exposureTime(), e.exposureTime());
      ASSERT_EQ(a.gain(), e.gain());
    }
    ASSERT_EQ(actual.processing().color().balance(), expected.processing().color().balance());
    ASSERT_EQ(actual.processing().color().gamma(), expected.processing().color().gamma());
    ASSERT_EQ(actual.processing().filters().noise().removal(), expected.processing().filters().noise().removal());
    ASSERT_EQ(actual.processing().filters().smoothing().gaussian(),
              expected.processing().filters().smoothing().gaussian());
    ASSERT_EQ(actual.processing().filters().outlier().removal(), expected.processing().filters().outlier().removal());
    ASSERT_EQ(actual.processing().filters().reflection().removal().isEnabled(),
              expected.processing().filters().reflection().removal().isEnabled());
  }

  void compareSettingsWithNodeState(const Zivid::Settings2D& expected)
  {
    auto color_image_sub = subscribe<sensor_msgs::Image>(color_image_color_topic_name);
    medium_wait_duration.sleep();
    zivid_camera::Capture2D capture_2d;
    ASSERT_TRUE(ros::service::call(capture_2d_service_name, capture_2d));
    short_wait_duration.sleep();
    ASSERT_EQ(color_image_sub.numMessages(), 1U);
    assertSensorMsgsImageContents(*color_image_sub.lastMessage(), camera_.capture(expected).imageRGBA());
  }
};

class LoadSettingsTest : public TestWithNodeSettings
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

class ZividCATest : public TestWithNodeSettings
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
  ASSERT_GT(numNodeAcquisitions(), 1U);

  performSuggestSettingsAndCompareWithCppAPI(ros::Duration{ 0.2 }, Request::AMBIENT_LIGHT_FREQUENCY_NONE);
  ASSERT_EQ(numNodeAcquisitions(), 1U);
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
