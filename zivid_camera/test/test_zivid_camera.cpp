// Copyright 2024 Zivid AS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Zivid AS nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Frame.h>
#include <Zivid/Version.h>
#include <gtest/gtest.h>

#include <cstdio>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <test_zivid_camera.hpp>
#include <zivid_camera/zivid_camera.hpp>
#include <zivid_interfaces/srv/camera_info_model_name.hpp>
#include <zivid_interfaces/srv/camera_info_serial_number.hpp>
#include <zivid_interfaces/srv/capture_and_save.hpp>
#include <zivid_interfaces/srv/capture_assistant_suggest_settings.hpp>
#include <zivid_interfaces/srv/is_connected.hpp>
#include <zivid_interfaces/srv/projection_resolution.hpp>
#include <zivid_interfaces/srv/projection_start.hpp>
#include <zivid_interfaces/srv/projection_status.hpp>

#ifdef _WIN32
#define ZIVID_SAMPLE_DATA_DIR "C:\\ProgramData\\Zivid\\"
#else
#define ZIVID_SAMPLE_DATA_DIR "/usr/share/Zivid/data/"
#endif

namespace
{
constexpr auto file_camera_default_path = ZIVID_SAMPLE_DATA_DIR "FileCameraZivid2M70.zfc";
constexpr auto file_camera_calibration_board_path =
  ZIVID_SAMPLE_DATA_DIR "BinWithCalibrationBoard.zfc";
}  // namespace

std::shared_ptr<zivid_camera::ZividCamera> ZividCameraNodeWrapper::getOrConstruct(
  FileCameraMode fileCamera, NodeReusePolicy reusePolicy)
{
  if (
    !m_fileCameraMode.has_value() || reusePolicy == NodeReusePolicy::RestartNode ||
    m_fileCameraMode.value() != fileCamera) {
    std::string fileCameraPath;
    switch (fileCamera) {
      case FileCameraMode::Default:
        fileCameraPath = file_camera_default_path;
        break;
      case FileCameraMode::CalibrationBoard:
        fileCameraPath = file_camera_calibration_board_path;
        break;
      default:
        throw std::runtime_error("Unexpected file camera mode");
    }
    auto node_options =
      rclcpp::NodeOptions{}.append_parameter_override("file_camera_path", fileCameraPath);
    m_zividRosNode.reset();
    m_fileCameraMode = fileCamera;
    m_zividRosNode = std::make_shared<zivid_camera::ZividCamera>(node_options);
  }
  return m_zividRosNode;
}

std::shared_ptr<zivid_camera::ZividCamera> ZividCameraNodeWrapper::get()
{
  if (m_zividRosNode == nullptr) {
    throw std::runtime_error("File camera not constructed");
  }
  return m_zividRosNode;
}

void ZividCameraNodeWrapper::reset()
{
  m_fileCameraMode.reset();
  m_zividRosNode.reset();
}

TestWithFileCamera::TestWithFileCamera()
: camera_(
    ZividCameraNodeWrapper::get()->zividApplication().createFileCamera(file_camera_default_path))
{
}

std::optional<FileCameraMode> ZividCameraNodeWrapper::m_fileCameraMode;
std::shared_ptr<zivid_camera::ZividCamera> ZividCameraNodeWrapper::m_zividRosNode;

TEST_F(ZividNodeTest, testCaptureServiceReady)
{
  auto client = test_node_->create_client<std_srvs::srv::Trigger>(capture_service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds{45}));
}

TEST_F(ZividNodeTest, testServiceCameraInfoModelName)
{
  auto model_name_request =
    doEmptySrvRequest<zivid_interfaces::srv::CameraInfoModelName>("camera_info/model_name");
#if (ZIVID_CORE_VERSION_MAJOR == 2 && ZIVID_CORE_VERSION_MINOR <= 13)
  ASSERT_EQ(model_name_request->model_name, std::string("FileCamera-") + ZIVID_CORE_VERSION);
#else
  ASSERT_EQ(model_name_request->model_name, std::string("FileCamera"));
#endif
}

TEST_F(ZividNodeTest, testServiceCameraInfoSerialNumber)
{
  auto response =
    doEmptySrvRequest<zivid_interfaces::srv::CameraInfoSerialNumber>("camera_info/serial_number");
  int file_camera_number = -1;
  int characters_consumed = -1;
  const int num_assigned_arguments = std::sscanf(
    response->serial_number.c_str(), "F%d%n", &file_camera_number, &characters_consumed);
  ASSERT_EQ(num_assigned_arguments, 1);
  ASSERT_EQ(static_cast<size_t>(characters_consumed), response->serial_number.size());
  // The exact value depends on how many file cameras have been loaded so far in this process, which
  // depends on the order and number of tests run.
  ASSERT_GE(file_camera_number, 0);
  ASSERT_LE(file_camera_number, 100);
}

TEST_F(ZividNodeTest, testServiceIsConnected)
{
  auto is_connected_request = doEmptySrvRequest<zivid_interfaces::srv::IsConnected>("is_connected");
  ASSERT_EQ(is_connected_request->is_connected, true);
}

TEST_F(ZividNodeTest, testCaptureConfigurationErrorIfBothPathAndYmlSet)
{
  auto run_test = [&](
                    const auto & service_name, const std::string & path_param,
                    const std::string & yml_param, const auto & yml_content) {
    auto color_image_sub = subscribe<sensor_msgs::msg::Image>(color_image_color_topic_name);
    auto assert_num_topics_received = [&](auto num_topics) {
      ASSERT_EQ(color_image_sub.numMessages(), num_topics);
    };

    const auto expectedErrorBothEmpty =
      "Both '" + path_param + "' and '" + yml_param +
      "' parameters are empty! Please set one of these parameters.";
    const auto expectedErrorBothSet =
      "Both '" + path_param + "' and '" + yml_param +
      "' parameters are non-empty! Please set only one of these parameters.";

    setNodeParameter(path_param, "");
    setNodeParameter(yml_param, "");
    verifyTriggerResponseError(doStdSrvsTriggerRequest(service_name), expectedErrorBothEmpty);
    executor_.spin_some();
    assert_num_topics_received(0);

    auto tmp_file = TmpFile("settings.yml", yml_content);
    setNodeParameter(path_param, tmp_file.string());
    setNodeParameter(yml_param, yml_content);
    verifyTriggerResponseError(doStdSrvsTriggerRequest(service_name), expectedErrorBothSet);
    assert_num_topics_received(0);

    setNodeParameter(path_param, "");
    verifyTriggerResponseSuccess(doStdSrvsTriggerRequest(service_name));
    executor_.spin_some();
    assert_num_topics_received(1);

    setNodeParameter(path_param, tmp_file.string());
    setNodeParameter(yml_param, "");
    verifyTriggerResponseSuccess(doStdSrvsTriggerRequest(service_name));
    executor_.spin_some();
    assert_num_topics_received(2);

    setNodeParameter(path_param, tmp_file.string());
    setNodeParameter(yml_param, yml_content);
    verifyTriggerResponseError(doStdSrvsTriggerRequest(service_name), expectedErrorBothSet);
    executor_.spin_some();
    assert_num_topics_received(2);

    setNodeParameter(path_param, "");
    setNodeParameter(yml_param, "");
    verifyTriggerResponseError(doStdSrvsTriggerRequest(service_name), expectedErrorBothEmpty);
    executor_.spin_some();
    assert_num_topics_received(2);
    // Leave both path and yml params empty, for the next test
  };

  run_test(
    capture_service_name, parameter_settings_file_path, parameter_settings_yaml,
    defaultSingleAcquisitionSettingsYml());
  run_test(
    capture_2d_service_name, parameter_settings_2d_file_path, parameter_settings_2d_yaml,
    defaultSingleAcquisitionSettings2DYml());
}

TEST_F(TestWithFileCamera, testCaptureExceptionsReturnErrors)
{
  AllCaptureTopicsSubscriber topics_subscriber(*this);

  const auto settingsYmlInvalid =
    R"(
__version__:
  serializer: 1
  data: 17
Settings:
  Acquisitions:
    - Acquisition:
  Experimental:
    Engine: stripe
  Processing:
    Filters:
      Experimental:
        ContrastDistortion:
          Correction:
            Enabled: yes
      Reflection:
        Removal:
          Enabled: yes
)";

  // This capture throws because Stripe engine is unsupported on this file camera.
  verifyTriggerResponseError(
    doCaptureUsingFilePath(settingsYmlInvalid),
    "This file camera can only be used with the phase engine.");
  verifyTriggerResponseError(
    doCaptureUsingYmlString(settingsYmlInvalid),
    "This file camera can only be used with the phase engine.");
  topics_subscriber.assert_num_topics_received(0U);

  const auto settingsYmlPhase =
    R"(
__version__:
  serializer: 1
  data: 17
Settings:
  Acquisitions:
    - Acquisition:
  Experimental:
    Engine: phase
)";

  verifyTriggerResponseSuccess(doCaptureUsingFilePath(settingsYmlPhase));
  topics_subscriber.assert_num_topics_received(1U);
  verifyTriggerResponseSuccess(doCaptureUsingYmlString(settingsYmlPhase));
  topics_subscriber.assert_num_topics_received(2U);
}

TEST_F(TestWithFileCamera, testCaptureInvalidColorSpaceErrors)
{
  AllCaptureTopicsSubscriber topics_subscriber(*this);

  setNodeParameter(parameter_color_space, "invalid_color_space");

  verifyTriggerResponseError(
    doSingleDefaultAcquisitionCaptureUsingFilePath(),
    "Invalid value for parameter 'color_space': 'invalid_color_space'. Expected one of: "
    "linear_rgb, srgb.");
  topics_subscriber.assert_num_topics_received(0U);

  setNodeParameter(parameter_color_space, "srgb");

  verifyTriggerResponseSuccess(doSingleDefaultAcquisitionCaptureUsingFilePath());
  topics_subscriber.assert_num_topics_received(1U);
}

TEST_F(TestWithFileCamera, testCapture2DInvalidColorSpaceErrors)
{
  AllCapture2DTopicsSubscriber topics_subscriber(*this);

  setNodeParameter(parameter_color_space, "invalid_color_space");

  verifyTriggerResponseError(
    doSingleDefaultAcquisitionCapture2DUsingFilePath(),
    "Invalid value for parameter 'color_space': 'invalid_color_space'. Expected one of: "
    "linear_rgb, srgb.");
  topics_subscriber.assert_num_topics_received(0U);

  setNodeParameter(parameter_color_space, "srgb");

  verifyTriggerResponseSuccess(doSingleDefaultAcquisitionCapture2DUsingFilePath());
  topics_subscriber.assert_num_topics_received(1U);
}

TEST_F(TestWithFileCamera, testCapture2DExceptionsReturnErrors)
{
  AllCapture2DTopicsSubscriber topics_subscriber(*this);

  const auto settings2DYmlInvalid =
    R"(
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
)";

  // This capture throws because acquisitions list is empty
  verifyTriggerResponseError(doCapture2DUsingFilePath(settings2DYmlInvalid));
  verifyTriggerResponseError(doCapture2DUsingYmlString(settings2DYmlInvalid));
  topics_subscriber.assert_num_topics_received(0U);

  const auto settings2DYmlValid =
    R"(
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
    - Acquisition:
)";

  verifyTriggerResponseSuccess(doCapture2DUsingFilePath(settings2DYmlValid));
  topics_subscriber.assert_num_topics_received(1U);
  verifyTriggerResponseSuccess(doCapture2DUsingYmlString(settings2DYmlValid));
  topics_subscriber.assert_num_topics_received(2U);
}

TEST_F(ZividNodeTest, testRepeatedCapturePublishesTopics)
{
  AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);
  all_capture_topics_subscriber.assert_num_topics_received(0);

  doSingleDefaultAcquisitionCaptureUsingFilePath();
  all_capture_topics_subscriber.assert_num_topics_received(1);

  doSingleDefaultAcquisitionCaptureUsingFilePath();
  all_capture_topics_subscriber.assert_num_topics_received(2);

  doSingleDefaultAcquisitionCaptureUsingFilePath();
  all_capture_topics_subscriber.assert_num_topics_received(3);

  rclcpp::sleep_for(short_wait_duration);
  all_capture_topics_subscriber.assert_num_topics_received(3);
}

TEST_F(ZividNodeTest, testRepeatedCapture2DPublishesTopics)
{
  AllCapture2DTopicsSubscriber all_capture_2d_topics_subscriber(*this);
  all_capture_2d_topics_subscriber.assert_num_topics_received(0);

  doSingleDefaultAcquisitionCapture2DUsingFilePath();
  all_capture_2d_topics_subscriber.assert_num_topics_received(1);

  doSingleDefaultAcquisitionCapture2DUsingFilePath();
  all_capture_2d_topics_subscriber.assert_num_topics_received(2);

  doSingleDefaultAcquisitionCapture2DUsingFilePath();
  all_capture_2d_topics_subscriber.assert_num_topics_received(3);

  rclcpp::sleep_for(short_wait_duration);
  all_capture_2d_topics_subscriber.assert_num_topics_received(3);
}

class CaptureOutputTest : public TestWithFileCamera
{
protected:
  Zivid::PointCloud captureViaSDKDefaultSettings()
  {
    return camera_
      .capture(Zivid::Settings{Zivid::Settings::Acquisitions{Zivid::Settings::Acquisition{}}})
      .pointCloud();
  }

  Zivid::PointCloud captureViaSDK(const Zivid::Settings & settings)
  {
    return camera_.capture(settings).pointCloud();
  }

  void compareFloat(float a, float b, float delta = 1e-6f) const
  {
    if (std::isnan(a)) {
      ASSERT_TRUE(std::isnan(b));
    } else {
      ASSERT_NEAR(a, b, delta);
    }
  }

  void comparePointCoordinate(float ros_coordinate, float sdk_coordinate) const
  {
    if (std::isnan(ros_coordinate)) {
      ASSERT_TRUE(std::isnan(sdk_coordinate));
    } else {
      // Output from the SDK is millimeters. In the ROS driver a transform is applied to convert
      // the ROS points to meters.
      const float delta = 0.000001f;
      ASSERT_NEAR(ros_coordinate, sdk_coordinate / 1000, delta);
    }
  }

  void assertPointCloud2MetaAndFields(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc2)
  {
    ASSERT_TRUE(pc2);
    assertSensorMsgsPointCloud2Meta(*pc2, 1944U, 1200U, 16U);
    ASSERT_EQ(pc2->fields.size(), 4U);
    assertPointCloud2Field(pc2->fields[0], "x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
    assertPointCloud2Field(pc2->fields[1], "y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
    assertPointCloud2Field(pc2->fields[2], "z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
    assertPointCloud2Field(pc2->fields[3], "rgba", 12, sensor_msgs::msg::PointField::UINT32, 1);
  }

  template <typename ZividDataType>
  void comparePointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc2,
    const Zivid::PointCloud & point_cloud)
  {
    static_assert(
      std::is_same_v<ZividDataType, Zivid::PointXYZColorRGBA> ||
      std::is_same_v<ZividDataType, Zivid::PointXYZColorRGBA_SRGB>);
    const auto expected_xyzrgba = point_cloud.copyData<ZividDataType>();
    ASSERT_EQ(pc2->width, expected_xyzrgba.width());
    ASSERT_EQ(pc2->height, expected_xyzrgba.height());
    for (std::size_t i = 0; i < expected_xyzrgba.size(); i++) {
      const uint8_t * point_ptr = &pc2->data[i * pc2->point_step];
      const float x = *reinterpret_cast<const float *>(&point_ptr[0]);
      const float y = *reinterpret_cast<const float *>(&point_ptr[4]);
      const float z = *reinterpret_cast<const float *>(&point_ptr[8]);
      const uint32_t argb = *reinterpret_cast<const uint32_t *>(&point_ptr[12]);
      const auto & expected = expected_xyzrgba(i);

      comparePointCoordinate(x, expected.point.x);
      comparePointCoordinate(y, expected.point.y);
      comparePointCoordinate(z, expected.point.z);
      const auto expected_argb = static_cast<uint32_t>(expected.color.a << 24) |
                                 static_cast<uint32_t>(expected.color.r << 16) |
                                 static_cast<uint32_t>(expected.color.g << 8) |
                                 static_cast<uint32_t>(expected.color.b);
      ASSERT_EQ(argb, expected_argb);
    }
  }
};

TEST_F(CaptureOutputTest, testCapturePointsXYZGBA_SRGB)
{
  auto points_sub = subscribe<sensor_msgs::msg::PointCloud2>(points_xyzrgba_topic_name);

  doSingleDefaultAcquisitionCaptureUsingFilePath();

  const auto last_pc2 = points_sub.lastMessage();
  assertPointCloud2MetaAndFields(last_pc2);

  const auto point_cloud = captureViaSDKDefaultSettings();
  comparePointCloud<Zivid::PointXYZColorRGBA_SRGB>(last_pc2, point_cloud);
}

TEST_F(CaptureOutputTest, testCapturePointsXYZGBA_LinearRGB)
{
  auto points_sub = subscribe<sensor_msgs::msg::PointCloud2>(points_xyzrgba_topic_name);

  setNodeColorSpaceLinearRGB();
  doSingleDefaultAcquisitionCaptureUsingFilePath();

  const auto last_pc2 = points_sub.lastMessage();
  assertPointCloud2MetaAndFields(last_pc2);

  const auto point_cloud = captureViaSDKDefaultSettings();
  comparePointCloud<Zivid::PointXYZColorRGBA>(last_pc2, point_cloud);
}

TEST_F(CaptureOutputTest, testCapturePointsXYZ)
{
  auto points_sub = subscribe<sensor_msgs::msg::PointCloud2>(points_xyz_topic_name);

  doSingleDefaultAcquisitionCaptureUsingFilePath();

  const auto & point_cloud = points_sub.lastMessage();
  ASSERT_TRUE(point_cloud);
  assertSensorMsgsPointCloud2Meta(
    *point_cloud, 1944U, 1200U,
    16U);  // 3x4 bytes for xyz + 4 bytes padding (w) = 16 bytes total
  ASSERT_EQ(point_cloud->fields.size(), 3U);
  assertPointCloud2Field(point_cloud->fields[0], "x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
  assertPointCloud2Field(point_cloud->fields[1], "y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
  assertPointCloud2Field(point_cloud->fields[2], "z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);

  auto point_cloud_sdk = captureViaSDKDefaultSettings();
  auto expected_xyz = point_cloud_sdk.copyData<Zivid::PointXYZ>();
  ASSERT_EQ(point_cloud->width, expected_xyz.width());
  ASSERT_EQ(point_cloud->height, expected_xyz.height());
  for (std::size_t i = 0; i < expected_xyz.size(); i++) {
    const uint8_t * point_ptr = &point_cloud->data[i * point_cloud->point_step];
    const float x = *reinterpret_cast<const float *>(&point_ptr[0]);
    const float y = *reinterpret_cast<const float *>(&point_ptr[4]);
    const float z = *reinterpret_cast<const float *>(&point_ptr[8]);

    const auto expected = expected_xyz(i);
    comparePointCoordinate(x, expected.x);
    comparePointCoordinate(y, expected.y);
    comparePointCoordinate(z, expected.z);
  }
}

TEST_F(CaptureOutputTest, testCapturePointsXYZWithROI)
{
  auto points_sub = subscribe<sensor_msgs::msg::PointCloud2>(points_xyz_topic_name);

  const auto settings_yaml =
    R"(
__version__:
  serializer: 1
  data: 17
Settings:
  Acquisitions:
    - Acquisition:
        Aperture: __not_set__
        Brightness: __not_set__
        ExposureTime: __not_set__
        Gain: __not_set__
  RegionOfInterest:
    Box:
      Enabled: yes
      Extents: [-2, 200.5]
      PointA: [-354, 191.5, 673]
      PointB: [420, -250, 876.5]
      PointO: [-370.5, -288, 886]
)";

  doCaptureUsingFilePath(settings_yaml);
  const auto & point_cloud = points_sub.lastMessage();
  ASSERT_TRUE(point_cloud);

  const auto point_cloud_sdk = captureViaSDK(Zivid::Settings{
    Zivid::Settings::Acquisitions{Zivid::Settings::Acquisition{}},
    Zivid::Settings::RegionOfInterest::Box{
      Zivid::Settings::RegionOfInterest::Box::Enabled::yes,
      Zivid::Settings::RegionOfInterest::Box::PointO{-370.5, -288, 886},
      Zivid::Settings::RegionOfInterest::Box::PointA{-354, 191.5, 673},
      Zivid::Settings::RegionOfInterest::Box::PointB{420, -250, 876.5},
      Zivid::Settings::RegionOfInterest::Box::Extents{-2, 200.5},
    },
  });

  auto expected = point_cloud_sdk.copyData<Zivid::PointXYZ>();
  const auto num_z_nan = [&] {
    size_t count = 0;
    for (size_t i = 0; i < expected.size(); ++i) {
      count += std::isnan(expected(i).z);
    }
    return count;
  }();
  // Verify that we have some number of points left (to verify that the ROI box did
  // not set everything to NaN)
  ASSERT_GT(num_z_nan, 500000);
  ASSERT_LT(num_z_nan, expected.size() - 500000);

  for (size_t i = 0; i < expected.size(); ++i) {
    const uint8_t * point_ptr = &point_cloud->data[i * point_cloud->point_step];
    const float x = *reinterpret_cast<const float *>(&point_ptr[0]);
    const float y = *reinterpret_cast<const float *>(&point_ptr[4]);
    const float z = *reinterpret_cast<const float *>(&point_ptr[8]);
    comparePointCoordinate(x, expected(i).x);
    comparePointCoordinate(y, expected(i).y);
    comparePointCoordinate(z, expected(i).z);
  }
}

TEST_F(CaptureOutputTest, testCapture3DColorImageSRGB)
{
  auto color_image_sub = subscribe<sensor_msgs::msg::Image>(color_image_color_topic_name);

  doSingleDefaultAcquisitionCaptureUsingFilePath();
  const auto image = color_image_sub.lastMessage();
  ASSERT_TRUE(image);
  const std::size_t bytes_per_pixel = 4U;
  assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "rgba8");

  const auto point_cloud = captureViaSDKDefaultSettings();
  const auto expected_rgba = point_cloud.copyData<Zivid::ColorRGBA_SRGB>();
  assertSensorMsgsImageContents(*image, expected_rgba);
}

TEST_F(CaptureOutputTest, testCapture3DColorImageLinearRGB)
{
  setNodeColorSpaceLinearRGB();
  auto color_image_sub = subscribe<sensor_msgs::msg::Image>(color_image_color_topic_name);

  doSingleDefaultAcquisitionCaptureUsingFilePath();
  const auto image = color_image_sub.lastMessage();
  ASSERT_TRUE(image);
  const std::size_t bytes_per_pixel = 4U;
  assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "rgba8");

  const auto point_cloud = captureViaSDKDefaultSettings();
  const auto expected_rgba = point_cloud.copyData<Zivid::ColorRGBA>();
  assertSensorMsgsImageContents(*image, expected_rgba);
}

TEST_F(CaptureOutputTest, testCaptureDepthImage)
{
  auto depth_image_sub = subscribe<sensor_msgs::msg::Image>(depth_image_topic_name);

  doSingleDefaultAcquisitionCaptureUsingFilePath();

  const auto image = depth_image_sub.lastMessage();
  ASSERT_TRUE(image);
  const std::size_t bytes_per_pixel = 4U;
  assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "32FC1");

  const auto point_cloud = captureViaSDKDefaultSettings();
  const auto expected_z = point_cloud.copyData<Zivid::PointZ>();
  ASSERT_EQ(image->width, expected_z.width());
  ASSERT_EQ(image->height, expected_z.height());
  for (std::size_t i = 0; i < expected_z.size(); i++) {
    const auto expected = expected_z(i);
    const float z = *reinterpret_cast<const float *>(image->data.data() + i * bytes_per_pixel);
    comparePointCoordinate(z, expected.z);
  }
}

TEST_F(CaptureOutputTest, testCaptureSNRImage)
{
  auto snr_image_sub = subscribe<sensor_msgs::msg::Image>(snr_image_topic_name);

  doSingleDefaultAcquisitionCaptureUsingFilePath();

  const auto image = snr_image_sub.lastMessage();
  ASSERT_TRUE(image);
  const std::size_t bytes_per_pixel = 4U;
  assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "32FC1");

  const auto point_cloud = captureViaSDKDefaultSettings();
  const auto expected_snr = point_cloud.copyData<Zivid::SNR>();
  ASSERT_EQ(image->width, expected_snr.width());
  ASSERT_EQ(image->height, expected_snr.height());
  for (std::size_t i = 0; i < expected_snr.size(); i++) {
    const auto expected = expected_snr(i);
    const float snr = *reinterpret_cast<const float *>(image->data.data() + i * bytes_per_pixel);
    ASSERT_EQ(snr, expected.value);
  }
}

TEST_F(CaptureOutputTest, testCaptureNormals)
{
  auto normals_sub = subscribe<sensor_msgs::msg::PointCloud2>(normals_xyz_topic_name);

  doSingleDefaultAcquisitionCaptureUsingFilePath();

  const auto & point_cloud = normals_sub.lastMessage();
  ASSERT_TRUE(point_cloud);
  assertSensorMsgsPointCloud2Meta(
    *point_cloud, 1944U, 1200U,
    3U * sizeof(float));  // 12 bytes total
  ASSERT_EQ(point_cloud->fields.size(), 3U);
  assertPointCloud2Field(
    point_cloud->fields[0], "normal_x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
  assertPointCloud2Field(
    point_cloud->fields[1], "normal_y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
  assertPointCloud2Field(
    point_cloud->fields[2], "normal_z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);

  auto point_cloud_sdk = captureViaSDKDefaultSettings();
  auto expected_normal_xyz_before_transform = point_cloud_sdk.copyData<Zivid::NormalXYZ>();
  ASSERT_EQ(point_cloud->width, expected_normal_xyz_before_transform.width());
  ASSERT_EQ(point_cloud->height, expected_normal_xyz_before_transform.height());
  ASSERT_EQ(point_cloud->width * point_cloud->height, expected_normal_xyz_before_transform.size());

  // Transform from mm to m (like is done internally in Zivid driver)
  const float scale = 0.001f;
  point_cloud_sdk.transform(
    Zivid::Matrix4x4{scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, 1});
  auto expected_normal_xyz_after_transform = point_cloud_sdk.copyData<Zivid::NormalXYZ>();
  ASSERT_EQ(
    expected_normal_xyz_after_transform.size(), expected_normal_xyz_before_transform.size());

  for (std::size_t i = 0; i < expected_normal_xyz_before_transform.size(); i++) {
    const uint8_t * cloud_ptr = &point_cloud->data[i * point_cloud->point_step];
    const float normal_x = *reinterpret_cast<const float *>(&cloud_ptr[0]);
    const float normal_y = *reinterpret_cast<const float *>(&cloud_ptr[4]);
    const float normal_z = *reinterpret_cast<const float *>(&cloud_ptr[8]);

    const auto & expected_sdk_before_transform = expected_normal_xyz_before_transform(i);
    // We do a transform in the ROS driver to scale from mm to meters. However,
    // `expected_normal_xyz` are calculated without transform, so we need a slightly higher
    // delta to compare.
    constexpr float delta = 0.1f;
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_x, expected_sdk_before_transform.x, delta));
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_y, expected_sdk_before_transform.y, delta));
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_z, expected_sdk_before_transform.z, delta));

    const auto & expected_sdk_after_transform = expected_normal_xyz_after_transform(i);
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_x, expected_sdk_after_transform.x));
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_y, expected_sdk_after_transform.y));
    ASSERT_NO_FATAL_FAILURE(compareFloat(normal_z, expected_sdk_after_transform.z));
  }
}

TEST_F(ZividNodeTest, testCaptureCameraInfo)
{
  auto color_camera_info_sub =
    subscribe<sensor_msgs::msg::CameraInfo>(color_camera_info_topic_name);
  auto depth_camera_info_sub =
    subscribe<sensor_msgs::msg::CameraInfo>(depth_camera_info_topic_name);
  auto snr_camera_info_sub = subscribe<sensor_msgs::msg::CameraInfo>(snr_camera_info_topic_name);

  doSingleDefaultAcquisitionCaptureUsingFilePath();

  ASSERT_EQ(color_camera_info_sub.numMessages(), 1U);
  ASSERT_EQ(depth_camera_info_sub.numMessages(), 1U);
  ASSERT_EQ(snr_camera_info_sub.numMessages(), 1U);

  assertCameraInfoForFileCamera(*color_camera_info_sub.lastMessage());
  assertCameraInfoForFileCamera(*depth_camera_info_sub.lastMessage());
  assertCameraInfoForFileCamera(*snr_camera_info_sub.lastMessage());

  setNodeParameter(parameter_intrinsics_source, "frame");
  doSingleDefaultAcquisitionCaptureUsingFilePath();

  ASSERT_EQ(color_camera_info_sub.numMessages(), 2U);
  ASSERT_EQ(depth_camera_info_sub.numMessages(), 2U);
  ASSERT_EQ(snr_camera_info_sub.numMessages(), 2U);

  assertCameraInfoForFileCameraWithIntrinsicsSourceFrame(*color_camera_info_sub.lastMessage());
  assertCameraInfoForFileCameraWithIntrinsicsSourceFrame(*depth_camera_info_sub.lastMessage());
  assertCameraInfoForFileCameraWithIntrinsicsSourceFrame(*snr_camera_info_sub.lastMessage());
}

TEST_F(ZividNodeTest, testCaptureInvalidIntrinsicsSourceCaptureErrors)
{
  auto color_image_sub = subscribe<sensor_msgs::msg::Image>(color_image_color_topic_name);
  auto depth_image_sub = subscribe<sensor_msgs::msg::Image>(depth_image_topic_name);
  auto snr_image_sub = subscribe<sensor_msgs::msg::Image>(snr_image_topic_name);

  setNodeParameter(parameter_intrinsics_source, "invalid_intrinsics_source");

  verifyTriggerResponseError(
    doSingleDefaultAcquisitionCaptureUsingFilePath(),
    "Invalid value for parameter 'intrinsics_source': 'invalid_intrinsics_source'. Expected one "
    "of: camera, frame.");

  ASSERT_EQ(color_image_sub.numMessages(), 0U);
  ASSERT_EQ(depth_image_sub.numMessages(), 0U);
  ASSERT_EQ(snr_image_sub.numMessages(), 0U);
}

TEST_F(ZividNodeTest, testCaptureInvalidIntrinsicsSourceCapture2D)
{
  auto color_camera_info_sub =
    subscribe<sensor_msgs::msg::CameraInfo>(color_camera_info_topic_name);

  verifyTriggerResponseSuccess(doSingleDefaultAcquisitionCapture2DUsingFilePath());
  ASSERT_EQ(color_camera_info_sub.numMessages(), 1U);
  assertCameraInfoForFileCamera(*color_camera_info_sub.lastMessage());

  setNodeParameter(parameter_intrinsics_source, "frame");
  verifyTriggerResponseSuccess(doSingleDefaultAcquisitionCapture2DUsingFilePath());
  ASSERT_EQ(color_camera_info_sub.numMessages(), 2U);
  assertCameraInfoForFileCamera(*color_camera_info_sub.lastMessage());
}

TEST_F(ZividNodeTest, testCaptureInvalidIntrinsicsSourceCapture2DNotAnError)
{
  AllCapture2DTopicsSubscriber topics_subscriber(*this);

  setNodeParameter(parameter_intrinsics_source, "invalid_intrinsics_source");
  verifyTriggerResponseSuccess(doSingleDefaultAcquisitionCapture2DUsingFilePath());
  topics_subscriber.assert_num_topics_received(1U);
}

class Capture2DOutputTest : public CaptureOutputTest
{
public:
  using ColorSpace = zivid_camera::ColorSpace;

  void testCapture2D(const std::string & settings_yaml, ColorSpace color_space)
  {
    AllCapture2DTopicsSubscriber all_capture_2d_topics_subscriber(*this);

    all_capture_2d_topics_subscriber.assert_num_topics_received(0);
    doCapture2DUsingFilePath(settings_yaml);

    rclcpp::sleep_for(short_wait_duration);
    all_capture_2d_topics_subscriber.assert_num_topics_received(1);

    const auto frame_2d_from_sdk =
      camera_.capture(deserializeZividDataModel<Zivid::Settings2D>(settings_yaml));

    auto verify_image_and_camera_info = [&](const auto & img, const auto & info) {
      assertCameraInfoForFileCamera(info);
      assertSensorMsgsImageMeta(img, 1944U, 1200U, 4U, "rgba8");
      switch (color_space) {
        case ColorSpace::sRGB:
          assertSensorMsgsImageContents(img, frame_2d_from_sdk.imageRGBA_SRGB());
          break;
        case ColorSpace::LinearRGB:
          assertSensorMsgsImageContents(img, frame_2d_from_sdk.imageRGBA());
          break;
        default:
          throw std::runtime_error(
            "Unknown color space: " + std::to_string(static_cast<int>(color_space)));
      }
    };

    verify_image_and_camera_info(
      *all_capture_2d_topics_subscriber.color_image_color_sub_.lastMessage(),
      *all_capture_2d_topics_subscriber.color_camera_info_sub_.lastMessage());

    rclcpp::sleep_for(short_wait_duration);
    all_capture_2d_topics_subscriber.assert_num_topics_received(1);

    doCapture2DUsingFilePath(settings_yaml);

    rclcpp::sleep_for(short_wait_duration);
    all_capture_2d_topics_subscriber.assert_num_topics_received(2);
    verify_image_and_camera_info(
      *all_capture_2d_topics_subscriber.color_image_color_sub_.lastMessage(),
      *all_capture_2d_topics_subscriber.color_camera_info_sub_.lastMessage());
  }
};

TEST_F(Capture2DOutputTest, testCapture2DDefaultSettings)
{
  testCapture2D(
    R"(
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
    - Acquisition:
       Aperture: 4
  )",
    ColorSpace::sRGB);
}

TEST_F(Capture2DOutputTest, testCapture2DCustomColorBalance)
{
  testCapture2D(
    R"(
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
    - Acquisition:
       Aperture: 4
  Processing:
    Color:
      Balance:
        Red: 2
        Blue: 3
        Green: 4
  )",
    ColorSpace::sRGB);
}

TEST_F(Capture2DOutputTest, testCapture2DLinearRGB)
{
  setNodeColorSpaceLinearRGB();
  testCapture2D(defaultSingleAcquisitionSettings2DYml(), ColorSpace::LinearRGB);
}

class CaptureAndSaveTest : public TestWithFileCamera
{
protected:
  void captureAndSaveToPath(const std::string & file_path, bool expect_success)
  {
    AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);

    setNodeParameter(parameter_settings_yaml, defaultSingleAcquisitionSettingsYml());

    auto request = std::make_shared<zivid_interfaces::srv::CaptureAndSave::Request>();
    request->file_path = file_path;

    if (std::filesystem::exists(file_path)) {
      std::filesystem::remove(file_path);
    }
    all_capture_topics_subscriber.assert_num_topics_received(0);
    if (expect_success) {
      auto response = doSrvRequest<zivid_interfaces::srv::CaptureAndSave>(
        capture_and_save_service_name, request, std::chrono::seconds{10});
      ASSERT_TRUE(response);
      verifyTriggerResponseSuccess(response);
      ASSERT_TRUE(std::filesystem::exists(file_path));
    } else {
      auto response = doSrvRequest<zivid_interfaces::srv::CaptureAndSave>(
        capture_and_save_service_name, request, std::chrono::seconds{10});
      ASSERT_TRUE(response);
      verifyTriggerResponseError(response);
      ASSERT_FALSE(std::filesystem::exists(file_path));
    }
    executor_.spin_some();
    all_capture_topics_subscriber.assert_num_topics_received(1);
  }
};

TEST_F(CaptureAndSaveTest, testCaptureAndSaveEmptyPathProvided) { captureAndSaveToPath("", false); }

TEST_F(CaptureAndSaveTest, testCaptureAndSaveInvalidPath)
{
  captureAndSaveToPath("invalid_path", false);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSaveInvalidExtension)
{
  captureAndSaveToPath(getTemporaryFilePath("invalid_extension.wrong").string(), false);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSaveZDF)
{
  captureAndSaveToPath(getTemporaryFilePath("valid.zdf").string(), true);

  // Note: the color_space parameter does not affect the ZDF file format, but we still want to test
  // that it does not break when set to either value.

  setNodeColorSpaceLinearRGB();
  captureAndSaveToPath(getTemporaryFilePath("valid_linear.zdf").string(), true);

  setNodeColorSpaceSRGB();
  captureAndSaveToPath(getTemporaryFilePath("valid_srgb.zdf").string(), true);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSavePLY)
{
  captureAndSaveToPath(getTemporaryFilePath("valid.ply").string(), true);

  setNodeColorSpaceLinearRGB();
  captureAndSaveToPath(getTemporaryFilePath("valid_linear.ply").string(), true);

  setNodeColorSpaceSRGB();
  captureAndSaveToPath(getTemporaryFilePath("valid_srgb.ply").string(), true);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSavePCD)
{
  captureAndSaveToPath(getTemporaryFilePath("valid.pcd").string(), true);

  setNodeColorSpaceLinearRGB();
  captureAndSaveToPath(getTemporaryFilePath("valid_linear.pcd").string(), true);

  setNodeColorSpaceSRGB();
  captureAndSaveToPath(getTemporaryFilePath("valid_srgb.pcd").string(), true);
}

TEST_F(CaptureAndSaveTest, testCaptureAndSaveXYZ)
{
  captureAndSaveToPath(getTemporaryFilePath("valid.xyz").string(), true);

  setNodeColorSpaceLinearRGB();
  captureAndSaveToPath(getTemporaryFilePath("valid_linear.xyz").string(), true);

  setNodeColorSpaceSRGB();
  captureAndSaveToPath(getTemporaryFilePath("valid_srgb.xyz").string(), true);
}

class ZividCATest : public CaptureOutputTest
{
protected:
  Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency
  toAPIAmbientLightFrequency(int ambient_light_frequency)
  {
    using Request = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request;
    using AmbientLightFrequency =
      Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency;
    switch (ambient_light_frequency) {
      case Request::AMBIENT_LIGHT_FREQUENCY_NONE:
        return AmbientLightFrequency::none;
      case Request::AMBIENT_LIGHT_FREQUENCY_50HZ:
        return AmbientLightFrequency::hz50;
      case Request::AMBIENT_LIGHT_FREQUENCY_60HZ:
        return AmbientLightFrequency::hz60;
    }
    throw std::runtime_error(
      "Could not convert value " + std::to_string(ambient_light_frequency) + " to Zivid API enum.");
  }

  decltype(auto) doCaptureAssistantRequest(
    int ambient_light_frequency, std::chrono::milliseconds duration)
  {
    using Request = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request;
    auto request = std::make_shared<Request>();
    request->ambient_light_frequency = ambient_light_frequency;
    request->max_capture_time = rclcpp::Duration{duration};

    return doSrvRequest<zivid_interfaces::srv::CaptureAssistantSuggestSettings>(
      capture_assistant_suggest_settings_service_name, request);
  }

  void performSuggestSettingsAndCompareWithCppAPI(
    std::chrono::milliseconds max_capture_time, int ambient_light_frequency)
  {
    // Set both settings_file and settings_yaml before calling the CA service
    setNodeParameter(parameter_settings_file_path, "foo");
    setNodeParameter(parameter_settings_yaml, "bar");

    auto points_sub = subscribe<sensor_msgs::msg::PointCloud2>(points_xyz_topic_name);

    auto caResponse = doCaptureAssistantRequest(ambient_light_frequency, max_capture_time);
    verifyTriggerResponseSuccess(caResponse);

    Zivid::CaptureAssistant::SuggestSettingsParameters suggest_settings_parameters{
      Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{max_capture_time},
      toAPIAmbientLightFrequency(ambient_light_frequency)};
    const auto api_suggested_settings =
      Zivid::CaptureAssistant::suggestSettings(camera_, suggest_settings_parameters);

    executor_.spin_some();

    ASSERT_EQ(
      getNodeStringParameter(parameter_settings_yaml),
      serializeZividDataModel(api_suggested_settings));
    ASSERT_EQ(caResponse->suggested_settings, serializeZividDataModel(api_suggested_settings));
    // settings_file_path parameter has been reset to empty
    ASSERT_EQ(getNodeStringParameter(parameter_settings_file_path), "");
    ASSERT_EQ(points_sub.numMessages(), 0U);

    // Triggering capture now should work
    doStdSrvsTriggerRequest(capture_service_name, capture_service_timeout);
    executor_.spin_some();
    ASSERT_EQ(points_sub.numMessages(), 1U);
  }
};

TEST_F(ZividCATest, testCaptureAssistantServiceAvailable)
{
  auto client = test_node_->create_client<zivid_interfaces::srv::CaptureAssistantSuggestSettings>(
    capture_assistant_suggest_settings_service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds{3}));
}

TEST_F(ZividCATest, testDifferentMaxCaptureTimeAndAmbientLightFrequency)
{
  using Request = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request;
  for (std::chrono::milliseconds max_capture_time :
       {std::chrono::milliseconds{200}, std::chrono::milliseconds{1'200},
        std::chrono::milliseconds{10'000}}) {
    for (auto ambient_light_frequency :
         {Request::AMBIENT_LIGHT_FREQUENCY_NONE, Request::AMBIENT_LIGHT_FREQUENCY_50HZ,
          Request::AMBIENT_LIGHT_FREQUENCY_60HZ}) {
      performSuggestSettingsAndCompareWithCppAPI(max_capture_time, ambient_light_frequency);
    }
  }
}

TEST_F(ZividCATest, testCaptureAssistantWithMaxCaptureTimeZeroFails)
{
  using Request = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request;
  verifyTriggerResponseError(
    doCaptureAssistantRequest(Request::AMBIENT_LIGHT_FREQUENCY_NONE, std::chrono::milliseconds{0}),
    "MaxCaptureTime{ 0 } is not in range [200, 10000]");
}

TEST_F(ZividCATest, testCaptureAssistantWithMaxCaptureTimeMinMax)
{
  using Request = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request;
  constexpr auto small_delta = std::chrono::milliseconds{1};
  const auto valid_range =
    Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime::validRange();

  verifyTriggerResponseError(
    doCaptureAssistantRequest(
      Request::AMBIENT_LIGHT_FREQUENCY_NONE, valid_range.min() - small_delta),
    "MaxCaptureTime{ 199 } is not in range [200, 10000]");

  verifyTriggerResponseSuccess(
    doCaptureAssistantRequest(Request::AMBIENT_LIGHT_FREQUENCY_NONE, valid_range.min()));

  verifyTriggerResponseError(
    doCaptureAssistantRequest(
      Request::AMBIENT_LIGHT_FREQUENCY_NONE, valid_range.max() + small_delta),
    "MaxCaptureTime{ 10001 } is not in range [200, 10000]");

  verifyTriggerResponseSuccess(
    doCaptureAssistantRequest(Request::AMBIENT_LIGHT_FREQUENCY_NONE, valid_range.max()));
}

TEST_F(ZividCATest, testCaptureAssistantDefaultAmbientLightFrequencyWorks)
{
  using Request = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request;
  auto request = std::make_shared<Request>();
  request->max_capture_time = rclcpp::Duration{std::chrono::seconds{1}};
  verifyTriggerResponseSuccess(doSrvRequest<zivid_interfaces::srv::CaptureAssistantSuggestSettings>(
    capture_assistant_suggest_settings_service_name, request));
}

TEST_F(ZividCATest, testCaptureAssistantInvalidAmbientLightFrequencyFails)
{
  using Request = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request;
  verifyTriggerResponseError(
    doCaptureAssistantRequest(255, std::chrono::seconds{1}),
    "Unhandled AMBIENT_LIGHT_FREQUENCY value: 255");
  verifyTriggerResponseSuccess(
    doCaptureAssistantRequest(Request::AMBIENT_LIGHT_FREQUENCY_60HZ, std::chrono::seconds{1}));
}

TEST_F(TestWithFileCamera, testProjectionServices)
{
  auto capture_2d_response = doEmptySrvRequest<std_srvs::srv::Trigger>(projection_capture_2d);
  verifyTriggerResponseError(capture_2d_response, "Camera is not projecting");

  auto resolution_response =
    doEmptySrvRequest<zivid_interfaces::srv::ProjectionResolution>(projection_resolution);
  verifyTriggerResponseError(
    resolution_response, "The camera does not support image display using the projector");

  auto start_response = doEmptySrvRequest<zivid_interfaces::srv::ProjectionStart>(projection_start);
  verifyTriggerResponseError(
    start_response, "The camera does not support image display using the projector");

  auto status_response =
    doEmptySrvRequest<zivid_interfaces::srv::ProjectionStatus>(projection_status);
  verifyTriggerResponseSuccess(status_response);
  ASSERT_EQ(status_response->projecting, false);

  auto stop_response = doEmptySrvRequest<std_srvs::srv::Trigger>(projection_stop);
  verifyTriggerResponseSuccess(stop_response);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  const auto return_code = RUN_ALL_TESTS();
  rclcpp::shutdown();

  ZividCameraNodeWrapper::reset();

  return return_code;
}
