// Copyright 2025 Zivid AS
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

#pragma once

#include <Zivid/Array2D.h>
#include <Zivid/Camera.h>
#include <Zivid/Color.h>
#include <Zivid/Frame.h>
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zivid_camera/zivid_camera.hpp>

enum class FileCameraMode
{
  Default,
  CalibrationBoard,
};

enum class NodeReusePolicy
{
  AllowReuse,
  RestartNode,
};

class ZividCameraNodeWrapper
{
public:
  static std::shared_ptr<zivid_camera::ZividCamera> getOrConstruct(
    FileCameraMode fileCamera, NodeReusePolicy reusePolicy);

  static std::shared_ptr<zivid_camera::ZividCamera> get();

  static void reset();

private:
  static std::optional<FileCameraMode> m_fileCameraMode;
  static std::shared_ptr<zivid_camera::ZividCamera> m_zividRosNode;
};

inline std::filesystem::path getTemporaryFilePath(const std::string & name)
{
  return std::filesystem::temp_directory_path() / name;
}

inline bool containsSubstring(const std::string & value, const std::string & substr)
{
  return value.find(substr) != std::string::npos;
}

class TmpFile
{
public:
  TmpFile(const std::string & name, const std::string & content)
  : m_path{getTemporaryFilePath(name)}
  {
    if (std::filesystem::exists(m_path)) {
      throw std::runtime_error{"Temporary file already exists: " + m_path.string()};
    }
    std::ofstream file{m_path};
    file << content;
    file.close();
  }

  TmpFile(const TmpFile &) noexcept = delete;
  TmpFile(TmpFile &&) noexcept = delete;
  TmpFile & operator=(const TmpFile &) = delete;
  TmpFile & operator=(TmpFile &&) = delete;

  ~TmpFile() { std::filesystem::remove(m_path); }

  std::string string() const { return m_path.string(); }

private:
  std::filesystem::path m_path;
};

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

  void printLine() { std::cerr << std::string(80, '-') << "\n"; }
};

class ZividNodeTest : public ZividNodeTestBase
{
protected:
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  static constexpr auto short_wait_duration{std::chrono::milliseconds{500}};
  static constexpr auto default_service_timeout{std::chrono::seconds{10}};
  static constexpr auto capture_service_timeout{std::chrono::seconds{20}};
  static constexpr auto capture_service_name = "capture";
  static constexpr auto capture_and_save_service_name = "capture_and_save";
  static constexpr auto capture_2d_service_name = "capture_2d";
  static constexpr auto capture_assistant_suggest_settings_service_name =
    "capture_assistant/"
    "suggest_settings";

  static constexpr auto color_camera_info_topic_name = "color/camera_info";
  static constexpr auto color_image_color_topic_name = "color/image_color";
  static constexpr auto depth_camera_info_topic_name = "depth/camera_info";
  static constexpr auto depth_image_topic_name = "depth/image";
  static constexpr auto snr_camera_info_topic_name = "depth/camera_info";
  static constexpr auto snr_image_topic_name = "snr/image";
  static constexpr auto points_xyz_topic_name = "points/xyz";
  static constexpr auto points_xyzrgba_topic_name = "points/xyzrgba";
  static constexpr auto normals_xyz_topic_name = "normals/xyz";

  static constexpr auto parameter_settings_file_path = "settings_file_path";
  static constexpr auto parameter_settings_yaml = "settings_yaml";
  static constexpr auto parameter_settings_2d_file_path = "settings_2d_file_path";
  static constexpr auto parameter_settings_2d_yaml = "settings_2d_yaml";

  ZividNodeTest(
    FileCameraMode file_camera_mode = FileCameraMode::Default,
    NodeReusePolicy camera_node_reuse_policy = NodeReusePolicy::AllowReuse)
  : test_node_(rclcpp::Node::make_shared("test_node"))
  {
    executor_.add_node(test_node_);
    executor_.add_node(
      ZividCameraNodeWrapper::getOrConstruct(file_camera_mode, camera_node_reuse_policy));

    // Reset test state
    setNodeParameter(parameter_settings_file_path, "");
    setNodeParameter(parameter_settings_yaml, "");
    setNodeParameter(parameter_settings_2d_file_path, "");
    setNodeParameter(parameter_settings_2d_yaml, "");
  }

  template <typename SrvType>
  decltype(auto) doSrvRequest(
    const std::string & service, std::shared_ptr<typename SrvType::Request> request,
    std::chrono::milliseconds timeout = default_service_timeout)
  {
    auto client = test_node_->create_client<SrvType>(service);
    if (!client->wait_for_service(timeout)) {
      throw std::runtime_error("The service '" + service + "' is not ready");
    }

    auto future = client->async_send_request(request);

    if (
      executor_.spin_until_future_complete(future, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service " << service);
      throw std::runtime_error(
        "Failed to invoke service '" + service + "'. No response within timeout.");
    }

    return future.get();
  }

  template <typename SrvType>
  decltype(auto) doEmptySrvRequest(
    const std::string & service, std::chrono::milliseconds timeout = default_service_timeout)
  {
    auto request = std::make_shared<typename SrvType::Request>();
    return doSrvRequest<SrvType>(service, request, timeout);
  }

  decltype(auto) doStdSrvsTriggerRequest(
    const std::string & service, std::chrono::milliseconds timeout = default_service_timeout)
  {
    return doEmptySrvRequest<std_srvs::srv::Trigger>(service, timeout);
  }

  template <typename Type>
  class SubscriptionWrapper
  {
  public:
    static SubscriptionWrapper<Type> make(rclcpp::Node::SharedPtr node, const std::string & topic)
    {
      auto w = SubscriptionWrapper<Type>();
      std::function<void(const std::shared_ptr<const Type> &)> cb = [impl_ptr =
                                                                       w.impl_.get()](auto msg) {
        impl_ptr->num_messages_++;
        impl_ptr->last_message_ = msg;
      };
      w.impl_->subscription_ = node->create_subscription<Type>(topic, 10, cb);
      return w;
    }

    decltype(auto) lastMessage() const { return impl_->last_message_; }

    std::size_t numMessages() const { return impl_->num_messages_; }

  private:
    SubscriptionWrapper() : impl_{std::make_unique<Impl>()} {}

    struct Impl
    {
      typename rclcpp::Subscription<Type>::SharedPtr subscription_;
      typename Type::ConstSharedPtr last_message_;
      std::size_t num_messages_ = 0;
    };

    std::unique_ptr<Impl> impl_;
  };

  std::string getNodeStringParameter(const std::string & key)
  {
    return ZividCameraNodeWrapper::get()->get_parameter(key).as_string();
  }

  void setNodeParameter(const std::string & key, const std::string & value)
  {
    auto set_result = ZividCameraNodeWrapper::get()->set_parameter(rclcpp::Parameter{key, value});
    ASSERT_TRUE(set_result.successful);
    ASSERT_EQ(getNodeStringParameter(key), value);
  }

  decltype(auto) doCaptureUsingFilePath(const std::string & ymlContent)
  {
    setNodeParameter(parameter_settings_yaml, "");
    auto tmp_file = TmpFile("settings.yml", ymlContent);
    setNodeParameter(parameter_settings_file_path, tmp_file.string());
    return doStdSrvsTriggerRequest(capture_service_name, capture_service_timeout);
  }

  decltype(auto) doCapture2DUsingFilePath(const std::string & ymlContent)
  {
    setNodeParameter(parameter_settings_2d_yaml, "");
    auto tmp_file = TmpFile("settings_2d.yml", ymlContent);
    setNodeParameter(parameter_settings_2d_file_path, tmp_file.string());
    return doStdSrvsTriggerRequest(capture_2d_service_name, capture_service_timeout);
  }

  decltype(auto) doCaptureUsingYmlString(const std::string & yml)
  {
    setNodeParameter(parameter_settings_file_path, "");
    setNodeParameter(parameter_settings_yaml, yml);
    return doStdSrvsTriggerRequest(capture_service_name, capture_service_timeout);
  }

  decltype(auto) doCapture2DUsingYmlString(const std::string & yml)
  {
    setNodeParameter(parameter_settings_2d_file_path, "");
    setNodeParameter(parameter_settings_2d_yaml, yml);
    return doStdSrvsTriggerRequest(capture_2d_service_name, capture_service_timeout);
  }

  constexpr decltype(auto) defaultSingleAcquisitionSettingsYml()
  {
    return
      R"(
__version__:
  serializer: 1
  data: 17
Settings:
  Acquisitions:
    - Acquisition:
)";
  }

  constexpr decltype(auto) defaultSingleAcquisitionSettings2DYml()
  {
    return
      R"(
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
    - Acquisition:
)";
  }

  decltype(auto) doSingleDefaultAcquisitionCaptureUsingFilePath()
  {
    return doCaptureUsingFilePath(defaultSingleAcquisitionSettingsYml());
  }

  decltype(auto) doSingleDefaultAcquisitionCapture2DUsingFilePath()
  {
    return doCapture2DUsingFilePath(defaultSingleAcquisitionSettings2DYml());
  }

  template <typename ZividDataModel>
  auto serializeZividDataModel(const ZividDataModel & dm)
  {
    return dm.serialize();
  }

  template <typename ZividDataModel>
  auto deserializeZividDataModel(const std::string & serialized)
  {
    return ZividDataModel::fromSerialized(serialized);
  }

  template <typename ResponseSharedPtr>
  auto verifyTriggerResponseSuccess(const ResponseSharedPtr & response)
  {
    ASSERT_TRUE(response->success);
    ASSERT_EQ(response->message, "");
  }

  template <typename ResponseSharedPtr>
  auto verifyTriggerResponseError(const ResponseSharedPtr & response)
  {
    ASSERT_FALSE(response->success);
    ASSERT_NE(response->message, "");
  }

  template <typename ResponseSharedPtr>
  auto verifyTriggerResponseError(const ResponseSharedPtr & response, const std::string & message)
  {
    verifyTriggerResponseError(response);
    ASSERT_EQ(response->message, message);
  }

  template <class Type>
  SubscriptionWrapper<Type> subscribe(const std::string & topic)
  {
    return SubscriptionWrapper<Type>::make(test_node_, topic);
  }

  class AllCaptureTopicsSubscriber
  {
  public:
    explicit AllCaptureTopicsSubscriber(ZividNodeTest & nodeTest)
    : color_camera_info_sub_(
        nodeTest.subscribe<sensor_msgs::msg::CameraInfo>(color_camera_info_topic_name)),
      color_image_color_sub_(
        nodeTest.subscribe<sensor_msgs::msg::Image>(color_image_color_topic_name)),
      depth_camera_info_sub_(
        nodeTest.subscribe<sensor_msgs::msg::CameraInfo>(depth_camera_info_topic_name)),
      depth_image_sub_(nodeTest.subscribe<sensor_msgs::msg::Image>(depth_image_topic_name)),
      snr_camera_info_sub_(
        nodeTest.subscribe<sensor_msgs::msg::CameraInfo>(snr_camera_info_topic_name)),
      snr_image_sub_(nodeTest.subscribe<sensor_msgs::msg::Image>(snr_image_topic_name)),
      points_xyz_sub_(nodeTest.subscribe<sensor_msgs::msg::PointCloud2>(points_xyz_topic_name)),
      points_xyzrgba_sub_(
        nodeTest.subscribe<sensor_msgs::msg::PointCloud2>(points_xyzrgba_topic_name)),
      normals_xyz_sub_(nodeTest.subscribe<sensor_msgs::msg::PointCloud2>(normals_xyz_topic_name))
    {
    }

    SubscriptionWrapper<sensor_msgs::msg::CameraInfo> color_camera_info_sub_;
    SubscriptionWrapper<sensor_msgs::msg::Image> color_image_color_sub_;
    SubscriptionWrapper<sensor_msgs::msg::CameraInfo> depth_camera_info_sub_;
    SubscriptionWrapper<sensor_msgs::msg::Image> depth_image_sub_;
    SubscriptionWrapper<sensor_msgs::msg::CameraInfo> snr_camera_info_sub_;
    SubscriptionWrapper<sensor_msgs::msg::Image> snr_image_sub_;
    SubscriptionWrapper<sensor_msgs::msg::PointCloud2> points_xyz_sub_;
    SubscriptionWrapper<sensor_msgs::msg::PointCloud2> points_xyzrgba_sub_;
    SubscriptionWrapper<sensor_msgs::msg::PointCloud2> normals_xyz_sub_;

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

  class AllCapture2DTopicsSubscriber
  {
  public:
    explicit AllCapture2DTopicsSubscriber(ZividNodeTest & node_test)
    : color_camera_info_sub_(
        node_test.subscribe<sensor_msgs::msg::CameraInfo>(color_camera_info_topic_name)),
      color_image_color_sub_(
        node_test.subscribe<sensor_msgs::msg::Image>(color_image_color_topic_name))
    {
    }

    SubscriptionWrapper<sensor_msgs::msg::CameraInfo> color_camera_info_sub_;
    SubscriptionWrapper<sensor_msgs::msg::Image> color_image_color_sub_;

    void assert_num_topics_received(std::size_t num_topics)
    {
      ASSERT_EQ(color_camera_info_sub_.numMessages(), num_topics);
      ASSERT_EQ(color_image_color_sub_.numMessages(), num_topics);
    }
  };

  template <std::size_t N>
  void assertArrayDoubleEq(
    const std::array<double, N> & actual, const std::array<double, N> & expected) const
  {
    for (std::size_t i = 0; i < N; i++) {
      EXPECT_DOUBLE_EQ(actual[i], expected[i]);
    }
  }

  void assertSensorMsgsPointCloud2Meta(
    const sensor_msgs::msg::PointCloud2 & point_cloud, std::size_t width, std::size_t height,
    std::size_t point_step)
  {
    ASSERT_EQ(point_cloud.width, width);
    ASSERT_EQ(point_cloud.height, height);
    ASSERT_EQ(point_cloud.point_step, point_step);
    ASSERT_EQ(point_cloud.row_step, point_cloud.width * point_cloud.point_step);
    ASSERT_EQ(point_cloud.is_dense, false);
    ASSERT_EQ(
      point_cloud.data.size(), point_cloud.width * point_cloud.height * point_cloud.point_step);
  }

  void assertSensorMsgsImageMeta(
    const sensor_msgs::msg::Image & image, std::size_t width, std::size_t height,
    std::size_t bytes_per_pixel, const std::string & encoding)
  {
    ASSERT_EQ(image.width, width);
    ASSERT_EQ(image.height, height);
    ASSERT_EQ(image.step, bytes_per_pixel * image.width);
    ASSERT_EQ(image.data.size(), image.step * image.height);
    ASSERT_EQ(image.encoding, encoding);
    ASSERT_EQ(image.is_bigendian, false);
  }

  void assertSensorMsgsImageContents(
    const sensor_msgs::msg::Image & image, const Zivid::Array2D<Zivid::ColorRGBA> & expected_rgba)
  {
    ASSERT_EQ(image.width, expected_rgba.width());
    ASSERT_EQ(image.height, expected_rgba.height());
    for (std::size_t i = 0; i < expected_rgba.size(); i++) {
      const auto expected_pixel = expected_rgba(i);
      const auto index = i * 4U;
      ASSERT_EQ(image.data[index], expected_pixel.r);
      ASSERT_EQ(image.data[index + 1], expected_pixel.g);
      ASSERT_EQ(image.data[index + 2], expected_pixel.b);
      ASSERT_EQ(image.data[index + 3], expected_pixel.a);
      ASSERT_EQ(expected_pixel.a, 255);
    }
  }

  template <typename FieldType>
  void assertPointCloud2Field(
    const FieldType & field, const std::string & name, uint32_t offset, uint32_t datatype,
    uint32_t count)
  {
    ASSERT_EQ(field.name, name);
    ASSERT_EQ(field.offset, offset);
    ASSERT_EQ(field.datatype, datatype);
    ASSERT_EQ(field.count, count);
  }

  void assertCameraInfoForFileCamera(const sensor_msgs::msg::CameraInfo & ci) const
  {
    ASSERT_EQ(ci.width, 1944U);
    ASSERT_EQ(ci.height, 1200U);
    ASSERT_EQ(ci.distortion_model, "plumb_bob");

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    assertArrayDoubleEq(
      ci.k,
      std::array<double, 9>{
        1781.447998046875, 0, 990.49267578125, 0, 1781.5296630859375, 585.81781005859375, 0, 0, 1});

    // R = I
    assertArrayDoubleEq(ci.r, std::array<double, 9>{1, 0, 0, 0, 1, 0, 0, 0, 1});

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    assertArrayDoubleEq(
      ci.p, std::array<double, 12>{
              1781.447998046875, 0, 990.49267578125, 0, 0, 1781.5296630859375, 585.81781005859375,
              0, 0, 0, 1, 0});
  }
};

class TestWithFileCamera : public ZividNodeTest
{
protected:
  TestWithFileCamera();
  Zivid::Camera camera_;
};
