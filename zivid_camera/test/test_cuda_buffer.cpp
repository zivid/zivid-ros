// Copyright 2026 Zivid AS
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

// Tests for parameter use_cuda_buffers. Only built when the driver is built with CUDA buffer
// support. A driver node publishing in CUDA buffers and a driver node publishing via host memory
// share one Zivid application and file camera, and the published data must be identical.

#include <Zivid/Application.h>
#include <Zivid/ComputeDevice.h>
#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <cuda_buffer/cuda_buffer_api.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>
#include <zivid_camera/zivid_camera.hpp>

#ifdef _WIN32
#define ZIVID_SAMPLE_DATA_DIR "C:\\ProgramData\\Zivid\\"
#else
#define ZIVID_SAMPLE_DATA_DIR "/usr/share/Zivid/data/"
#endif

namespace
{
constexpr auto file_camera_path = ZIVID_SAMPLE_DATA_DIR "FileCameraZivid2M70.zfc";
constexpr auto gpu_namespace = "gpu";
constexpr auto cpu_namespace = "cpu";
constexpr auto service_timeout = std::chrono::seconds{20};

constexpr auto settings_yml = R"(
__version__:
  serializer: 1
  data: 17
Settings:
  Acquisitions:
    - Acquisition:
)";

constexpr auto settings_2d_yml = R"(
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
    - Acquisition:
)";

std::vector<uint8_t> copyToHost(const rosidl::Buffer<uint8_t> & data)
{
  if (data.get_backend_type() != "cuda") {
    return data.to_vector();
  }
  cudaStream_t stream = nullptr;
  EXPECT_EQ(cudaStreamCreate(&stream), cudaSuccess);
  std::vector<uint8_t> host(data.size());
  {
    auto read_handle = cuda_buffer_backend::from_input_buffer(data, stream);
    EXPECT_EQ(
      cudaMemcpyAsync(
        host.data(), read_handle.get_ptr(), host.size(), cudaMemcpyDeviceToHost, stream),
      cudaSuccess);
    EXPECT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  }
  EXPECT_EQ(cudaStreamDestroy(stream), cudaSuccess);
  return host;
}

template <typename Msg>
class LastMessage
{
public:
  LastMessage(rclcpp::Node & node, const std::string & topic, const std::string & buffer_backends)
  : subscription_{node.create_subscription<Msg>(
      topic, rclcpp::SensorDataQoS{}.reliable(),
      [this](typename Msg::ConstSharedPtr msg) { message_ = msg; },
      subscriptionOptions(buffer_backends))}
  {
  }

  typename Msg::ConstSharedPtr message() const { return message_; }
  void reset() { message_.reset(); }

private:
  static rclcpp::SubscriptionOptions subscriptionOptions(const std::string & buffer_backends)
  {
    rclcpp::SubscriptionOptions options;
    options.acceptable_buffer_backends = buffer_backends;
    return options;
  }

  typename rclcpp::Subscription<Msg>::SharedPtr subscription_;
  typename Msg::ConstSharedPtr message_;
};
}  // namespace

class CudaBufferTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // Make the CUDA primary context current before creating the Zivid application, so that the
    // Zivid SDK uses the same context as the CUDA buffer backend.
    ASSERT_EQ(cudaFree(nullptr), cudaSuccess);
    application_ = std::make_shared<Zivid::Application>();
    if (application_->computeDevice().backend() != Zivid::ComputeBackend::cuda) {
      application_.reset();
    }
  }

  static void TearDownTestSuite() { application_.reset(); }

  void SetUp() override
  {
    if (application_ == nullptr) {
      GTEST_SKIP() << "The Zivid SDK does not use the CUDA backend";
    }
    gpu_node_ = makeDriverNode(gpu_namespace, true);
    cpu_node_ = makeDriverNode(cpu_namespace, false);
    test_node_ = rclcpp::Node::make_shared("cuda_buffer_test_node");
    executor_.add_node(gpu_node_);
    executor_.add_node(cpu_node_);
    executor_.add_node(test_node_);
  }

  static std::shared_ptr<zivid_camera::ZividCamera> makeDriverNode(
    const std::string & ns, bool use_cuda_buffers,
    const std::vector<rclcpp::Parameter> & extra_parameters = {})
  {
    auto options = rclcpp::NodeOptions{}
                     .append_parameter_override("file_camera_path", file_camera_path)
                     .append_parameter_override("use_cuda_buffers", use_cuda_buffers)
                     .append_parameter_override("health_check_diagnostics_period", 0.0)
                     .append_parameter_override("settings_yaml", settings_yml)
                     .append_parameter_override("settings_2d_yaml", settings_2d_yml);
    for (const auto & parameter : extra_parameters) {
      options.append_parameter_override(parameter.get_name(), parameter.get_parameter_value());
    }
    return std::make_shared<zivid_camera::ZividCamera>("zivid_camera", ns, options, application_);
  }

  void setColorSpace(const std::string & color_space)
  {
    for (const auto & node : {gpu_node_, cpu_node_}) {
      ASSERT_TRUE(node->set_parameter(rclcpp::Parameter{"color_space", color_space}).successful);
    }
  }

  void callTrigger(const std::string & service)
  {
    auto client = test_node_->create_client<std_srvs::srv::Trigger>(service);
    ASSERT_TRUE(client->wait_for_service(service_timeout)) << service;
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(
      executor_.spin_until_future_complete(future, service_timeout),
      rclcpp::FutureReturnCode::SUCCESS)
      << service;
    const auto response = future.get();
    ASSERT_TRUE(response->success) << service << ": " << response->message;
  }

  void spinUntil(const std::function<bool()> & condition)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{10};
    while (!condition() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      rclcpp::sleep_for(std::chrono::milliseconds{10});
    }
    ASSERT_TRUE(condition()) << "Timed out waiting for messages";
  }

  // Captures on both driver nodes, then verifies that the message on `topic` published by the GPU
  // node has identical metadata and data as the message from the CPU node. The subscription to the
  // GPU node accepts the given buffer backends, and the message must arrive with
  // `expected_gpu_backend`.
  template <typename Msg>
  void captureAndCompare(
    const std::string & service, const std::string & topic,
    const std::string & gpu_subscription_backends = "cuda",
    const std::string & expected_gpu_backend = "cuda")
  {
    LastMessage<Msg> gpu_message{
      *test_node_, std::string{"/"} + gpu_namespace + "/" + topic, gpu_subscription_backends};
    LastMessage<Msg> cpu_message{
      *test_node_, std::string{"/"} + cpu_namespace + "/" + topic, "cpu"};
    spinUntil([&] {
      return gpu_node_->count_subscribers(std::string{"/"} + gpu_namespace + "/" + topic) > 0 &&
             cpu_node_->count_subscribers(std::string{"/"} + cpu_namespace + "/" + topic) > 0;
    });

    callTrigger(std::string{"/"} + gpu_namespace + "/" + service);
    callTrigger(std::string{"/"} + cpu_namespace + "/" + service);
    spinUntil([&] { return gpu_message.message() && cpu_message.message(); });

    const auto & gpu = *gpu_message.message();
    const auto & cpu = *cpu_message.message();
    EXPECT_EQ(gpu.data.get_backend_type(), expected_gpu_backend) << topic;
    EXPECT_EQ(cpu.data.get_backend_type(), "cpu") << topic;
    EXPECT_EQ(gpu.width, cpu.width);
    EXPECT_EQ(gpu.height, cpu.height);
    if constexpr (std::is_same_v<Msg, sensor_msgs::msg::Image>) {
      EXPECT_EQ(gpu.encoding, cpu.encoding);
      EXPECT_EQ(gpu.step, cpu.step);
    } else {
      EXPECT_EQ(gpu.fields, cpu.fields);
      EXPECT_EQ(gpu.point_step, cpu.point_step);
      EXPECT_EQ(gpu.row_step, cpu.row_step);
    }
    ASSERT_EQ(gpu.data.size(), cpu.data.size()) << topic;
    ASSERT_GT(cpu.data.size(), 0U) << topic;
    const auto gpu_data = copyToHost(gpu.data);
    const auto cpu_data = cpu.data.to_vector();
    EXPECT_EQ(std::memcmp(gpu_data.data(), cpu_data.data(), cpu_data.size()), 0)
      << "Data differs on topic " << topic;
  }

  static std::shared_ptr<Zivid::Application> application_;
  std::shared_ptr<zivid_camera::ZividCamera> gpu_node_;
  std::shared_ptr<zivid_camera::ZividCamera> cpu_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

std::shared_ptr<Zivid::Application> CudaBufferTest::application_;

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

TEST_F(CudaBufferTest, testPointsXYZ) { captureAndCompare<PointCloud2>("capture", "points/xyz"); }

TEST_F(CudaBufferTest, testPointsXYZRGBALinearRGB)
{
  setColorSpace("linear_rgb");
  captureAndCompare<PointCloud2>("capture", "points/xyzrgba");
}

TEST_F(CudaBufferTest, testPointsXYZRGBASRGB)
{
  setColorSpace("srgb");
  captureAndCompare<PointCloud2>("capture", "points/xyzrgba");
}

TEST_F(CudaBufferTest, testNormalsXYZ) { captureAndCompare<PointCloud2>("capture", "normals/xyz"); }

TEST_F(CudaBufferTest, testColorImageFrom3DLinearRGB)
{
  setColorSpace("linear_rgb");
  captureAndCompare<Image>("capture", "color/image_color");
}

TEST_F(CudaBufferTest, testColorImageFrom3DSRGB)
{
  setColorSpace("srgb");
  captureAndCompare<Image>("capture", "color/image_color");
}

TEST_F(CudaBufferTest, testDepthImage) { captureAndCompare<Image>("capture", "depth/image"); }

TEST_F(CudaBufferTest, testSnrImage) { captureAndCompare<Image>("capture", "snr/image"); }

TEST_F(CudaBufferTest, testColorImageFrom2DLinearRGB)
{
  setColorSpace("linear_rgb");
  captureAndCompare<Image>("capture_2d", "color/image_color");
}

TEST_F(CudaBufferTest, testColorImageFrom2DSRGB)
{
  setColorSpace("srgb");
  captureAndCompare<Image>("capture_2d", "color/image_color");
}

TEST_F(CudaBufferTest, testCpuSubscriberReceivesHostCopy)
{
  captureAndCompare<PointCloud2>("capture", "points/xyz", "cpu", "cpu");
  captureAndCompare<Image>("capture_2d", "color/image_color", "cpu", "cpu");
}

TEST_F(CudaBufferTest, testOnlyRawImageTransportIsAdvertised)
{
  // A compressed image subscriber would make the in-process compressed publisher plugin read the
  // CUDA-backed image on the CPU. Verify that the GPU node only advertises the raw transport, and
  // that capturing with such a subscriber present still succeeds.
  auto compressed_subscription = test_node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    std::string{"/"} + gpu_namespace + "/color/image_color/compressed", 10,
    [](sensor_msgs::msg::CompressedImage::ConstSharedPtr) {});
  EXPECT_EQ(
    gpu_node_->count_publishers(std::string{"/"} + gpu_namespace + "/color/image_color/compressed"),
    0U);
  EXPECT_GT(
    cpu_node_->count_publishers(std::string{"/"} + cpu_namespace + "/color/image_color/compressed"),
    0U);
  captureAndCompare<Image>("capture_2d", "color/image_color");
}

TEST_F(CudaBufferTest, testNonRawImageTransportIsRejected)
{
  EXPECT_THROW(
    makeDriverNode(
      "compressed", true,
      {rclcpp::Parameter{
        "color.image_color.enable_pub_plugins",
        std::vector<std::string>{"image_transport/raw", "image_transport/compressed"}}}),
    std::runtime_error);
}

TEST_F(CudaBufferTest, testLatchedPublisherIsRejected)
{
  EXPECT_THROW(
    makeDriverNode(
      "latched", true, {rclcpp::Parameter{"use_latched_publisher_for_points_xyz", true}}),
    std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto return_code = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return return_code;
}
