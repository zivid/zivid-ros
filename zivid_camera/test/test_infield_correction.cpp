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

#include <gtest/gtest.h>

#include <ctime>
#include <std_srvs/srv/trigger.hpp>
#include <test_zivid_camera.hpp>
#include <zivid_interfaces/srv/infield_correction_capture.hpp>
#include <zivid_interfaces/srv/infield_correction_compute.hpp>
#include <zivid_interfaces/srv/infield_correction_read.hpp>

class TestWithCalibrationBoard : public ZividNodeTest
{
protected:
  TestWithCalibrationBoard(NodeReusePolicy reusePolicy = NodeReusePolicy::AllowReuse)
  : ZividNodeTest{FileCameraMode::CalibrationBoard, reusePolicy}
  {
  }

  static void verifyInfieldComputeSuccess(
    const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> & response,
    int number_of_captures, bool compute_and_write = false)
  {
    const std::string operation_message =
      (compute_and_write ? "successfully written to camera" : "computed successfully");
    ASSERT_TRUE(response->success);
    ASSERT_PRED2(
      containsSubstring, response->message,
      "Camera correction " + operation_message +
        ".\nNumber of captures: " + std::to_string(number_of_captures) + "\nCamera metrics:\n");
    ASSERT_EQ(response->infield_correction_started, true);
    ASSERT_EQ(response->number_of_captures, number_of_captures);
    ASSERT_EQ(response->trueness_errors.size(), number_of_captures);
    ASSERT_GT(response->average_trueness_error, 0.0);
    ASSERT_LT(response->average_trueness_error, 0.01);
    ASSERT_EQ(response->average_trueness_error, response->maximum_trueness_error);
    ASSERT_GT(response->dimension_accuracy, 0.0);
    ASSERT_LT(response->dimension_accuracy, 0.01);
    ASSERT_GT(response->z_min, 0.1);
    ASSERT_LT(response->z_min, 10.0);
    ASSERT_GT(response->z_max, response->z_min);
    ASSERT_LT(response->z_max, 10.0);
  }

  static void verifyYearsOfTimeSinceEpoch(const builtin_interfaces::msg::Time & time)
  {
    const auto seconds_since_epoch = static_cast<std::time_t>(time.sec);
    std::tm * tm_info = std::localtime(&seconds_since_epoch);
    const int year = tm_info->tm_year + 1900;
    ASSERT_GE(year, 2025);
    ASSERT_LE(year, 2125);  // This needs to be updated in 100 years.
  }

  void infieldStartCapture()
  {
    auto start = doStdSrvsTriggerRequest("infield_correction/start");
    verifyTriggerResponseSuccess(start);

    auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
      "infield_correction/capture", capture_service_timeout);
    ASSERT_TRUE(capture->success);
  }

  void infieldStartCaptureWrite()
  {
    infieldStartCapture();

    auto compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
      "infield_correction/compute_and_write");
    verifyInfieldComputeSuccess(compute, 1, true);
  }
};

class TestWithCalibrationBoardFreshNode : public TestWithCalibrationBoard
{
protected:
  TestWithCalibrationBoardFreshNode() : TestWithCalibrationBoard{NodeReusePolicy::RestartNode} {}
};

TEST_F(ZividNodeTest, testInfieldCorrectionCaptureFailed)
{
  AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);
  all_capture_topics_subscriber.assert_num_topics_received(0);

  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture", capture_service_timeout);
  all_capture_topics_subscriber.assert_num_topics_received(1);
  ASSERT_FALSE(capture->success);
  ASSERT_EQ(
    capture->status,
    zivid_interfaces::srv::InfieldCorrectionCapture::Response::STATUS_DETECTION_FAILED);
  ASSERT_EQ(capture->number_of_captures, 0);
  ASSERT_EQ(
    capture->message,
    "Invalid detection. This measurement will not be used. Feedback: Failed to detect calibration "
    "object. Detection failed because no fiducial markers corresponding to a Zivid calibration "
    "board were found. Ensure the calibration board and its fiducial marker are both fully "
    "visible.");
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionCaptureOk)
{
  AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);
  all_capture_topics_subscriber.assert_num_topics_received(0);

  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture", capture_service_timeout);
  all_capture_topics_subscriber.assert_num_topics_received(1);
  ASSERT_TRUE(capture->success);
  ASSERT_EQ(capture->status, zivid_interfaces::srv::InfieldCorrectionCapture::Response::STATUS_OK);
  ASSERT_EQ(capture->number_of_captures, 1);
}

TEST_F(TestWithCalibrationBoard, testInfieldCorrectionStart)
{
  auto response = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(response);
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionStartRequiredBeforeCapture)
{
  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture", capture_service_timeout);
  ASSERT_FALSE(capture->success);
  ASSERT_EQ(
    capture->status, zivid_interfaces::srv::InfieldCorrectionCapture::Response::STATUS_NOT_SET);
  ASSERT_EQ(capture->number_of_captures, 0);
  ASSERT_EQ(
    capture->message,
    "Infield correction not started. Please call the '/infield_correction/start' service first.");

  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture", capture_service_timeout);
  ASSERT_TRUE(capture->success);
  ASSERT_EQ(capture->status, zivid_interfaces::srv::InfieldCorrectionCapture::Response::STATUS_OK);
  ASSERT_EQ(capture->number_of_captures, 1);
  ASSERT_PRED2(
    containsSubstring, capture->message,
    "Valid detection. Collected 1 valid measurement(s) so far. Measured positions:");
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionRestart)
{
  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture", capture_service_timeout);
  ASSERT_TRUE(capture->success);

  auto compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_EQ(compute->number_of_captures, 1);

  start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_EQ(compute->number_of_captures, 0);
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionCompute)
{
  auto compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_FALSE(compute->success);
  ASSERT_EQ(compute->infield_correction_started, false);
  ASSERT_EQ(compute->number_of_captures, 0);
  ASSERT_EQ(
    compute->message,
    "Infield correction not started. Please call the '/infield_correction/start' service first.");

  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_FALSE(compute->success);
  ASSERT_EQ(compute->infield_correction_started, true);
  ASSERT_EQ(compute->number_of_captures, 0);
  ASSERT_TRUE(compute->trueness_errors.empty());
  ASSERT_EQ(compute->average_trueness_error, 0);
  ASSERT_EQ(compute->maximum_trueness_error, 0);
  ASSERT_EQ(compute->dimension_accuracy, 0);
  ASSERT_EQ(compute->z_min, 0);
  ASSERT_EQ(compute->z_max, 0);
  ASSERT_EQ(
    compute->message, "Cannot compute in-field correction with empty data set. { dataset: {  } }");

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture", capture_service_timeout);
  ASSERT_TRUE(capture->success);

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  verifyInfieldComputeSuccess(compute, 1);
  constexpr float margin = 0.0001;
  ASSERT_EQ(compute->infield_correction_started, true);
  ASSERT_NEAR(compute->trueness_errors.at(0), 0.000469684601, margin);
  ASSERT_NEAR(compute->average_trueness_error, 0.000469684601, margin);
  ASSERT_NEAR(compute->maximum_trueness_error, 0.000469684601, margin);
  ASSERT_NEAR(compute->dimension_accuracy, 0.00212132023, margin);
  ASSERT_NEAR(compute->z_min, 1.03170741, margin);
  ASSERT_NEAR(compute->z_max, 1.13170743, margin);

  capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_TRUE(capture->success);

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  verifyInfieldComputeSuccess(compute, 2);
  ASSERT_NEAR(compute->trueness_errors.at(0), 0.000469684601, margin);
  ASSERT_NEAR(compute->trueness_errors.at(1), 0.000469684601, margin);
  ASSERT_NEAR(compute->average_trueness_error, 0.000469684601, margin);
  ASSERT_NEAR(compute->maximum_trueness_error, 0.000469684601, margin);
  ASSERT_NEAR(compute->dimension_accuracy, 0.00183711736, margin);
  ASSERT_NEAR(compute->z_min, 1.03170741, margin);
  ASSERT_NEAR(compute->z_max, 1.13170743, margin);

  auto compute_and_write = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute_and_write");
  verifyInfieldComputeSuccess(compute_and_write, 2, true);
  ASSERT_EQ(compute_and_write->trueness_errors.at(0), compute->trueness_errors.at(0));
  ASSERT_EQ(compute_and_write->trueness_errors.at(1), compute->trueness_errors.at(1));
  ASSERT_EQ(compute_and_write->average_trueness_error, compute->average_trueness_error);
  ASSERT_EQ(compute_and_write->maximum_trueness_error, compute->maximum_trueness_error);
  ASSERT_EQ(compute_and_write->dimension_accuracy, compute->dimension_accuracy);
  ASSERT_EQ(compute_and_write->z_min, compute->z_min);
  ASSERT_EQ(compute_and_write->z_max, compute->z_max);
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionReadWriteResetCycle)
{
  auto read =
    doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionRead>("infield_correction/read");
  ASSERT_TRUE(read->success);
  ASSERT_FALSE(read->has_camera_correction);
  ASSERT_EQ(read->camera_correction_timestamp.sec, 0);
  ASSERT_EQ(read->camera_correction_timestamp.nanosec, 0);
  ASSERT_EQ(read->message, "This camera has no in-field correction written to it.");

  infieldStartCaptureWrite();

  read = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionRead>("infield_correction/read");
  ASSERT_TRUE(read->success);
  ASSERT_TRUE(read->has_camera_correction);
  ASSERT_GT(read->camera_correction_timestamp.sec, 0);
  ASSERT_EQ(read->camera_correction_timestamp.nanosec, 0);
  verifyYearsOfTimeSinceEpoch(read->camera_correction_timestamp);
  ASSERT_PRED2(containsSubstring, read->message, "Timestamp of the current camera correction: ");

  auto reset = doStdSrvsTriggerRequest("infield_correction/reset");
  ASSERT_TRUE(reset->success);
  ASSERT_EQ(reset->message, "");

  read = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionRead>("infield_correction/read");
  ASSERT_TRUE(read->success);
  ASSERT_FALSE(read->has_camera_correction);
  ASSERT_EQ(read->camera_correction_timestamp.sec, 0);
  ASSERT_EQ(read->camera_correction_timestamp.nanosec, 0);
  ASSERT_EQ(read->message, "This camera has no in-field correction written to it.");
}

TEST_F(TestWithCalibrationBoard, testInfieldCorrectionWriteThenCaptureOperations)
{
  infieldStartCaptureWrite();

  const std::string message =
    "A new infield correction has been written to the camera. The infield correction session needs "
    "to be restarted before proceeding. This can be done by calling the "
    "'/infield_correction/start' service";

  verifyTriggerResponseError(
    doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
      "infield_correction/capture", capture_service_timeout),
    message);

  verifyTriggerResponseError(
    doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
      "infield_correction/compute"),
    message);

  verifyTriggerResponseError(
    doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
      "infield_correction/compute_and_write"),
    message);

  verifyTriggerResponseError(
    doStdSrvsTriggerRequest("infield_correction/remove_last_capture"), message);
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionRemoveLastCapture)
{
  auto remove_last_capture = doStdSrvsTriggerRequest("infield_correction/remove_last_capture");
  ASSERT_FALSE(remove_last_capture->success);
  ASSERT_EQ(
    remove_last_capture->message,
    "Infield correction not started. Please call the '/infield_correction/start' service first.");

  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  ASSERT_TRUE(start->success);

  remove_last_capture = doStdSrvsTriggerRequest("infield_correction/remove_last_capture");
  ASSERT_FALSE(remove_last_capture->success);
  ASSERT_EQ(remove_last_capture->message, "Infield correction dataset is empty");

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture", capture_service_timeout);
  ASSERT_TRUE(capture->success);

  auto compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  verifyInfieldComputeSuccess(compute, 1);

  remove_last_capture = doStdSrvsTriggerRequest("infield_correction/remove_last_capture");
  ASSERT_TRUE(remove_last_capture->success);
  ASSERT_EQ(remove_last_capture->message, "");

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_FALSE(compute->success);
  ASSERT_EQ(compute->infield_correction_started, true);
  ASSERT_EQ(compute->number_of_captures, 0);
}

TEST_F(TestWithCalibrationBoard, testInfieldCorrectionCapturePublish)
{
  auto points_sub = subscribe<sensor_msgs::msg::PointCloud2>(points_xyzrgba_topic_name);
  auto color_image_sub = subscribe<sensor_msgs::msg::Image>(color_image_color_topic_name);
  auto depth_image_sub = subscribe<sensor_msgs::msg::Image>(depth_image_topic_name);
  auto snr_image_sub = subscribe<sensor_msgs::msg::Image>(snr_image_topic_name);
  auto normals_sub = subscribe<sensor_msgs::msg::PointCloud2>(normals_xyz_topic_name);

  infieldStartCapture();

  {
    const auto last_pc2 = points_sub.lastMessage();
    ASSERT_TRUE(last_pc2);
    assertSensorMsgsPointCloud2Meta(*last_pc2, 1944U, 1200U, 16U);
    ASSERT_EQ(last_pc2->fields.size(), 4U);
    assertPointCloud2Field(last_pc2->fields[0], "x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
    assertPointCloud2Field(last_pc2->fields[1], "y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
    assertPointCloud2Field(last_pc2->fields[2], "z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
    assertPointCloud2Field(
      last_pc2->fields[3], "rgba", 12, sensor_msgs::msg::PointField::UINT32, 1);
  }

  {
    const auto image = color_image_sub.lastMessage();
    ASSERT_TRUE(image);
    const std::size_t bytes_per_pixel = 4U;
    assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "rgba8");
  }

  {
    const auto image = depth_image_sub.lastMessage();
    ASSERT_TRUE(image);
    const std::size_t bytes_per_pixel = 4U;
    assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "32FC1");
  }

  {
    const auto image = snr_image_sub.lastMessage();
    ASSERT_TRUE(image);
    const std::size_t bytes_per_pixel = 4U;
    assertSensorMsgsImageMeta(*image, 1944U, 1200U, bytes_per_pixel, "32FC1");
  }

  {
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
  }
}
