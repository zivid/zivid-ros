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
#include <filesystem>
#include <std_srvs/srv/trigger.hpp>
#include <test_zivid_camera.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_calibrate.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_capture.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_load.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_start.hpp>

template <typename T>
struct DependentFalse : std::false_type
{
};

class TestHandEyeWithCalibrationBoard : public ZividNodeTest
{
protected:
  TestHandEyeWithCalibrationBoard(NodeReusePolicy reusePolicy = NodeReusePolicy::AllowReuse)
  : ZividNodeTest{FileCameraMode::CalibrationBoard, reusePolicy}
  {
  }

  void startWithCalibrationBoard()
  {
    setSingleDefaultAcquisitionSettingsUsingYml();
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationStart::Request>();
    request->calibration_objects.type =
      zivid_interfaces::msg::HandEyeCalibrationObjects::CALIBRATION_BOARD;
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationStart>(
      "hand_eye_calibration/start", request);
    verifyTriggerResponseSuccess(response);
  }

  void startWithMarkers(const std::vector<int> & marker_ids, const std::string & marker_dictionary)
  {
    setSingleDefaultAcquisitionSettingsUsingYml();
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationStart::Request>();
    request->calibration_objects.type =
      zivid_interfaces::msg::HandEyeCalibrationObjects::FIDUCIAL_MARKERS;
    request->calibration_objects.marker_ids = marker_ids;
    request->calibration_objects.marker_dictionary = marker_dictionary;
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationStart>(
      "hand_eye_calibration/start", request);
    verifyTriggerResponseSuccess(response);
  }

  decltype(auto) handEyeCapture(const std::array<double, 3> & posePosition)
  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCapture::Request>();
    request->robot_pose.position.x = posePosition.at(0);
    request->robot_pose.position.y = posePosition.at(1);
    request->robot_pose.position.z = posePosition.at(2);
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCapture>(
      "hand_eye_calibration/capture", request, capture_service_timeout);
    return response;
  }

  decltype(auto) handEyeCalibrate(int configuration)
  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration = configuration;
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    return response;
  }

  template <typename ServiceType>
  void commonLoadAndStartInvalidTests()
  {
    using ServiceTypeRequest = typename ServiceType::Request;

    std::string service_name;
    if constexpr (std::is_same_v<ServiceType, zivid_interfaces::srv::HandEyeCalibrationStart>) {
      service_name = "hand_eye_calibration/start";
    } else if constexpr (std::is_same_v<
                           ServiceType, zivid_interfaces::srv::HandEyeCalibrationLoad>) {
      service_name = "hand_eye_calibration/load";
    } else {
      static_assert(DependentFalse<ServiceType>::value, "Unhandled service type");
    }

    {
      auto request = std::make_shared<ServiceTypeRequest>();
      auto response = doSrvRequest<ServiceType>(service_name, request);
      verifyTriggerResponseError(response, "Invalid calibration object type: 0");
    }

    {
      auto request = std::make_shared<ServiceTypeRequest>();
      request->calibration_objects.type =
        zivid_interfaces::msg::HandEyeCalibrationObjects::FIDUCIAL_MARKERS;
      auto response = doSrvRequest<ServiceType>(service_name, request);
      verifyTriggerResponseError(response);
      ASSERT_PRED2(
        containsSubstring, response->message,
        "Invalid dictionary name \"\". Expected one of the following values:");
    }

    {
      auto request = std::make_shared<ServiceTypeRequest>();
      request->calibration_objects.type =
        zivid_interfaces::msg::HandEyeCalibrationObjects::FIDUCIAL_MARKERS;
      request->calibration_objects.marker_dictionary = "aruco4x4_50";
      auto response = doSrvRequest<ServiceType>(service_name, request);
      verifyTriggerResponseError(response, "The list of marker IDs cannot be empty");
    }

    {
      auto request = std::make_shared<ServiceTypeRequest>();
      request->calibration_objects.type =
        zivid_interfaces::msg::HandEyeCalibrationObjects::CALIBRATION_BOARD;
      request->working_directory = "relative_path";
      auto response = doSrvRequest<ServiceType>(service_name, request);
      verifyTriggerResponseError(response, "Expected an absolute path but got: relative_path");
    }

    {
      auto request = std::make_shared<ServiceTypeRequest>();
      request->calibration_objects.type =
        zivid_interfaces::msg::HandEyeCalibrationObjects::CALIBRATION_BOARD;
      request->working_directory = "/some/invalid/absolute/path";
      auto response = doSrvRequest<ServiceType>(service_name, request);
      verifyTriggerResponseError(response);
    }
  }
};

class TestHandEyeWithCalibrationBoardFreshNode : public TestHandEyeWithCalibrationBoard
{
protected:
  TestHandEyeWithCalibrationBoardFreshNode()
  : TestHandEyeWithCalibrationBoard{NodeReusePolicy::RestartNode}
  {
  }
};

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeStartCalibrationBoardSuccess)
{
  startWithCalibrationBoard();
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeStartWorkingDirectorySuccess)
{
  const auto path =
    std::filesystem::temp_directory_path() / "zivid_camera_test_empty_hand_eye_working_directory";
  auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationStart::Request>();
  request->calibration_objects.type =
    zivid_interfaces::msg::HandEyeCalibrationObjects::CALIBRATION_BOARD;
  request->working_directory = path.string();
  auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationStart>(
    "hand_eye_calibration/start", request);
  verifyTriggerResponseSuccess(response);
  ASSERT_TRUE(std::filesystem::is_directory(path));
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeStartMarkersSuccess)
{
  startWithMarkers({1}, "aruco4x4_50");
  startWithMarkers({3, 5, 7}, "aruco5x5_250");
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeStartInvalid)
{
  commonLoadAndStartInvalidTests<zivid_interfaces::srv::HandEyeCalibrationStart>();
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeLoadInvalid)
{
  commonLoadAndStartInvalidTests<zivid_interfaces::srv::HandEyeCalibrationLoad>();

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationLoad::Request>();
    request->calibration_objects.type =
      zivid_interfaces::msg::HandEyeCalibrationObjects::CALIBRATION_BOARD;
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationLoad>(
      "hand_eye_calibration/load", request);
    verifyTriggerResponseError(response, "Provided path to working directory is empty");
  }
}

void verifyDirectoryContents(
  const std::filesystem::path & directory, std::vector<std::string> expected_files)
{
  ASSERT_TRUE(std::filesystem::is_directory(directory));
  if (expected_files.empty()) {
    ASSERT_TRUE(std::filesystem::is_empty(directory));
    return;
  }

  std::vector<std::string> directory_files;
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    ASSERT_TRUE(std::filesystem::is_regular_file(entry));
    directory_files.push_back(entry.path().filename().string());
  }

  std::sort(directory_files.begin(), directory_files.end());
  std::sort(expected_files.begin(), expected_files.end());
  ASSERT_EQ(directory_files, expected_files);
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeStartCaptureLoadCycle)
{
  setSingleDefaultAcquisitionSettingsUsingYml();
  const TmpDirectory tmp_directory{"zivid_ros_hand_eye_calibration_test"};

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationStart::Request>();
    request->calibration_objects.type =
      zivid_interfaces::msg::HandEyeCalibrationObjects::CALIBRATION_BOARD;
    request->working_directory = tmp_directory.string();
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationStart>(
      "hand_eye_calibration/start", request);
    verifyTriggerResponseSuccess(response);
  }
  verifyDirectoryContents(tmp_directory.path(), {});

  verifyTriggerResponseSuccess(handEyeCapture({}));
  std::vector<std::string> files = {
    "calibration_object_pose_0.zdf",
    "checkerboard_pose_in_camera_frame_0.yaml",
    "robot_pose_0.yaml",
  };
  verifyDirectoryContents(tmp_directory.path(), files);

  verifyTriggerResponseSuccess(handEyeCapture({1, 2, 3}));
  files.insert(
    files.end(), {
                   "calibration_object_pose_1.zdf",
                   "checkerboard_pose_in_camera_frame_1.yaml",
                   "robot_pose_1.yaml",
                 });
  verifyDirectoryContents(tmp_directory.path(), files);

  const auto original_calibration_response =
    handEyeCalibrate(zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND);
  ASSERT_TRUE(original_calibration_response->success);

  files.push_back("hand_eye_transform.yaml");
  verifyDirectoryContents(tmp_directory.path(), files);

  // Clear session and ensure captured data in the zivid driver is gone, but data remains on disk.
  startWithCalibrationBoard();
  verifyDirectoryContents(tmp_directory.path(), files);

  {
    const auto response =
      handEyeCalibrate(zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND);
    verifyTriggerResponseError(response);
    ASSERT_EQ(response->message, "No input given to hand eye calibration. { inputs: {  } }");
  }

  // Load previous working directory and verify equal calibration response.
  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationLoad::Request>();
    request->calibration_objects.type =
      zivid_interfaces::msg::HandEyeCalibrationObjects::CALIBRATION_BOARD;
    request->working_directory = tmp_directory.string();
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationLoad>(
      "hand_eye_calibration/load", request, hand_eye_calibration_load_service_timeout);
    verifyTriggerResponseSuccess(response);
  }

  const auto loaded_calibration_response =
    handEyeCalibrate(zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND);
  ASSERT_TRUE(loaded_calibration_response->success);

  ASSERT_EQ(*original_calibration_response, *loaded_calibration_response);

  // Verify read-only mode, any captures should error out.
  {
    const auto response = handEyeCapture({1, 2, 3});
    verifyTriggerResponseError(
      response,
      "Hand-eye calibration is currently in workspace read-only state, capturing is not allowed.");
    ASSERT_EQ(response->capture_handle, -1);
  }

  verifyDirectoryContents(tmp_directory.path(), files);
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCaptureSuccessCalibrationBoard)
{
  AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);
  all_capture_topics_subscriber.assert_num_topics_received(0);
  startWithCalibrationBoard();
  auto response = handEyeCapture({});
  verifyTriggerResponseSuccess(response);
  all_capture_topics_subscriber.assert_num_topics_received(1);
  ASSERT_EQ(response->capture_handle, 0);

  response = handEyeCapture({1, 2, 3});
  verifyTriggerResponseSuccess(response);
  all_capture_topics_subscriber.assert_num_topics_received(2);
  ASSERT_EQ(response->capture_handle, 1);
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCaptureSuccessMarkers)
{
  AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);
  all_capture_topics_subscriber.assert_num_topics_received(0);

  startWithMarkers({1}, "aruco4x4_50");
  auto response = handEyeCapture({});
  verifyTriggerResponseSuccess(response);
  all_capture_topics_subscriber.assert_num_topics_received(1);
  ASSERT_EQ(response->capture_handle, 0);

  response = handEyeCapture({1, 2, 3});
  verifyTriggerResponseSuccess(response);
  all_capture_topics_subscriber.assert_num_topics_received(2);
  ASSERT_EQ(response->capture_handle, 1);
}

TEST_F(ZividNodeTest, testHandEyeCaptureNoCalibrationBoard)
{
  AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);
  all_capture_topics_subscriber.assert_num_topics_received(0);

  setSingleDefaultAcquisitionSettingsUsingYml();

  auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationStart::Request>();
  request->calibration_objects.type =
    zivid_interfaces::msg::HandEyeCalibrationObjects::CALIBRATION_BOARD;
  auto start = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationStart>(
    "hand_eye_calibration/start", request);
  verifyTriggerResponseSuccess(start);

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::HandEyeCalibrationCapture>(
    "hand_eye_calibration/capture", capture_service_timeout);
  ASSERT_FALSE(capture->success);
  verifyTriggerResponseError(capture);
  all_capture_topics_subscriber.assert_num_topics_received(1);
  ASSERT_EQ(
    capture->message,
    "Could not detect calibration board: Detection failed because no fiducial markers "
    "corresponding to a Zivid calibration board were found. Ensure the calibration board and its "
    "fiducial marker are both fully visible.");
  ASSERT_EQ(capture->capture_handle, -1);
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCaptureWrongMarkerID)
{
  AllCaptureTopicsSubscriber all_capture_topics_subscriber(*this);
  all_capture_topics_subscriber.assert_num_topics_received(0);

  startWithMarkers({2}, "aruco4x4_50");
  auto response = handEyeCapture({});
  verifyTriggerResponseError(response);
  all_capture_topics_subscriber.assert_num_topics_received(1);
  ASSERT_EQ(
    response->message,
    "Could not detect fiducial markers: { Valid: false, Allowed marker ids: { 2 } }");
  ASSERT_EQ(response->capture_handle, -1);
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCapturePublish)
{
  auto points_sub = subscribe<sensor_msgs::msg::PointCloud2>(points_xyzrgba_topic_name);
  auto color_image_sub = subscribe<sensor_msgs::msg::Image>(color_image_color_topic_name);
  auto depth_image_sub = subscribe<sensor_msgs::msg::Image>(depth_image_topic_name);
  auto snr_image_sub = subscribe<sensor_msgs::msg::Image>(snr_image_topic_name);
  auto normals_sub = subscribe<sensor_msgs::msg::PointCloud2>(normals_xyz_topic_name);

  startWithCalibrationBoard();
  verifyTriggerResponseSuccess(handEyeCapture({}));

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

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateCalibrationBoardSuccess)
{
  startWithCalibrationBoard();
  verifyTriggerResponseSuccess(handEyeCapture({}));
  verifyTriggerResponseSuccess(handEyeCapture({1, 2, 3}));

  for (const int configuration :
       {zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND,
        zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_IN_HAND}) {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration = configuration;
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    ASSERT_TRUE(response->success);
    ASSERT_PRED2(
      containsSubstring, response->message,
      "Hand-eye calibration completed. { Hand-Eye transformation:");
    ASSERT_NE(response->transform.translation.x, 0);
    ASSERT_NE(response->transform.translation.y, 0);
    ASSERT_NE(response->transform.translation.z, 0);
    ASSERT_EQ(response->residuals.size(), 2);
  }
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateMarkersSuccess)
{
  startWithMarkers({1}, "aruco4x4_50");
  for (int i = 0; i < 6; i++) {
    const float x = static_cast<float>(3 * i);
    verifyTriggerResponseSuccess(handEyeCapture({x, x + 1.f, x + 2.f}));
  }

  for (const int configuration :
       {zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND,
        zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_IN_HAND}) {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration = configuration;
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    ASSERT_TRUE(response->success);
    ASSERT_PRED2(
      containsSubstring, response->message,
      "Hand-eye calibration completed. { Hand-Eye transformation:");
    ASSERT_EQ(response->residuals.size(), 6);
  }
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateInvalidConfiguration)
{
  startWithCalibrationBoard();
  verifyTriggerResponseSuccess(handEyeCapture({}));
  verifyTriggerResponseSuccess(handEyeCapture({1, 2, 3}));

  auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
  auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
    "hand_eye_calibration/calibrate", request);
  verifyTriggerResponseError(response);
  ASSERT_EQ(
    response->message,
    "Invalid hand-eye calibration configuration: 0. Valid values: EYE_TO_HAND (1), EYE_IN_HAND "
    "(2)");
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateNoCaptures)
{
  startWithCalibrationBoard();

  auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
  request->configuration = zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
  auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
    "hand_eye_calibration/calibrate", request);
  verifyTriggerResponseError(response);
  ASSERT_EQ(response->message, "No input given to hand eye calibration. { inputs: {  } }");
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateSingleCapture)
{
  startWithCalibrationBoard();
  verifyTriggerResponseSuccess(handEyeCapture({}));

  auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
  request->configuration = zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
  auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
    "hand_eye_calibration/calibrate", request);
  verifyTriggerResponseError(response);
  ASSERT_PRED2(
    containsSubstring, response->message,
    "Need at least two or more sets of inputs to perform hand eye calibration with calibration "
    "board. { inputs:");
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateDuplicateCaptures)
{
  startWithCalibrationBoard();
  verifyTriggerResponseSuccess(handEyeCapture({}));
  verifyTriggerResponseSuccess(handEyeCapture({}));

  auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
  request->configuration = zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
  auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
    "hand_eye_calibration/calibrate", request);
  verifyTriggerResponseError(response);
  ASSERT_PRED2(containsSubstring, response->message, "Poses with index [0] and [1] are identical.");
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateHandlesSuccess)
{
  startWithCalibrationBoard();
  verifyTriggerResponseSuccess(handEyeCapture({}));
  verifyTriggerResponseSuccess(handEyeCapture({}));
  verifyTriggerResponseSuccess(handEyeCapture({1, 2, 3}));
  verifyTriggerResponseSuccess(handEyeCapture({4, 5, 6}));

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    request->capture_handles = {1, 2, 3};
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    ASSERT_TRUE(response->success);
    ASSERT_PRED2(
      containsSubstring, response->message,
      "Hand-eye calibration completed. { Hand-Eye transformation:");
  }

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    request->capture_handles = {2, 3};
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    ASSERT_TRUE(response->success);
    ASSERT_PRED2(
      containsSubstring, response->message,
      "Hand-eye calibration completed. { Hand-Eye transformation:");
  }
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateHandlesInvalid)
{
  startWithCalibrationBoard();
  verifyTriggerResponseSuccess(handEyeCapture({}));
  verifyTriggerResponseSuccess(handEyeCapture({}));
  verifyTriggerResponseSuccess(handEyeCapture({1, 2, 3}));
  verifyTriggerResponseSuccess(handEyeCapture({4, 5, 6}));

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    verifyTriggerResponseError(response);
    ASSERT_PRED2(
      containsSubstring, response->message, "Poses with index [0] and [1] are identical.");
  }

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    request->capture_handles = {0};
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    verifyTriggerResponseError(response);
    ASSERT_PRED2(
      containsSubstring, response->message,
      "Need at least two or more sets of inputs to perform hand eye calibration with calibration "
      "board. { inputs:");
  }

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    request->capture_handles = {0, 1, 2};
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    verifyTriggerResponseError(response);
    ASSERT_PRED2(
      containsSubstring, response->message, "Poses with index [0] and [1] are identical.");
  }

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    request->capture_handles = {0, 2, 1};
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    verifyTriggerResponseError(response);
    ASSERT_PRED2(
      containsSubstring, response->message, "Poses with index [0] and [2] are identical.");
  }

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    request->capture_handles = {1, 2, 100};
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    verifyTriggerResponseError(response);
    ASSERT_PRED2(containsSubstring, response->message, "Invalid capture handle: 100");
  }

  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    request->capture_handles = {1, 2, -1};
    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    verifyTriggerResponseError(response);
    ASSERT_PRED2(containsSubstring, response->message, "Invalid capture handle: -1");
  }
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateLowDOFMarkers)
{
  // This test exists mainly to check the interfaces, that all the input and output fields can be
  // assigned and read as expected. The calibration input data does not make a lot of sense, so it
  // would be reasonable for the output data to change in other SDK version. Another calibration
  // result, or an unsuccessful calibration, would be just as valid here.
  constexpr double margin = 0.1;
  const std::string marker_dictionary = "aruco4x4_50";
  const std::vector<int> marker_ids = {1, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
  startWithMarkers(marker_ids, marker_dictionary);

  setSingleDefaultAcquisitionSettingsUsingYml();
  verifyTriggerResponseSuccess(handEyeCapture({}));
  verifyTriggerResponseSuccess(handEyeCapture({1, 2, 3}));
  verifyTriggerResponseSuccess(handEyeCapture({4, 5, 6}));
  verifyTriggerResponseSuccess(handEyeCapture({7, 8, 9}));
  verifyTriggerResponseSuccess(handEyeCapture({10, 11, 12}));
  verifyTriggerResponseSuccess(handEyeCapture({13, 14, 15}));

  auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
  request->configuration = zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_IN_HAND;
  request->fixed_objects.marker_dictionary = marker_dictionary;
  request->fixed_objects.type =
    zivid_interfaces::msg::FixedPlacementOfCalibrationObjects::FIDUCIAL_MARKERS;

  auto makeMarker = [](int marker_id, double x, double y, double z) {
    zivid_interfaces::msg::FixedPlacementOfFiducialMarker marker;
    constexpr double mmToMeter = 0.001;
    marker.marker_id = marker_id;
    marker.position.x = mmToMeter * x;
    marker.position.y = mmToMeter * y;
    marker.position.z = mmToMeter * z;
    return marker;
  };
  request->fixed_objects.markers = {
    makeMarker(1, -80.9, 512.5, 1.8),  makeMarker(21, -9.1, 508.8, 2.0),
    makeMarker(22, 62.5, 505.3, 2.0),  makeMarker(23, -83.2, 462.7, 1.7),
    makeMarker(24, -11.6, 459.3, 2.3), makeMarker(25, 60.2, 455.9, 1.8),
    makeMarker(26, -85.8, 413.1, 1.0), makeMarker(27, -13.9, 409.5, 1.7),
    makeMarker(28, 57.8, 405.9, 1.7),  makeMarker(29, -88.2, 363.4, 0.8),
    makeMarker(30, -16.3, 359.8, 1.1), makeMarker(31, 55.6, 356.2, 1.4),
  };

  auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
    "hand_eye_calibration/calibrate", request);
  ASSERT_TRUE(response->success);
  ASSERT_PRED2(
    containsSubstring, response->message,
    "Hand-eye calibration completed. { Hand-Eye transformation:");

  EXPECT_NEAR(response->transform.rotation.x, -0.55137087592950862, margin);
  EXPECT_NEAR(response->transform.rotation.y, 0.56498650630404534, margin);
  EXPECT_NEAR(response->transform.rotation.z, -0.028818253506047845, margin);
  EXPECT_NEAR(response->transform.rotation.w, 0.613147553977895, margin);
  EXPECT_NEAR(response->transform.translation.x, -6.66133349609375, margin);
  EXPECT_NEAR(response->transform.translation.y, -6.8641166992187497, margin);
  EXPECT_NEAR(response->transform.translation.z, -7.1228476562500003, margin);
  ASSERT_EQ(response->residuals.size(), 6);
  EXPECT_NEAR(response->residuals.at(0).rotation, 173.12001037597656, margin);
  EXPECT_NEAR(response->residuals.at(0).translation, 11.606989860534668, margin);
}

TEST_F(TestHandEyeWithCalibrationBoard, testHandEyeCalibrateLowDOFCalibrationBoard)
{
  // This test exists mainly to check the interfaces, that all the input and output fields can be
  // assigned and read as expected. The calibration input data does not make a lot of sense, so it
  // would be reasonable for the output data to change in other SDK version. Another calibration
  // result, or an unsuccessful calibration, would be just as valid here.
  constexpr double margin = 0.1;

  startWithCalibrationBoard();
  setSingleDefaultAcquisitionSettingsUsingYml();
  verifyTriggerResponseSuccess(handEyeCapture({}));
  verifyTriggerResponseSuccess(handEyeCapture({1, 2, 3}));

  // Using position
  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    request->fixed_objects.type =
      zivid_interfaces::msg::FixedPlacementOfCalibrationObjects::CALIBRATION_BOARD;

    request->fixed_objects.calibration_board.representation =
      zivid_interfaces::msg::FixedPlacementOfCalibrationBoard::POSITION;
    request->fixed_objects.calibration_board.position.x = 0;
    request->fixed_objects.calibration_board.position.y = 0;
    request->fixed_objects.calibration_board.position.z = 0;

    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    ASSERT_TRUE(response->success);
    ASSERT_PRED2(
      containsSubstring, response->message,
      "Hand-eye calibration completed. { Hand-Eye transformation:");

    EXPECT_NEAR(response->transform.rotation.x, -0.3338221307333164, margin);
    EXPECT_NEAR(response->transform.rotation.y, 0.12382566623145259, margin);
    EXPECT_NEAR(response->transform.rotation.z, 0.0287235994234704, margin);
    EXPECT_NEAR(response->transform.rotation.w, 0.9340262029926838, margin);
    EXPECT_NEAR(response->transform.translation.x, 0.20223348999023438, margin);
    EXPECT_NEAR(response->transform.translation.y, 0.40201751708984373, margin);
    EXPECT_NEAR(response->transform.translation.z, 0.61725994873046874, margin);
    ASSERT_EQ(response->residuals.size(), 2);
    EXPECT_NEAR(response->residuals.at(0).rotation, 0, margin);
    EXPECT_NEAR(response->residuals.at(0).translation, 1.8708286285400391, margin);
  }

  // Using pose
  {
    auto request = std::make_shared<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request>();
    request->configuration =
      zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND;
    request->fixed_objects.type =
      zivid_interfaces::msg::FixedPlacementOfCalibrationObjects::CALIBRATION_BOARD;

    request->fixed_objects.calibration_board.representation =
      zivid_interfaces::msg::FixedPlacementOfCalibrationBoard::POSE;
    request->fixed_objects.calibration_board.pose.position.x = 0;
    request->fixed_objects.calibration_board.pose.position.x = 0;
    request->fixed_objects.calibration_board.pose.position.x = 0;
    request->fixed_objects.calibration_board.pose.orientation.x = 0;
    request->fixed_objects.calibration_board.pose.orientation.y = 0;
    request->fixed_objects.calibration_board.pose.orientation.z = 0;
    request->fixed_objects.calibration_board.pose.orientation.w = 1;

    auto response = doSrvRequest<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
      "hand_eye_calibration/calibrate", request);
    ASSERT_TRUE(response->success);
    ASSERT_PRED2(
      containsSubstring, response->message,
      "Hand-eye calibration completed. { Hand-Eye transformation:");

    EXPECT_NEAR(response->transform.rotation.x, 0.020668384913865655, margin);
    EXPECT_NEAR(response->transform.rotation.y, -0.080071359828544894, margin);
    EXPECT_NEAR(response->transform.rotation.z, 0.012730027152341797, margin);
    EXPECT_NEAR(response->transform.rotation.w, 0.99649353232488391, margin);
    EXPECT_NEAR(response->transform.translation.x, 0.62237646484374998, margin);
    EXPECT_NEAR(response->transform.translation.y, 1.1673579101562499, margin);
    EXPECT_NEAR(response->transform.translation.z, 0.41257031249999998, margin);
    ASSERT_EQ(response->residuals.size(), 2);
    EXPECT_NEAR(response->residuals.at(0).rotation, 0, margin);
    EXPECT_NEAR(response->residuals.at(0).translation, 1.8708287477493286, margin);
  }
}
