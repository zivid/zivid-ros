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

#include <Zivid/Calibration/Detector.h>
#include <Zivid/Camera.h>
#include <Zivid/Experimental/Calibration.h>

#include <cstdint>
#include <numeric>
#include <sstream>
#include <zivid_camera/capture_settings_controller.hpp>
#include <zivid_camera/detector_controller.hpp>
#include <zivid_camera/utility.hpp>

namespace zivid_camera
{
DetectorController::DetectorController(
  rclcpp::Node & zivid_camera_node, Zivid::Camera & camera,
  CaptureSettingsController<Zivid::Settings> & settings_controller,
  ControllerInterface controller_interface)
: node_{zivid_camera_node},
  camera_{camera},
  settings_controller_{settings_controller},
  controller_interface_{controller_interface}
{
  using namespace std::placeholders;

  detect_calibration_board_service_ =
    node_.create_service<zivid_interfaces::srv::CaptureAndDetectCalibrationBoard>(
      "capture_and_detect_calibration_board",
      std::bind(&DetectorController::detectCalibrationBoardHandler, this, _1, _2, _3));
  detect_markers_service_ = node_.create_service<zivid_interfaces::srv::CaptureAndDetectMarkers>(
    "capture_and_detect_markers",
    std::bind(&DetectorController::detectMarkersHandler, this, _1, _2, _3));
}

DetectorController::~DetectorController() = default;

void DetectorController::detectCalibrationBoardHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<
    zivid_interfaces::srv::CaptureAndDetectCalibrationBoard::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::CaptureAndDetectCalibrationBoard::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      auto frame = Zivid::Calibration::captureCalibrationBoard(camera_);
      ensureIdentityOrThrow(frame.pointCloud().transformationMatrix());
      auto detection = Zivid::Calibration::detectCalibrationBoard(frame);
      response->detection_result = toZividMsgDetectionResult(detection);
      if (
        response->detection_result.status !=
        zivid_interfaces::msg::DetectionResultCalibrationBoard::STATUS_OK) {
        response->success = false;
        response->message =
          "Failed to detect calibration board. " + response->detection_result.status_description;
      }
      // Publishing the frame also modifies (transforms) the point cloud in-place to scale it. The
      // above computations assume a non-transformed point cloud, so publish the frame at the end.
      controller_interface_.publishFrame(frame);
    },
    response, node_.get_logger(), "detectCalibrationBoard");
}

void DetectorController::detectMarkersHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::CaptureAndDetectMarkers::Request> request,
  std::shared_ptr<zivid_interfaces::srv::CaptureAndDetectMarkers::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      const auto settings = settings_controller_.currentSettings();
      RCLCPP_INFO(
        node_.get_logger(), "Capturing with %zd acquisition(s)", settings.acquisitions().size());
      RCLCPP_DEBUG_STREAM(node_.get_logger(), settings);
      const auto frame = camera_.capture(settings);
      ensureIdentityOrThrow(frame.pointCloud().transformationMatrix());
      const auto detection = Zivid::Calibration::detectMarkers(
        frame, request->marker_ids,
        Zivid::Calibration::MarkerDictionary::fromString(request->marker_dictionary));
      response->detection_result = toZividMsgDetectionResult(detection);
      response->success = detection.valid();
      if (!response->success) {
        response->message = "Failed to detect any fiducial markers.";
      }
      // Publishing the frame also modifies (transforms) the point cloud in-place to scale it. The
      // above computations assume a non-transformed point cloud, so publish the frame at the end.
      controller_interface_.publishFrame(frame);
    },
    response, node_.get_logger(), "detectMarkers");
}
}  // namespace zivid_camera
