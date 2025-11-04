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

#include <cstdint>
#include <numeric>
#include <sstream>
#include <zivid_camera/infield_correction_controller.hpp>
#include <zivid_camera/utility.hpp>
#include <zivid_camera/zivid_camera.hpp>

namespace zivid_camera
{
namespace
{
std::string infieldCorrectionDistancesToString(
  const std::vector<Calibration::InfieldCorrectionInput> & dataset)
{
  auto zValues = std::vector<double>(dataset.size());
  std::transform(dataset.begin(), dataset.end(), zValues.begin(), [](const auto & input) {
    return zividLengthToRos(input.detectionResult().centroid().z);
  });
  std::sort(zValues.begin(), zValues.end());

  std::stringstream ss;
  ss << "Measured positions: [";
  for (size_t i = 0; i < zValues.size(); ++i) {
    ss << std::setprecision(3) << std::fixed << zValues.at(i) << " m";
    if (i < zValues.size() - 1) {
      ss << ", ";
    }
  }
  ss << "].";

  if (!zValues.empty()) {
    ss << " Range covered: " << zValues.back() - zValues.front() << " m.";
  }

  return ss.str();
}

struct InfieldCorrectionStatistics
{
  size_t numberOfCaptures;
  std::vector<float> truenessErrorList;
  float currentTruenessError;
  float averageTruenessError;
  float maximumTruenessError;
};

InfieldCorrectionStatistics calculateInfieldCorrectionStatistics(
  const std::vector<Calibration::InfieldCorrectionInput> & dataset)
{
  if (dataset.empty()) {
    throw std::runtime_error("Empty dataset");
  }
  const auto dimensionTruenessErrors = [&] {
    std::vector<float> result;
    std::transform(
      dataset.begin(), dataset.end(), std::back_inserter(result),
      [](const auto & input) { return Calibration::verifyCamera(input).localDimensionTrueness(); });
    return result;
  }();
  const float currentTruenessError = dimensionTruenessErrors.back();
  const float averageTruenessError =
    std::accumulate(cbegin(dimensionTruenessErrors), cend(dimensionTruenessErrors), 0.0F) /
    static_cast<float>(size(dimensionTruenessErrors));
  const float maximumTruenessError =
    *std::max_element(dimensionTruenessErrors.begin(), dimensionTruenessErrors.end());
  return InfieldCorrectionStatistics{
    dataset.size(), dimensionTruenessErrors, currentTruenessError, averageTruenessError,
    maximumTruenessError};
}

std::string infieldCorrectionEstimateToString(
  const InfieldCorrectionStatistics & statistics,
  const Calibration::AccuracyEstimate & accuracyEstimate)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "Number of captures: " << statistics.numberOfCaptures << "\n";
  ss << "Camera metrics:";
  ss << "\n  Current trueness error: " << 100.0F * statistics.currentTruenessError << "%.";
  if (statistics.numberOfCaptures > 1) {
    ss << "\n  Average trueness error: " << 100.0F * statistics.averageTruenessError << "%.";
    ss << "\n  Maximum trueness error: " << 100.0F * statistics.maximumTruenessError << "%.";
  }
  ss << "\n";
  ss << "Expected post-correction metrics:";
  ss << "\n  Dimension trueness error: " << 100.0F * accuracyEstimate.dimensionAccuracy()
     << "% or less.";
  ss << "\n  Optimized workspace (depth): " << zividLengthToRos(accuracyEstimate.zMin()) << " - "
     << zividLengthToRos(accuracyEstimate.zMax()) << " m.";
  return ss.str();
}
}  // namespace

InfieldCorrectionController::InfieldCorrectionController(
  rclcpp::Node & zivid_camera_node, Zivid::Camera & camera,
  ControllerInterface controller_interface)
: node_{zivid_camera_node}, camera_{camera}, controller_interface_{controller_interface}
{
  using namespace std::placeholders;

  state_ = std::make_unique<InfieldCorrectionState>();
  read_service_ = node_.create_service<zivid_interfaces::srv::InfieldCorrectionRead>(
    "infield_correction/read",
    std::bind(&InfieldCorrectionController::readServiceHandler, this, _1, _2, _3));
  reset_service_ = node_.create_service<std_srvs::srv::Trigger>(
    "infield_correction/reset",
    std::bind(&InfieldCorrectionController::resetServiceHandler, this, _1, _2, _3));
  remove_last_capture_service_ = node_.create_service<std_srvs::srv::Trigger>(
    "infield_correction/remove_last_capture",
    std::bind(&InfieldCorrectionController::removeLastCaptureServiceHandler, this, _1, _2, _3));
  start_service_ = node_.create_service<std_srvs::srv::Trigger>(
    "infield_correction/start",
    std::bind(&InfieldCorrectionController::startServiceHandler, this, _1, _2, _3));
  capture_service_ = node_.create_service<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture",
    std::bind(&InfieldCorrectionController::captureServiceHandler, this, _1, _2, _3));
  compute_service_ = node_.create_service<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute",
    std::bind(&InfieldCorrectionController::computeServiceHandler, this, _1, _2, _3));
  compute_and_write_service_ =
    node_.create_service<zivid_interfaces::srv::InfieldCorrectionCompute>(
      "infield_correction/compute_and_write",
      std::bind(&InfieldCorrectionController::computeAndWriteServiceHandler, this, _1, _2, _3));
}

InfieldCorrectionController::~InfieldCorrectionController() = default;

void InfieldCorrectionController::readServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionRead::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionRead::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      if (Calibration::hasCameraCorrection(camera_)) {
        const auto timestamp = Calibration::cameraCorrectionTimestamp(camera_);
        const auto time = std::chrono::system_clock::to_time_t(timestamp);
        std::stringstream ss;
        ss << "Timestamp of the current camera correction: "
           << std::put_time(std::gmtime(&time), "%FT%TZ");
        response->message = ss.str();
        response->has_camera_correction = true;
        response->camera_correction_timestamp.sec = static_cast<int32_t>(time);
        response->camera_correction_timestamp.nanosec = 0;
      } else {
        response->has_camera_correction = false;
        response->message = "This camera has no in-field correction written to it.";
      }
    },
    response, node_.get_logger(), "InfieldCorrectionRead");
}

void InfieldCorrectionController::resetServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() { Calibration::resetCameraCorrection(camera_); }, response, node_.get_logger(),
    "InfieldCorrectionReset");
}

void InfieldCorrectionController::removeLastCaptureServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      ensureStarted();
      if (state_->dataset.empty()) {
        throw std::runtime_error("Infield correction dataset is empty");
      }
      state_->dataset.pop_back();
    },
    response, node_.get_logger(), "InfieldCorrectionRemoveLastCapture");
}

void InfieldCorrectionController::startServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      *state_ = {};
      state_->state = InfieldCorrectionState::State::Started;
    },
    response, node_.get_logger(), "InfieldCorrectionStart");
}

void InfieldCorrectionController::captureServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCapture::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCapture::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      auto & dataset = state_->dataset;
      response->number_of_captures = safeCast<int>(dataset.size());
      ensureStarted();
      const auto frame = Calibration::captureCalibrationBoard(camera_);
      ensureIdentityOrThrow(frame.pointCloud().transformationMatrix());
      const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);
      const auto input = Calibration::InfieldCorrectionInput{detectionResult};

      using Response = zivid_interfaces::srv::InfieldCorrectionCapture::Response;
      using Calibration::InfieldCorrectionDetectionStatus;
      switch (input.status()) {
        case InfieldCorrectionDetectionStatus::ok:
          response->status = Response::STATUS_OK;
          break;
        case InfieldCorrectionDetectionStatus::detectionFailed:
          response->status = Response::STATUS_DETECTION_FAILED;
          break;
#if defined ZIVID_EXPERIMENTAL_INFIELD
        case InfieldCorrectionDetectionStatus::invalidCaptureMethod:
          response->status = Response::STATUS_INVALID_CAPTURE_METHOD;
          break;
#endif
        case InfieldCorrectionDetectionStatus::invalidAlignment:
          response->status = Response::STATUS_INVALID_ALIGNMENT;
          break;
        default:
          throw std::runtime_error(
            "Unhandled status value: " + std::to_string(static_cast<int>(input.status())));
      }

      response->detection_result = toZividMsgDetectionResult(detectionResult);

      if (input.valid()) {
        dataset.push_back(input);
        response->number_of_captures = safeCast<int>(dataset.size());
        response->message = "Valid detection. Collected " + std::to_string(dataset.size()) +
                            " valid measurement(s) so far. " +
                            infieldCorrectionDistancesToString(dataset);

      } else {
        response->success = false;
        response->message = "Invalid detection. This measurement will not be used. Feedback: " +
                            input.statusDescription();
      }

      // Publishing the frame also modifies (transforms) the point cloud in-place to scale it. The
      // above computations assume a non-transformed point cloud, so publish the frame at the end.
      controller_interface_.publishFrame(frame);
    },
    response, node_.get_logger(), "InfieldCorrectionCapture");
}

void InfieldCorrectionController::computeServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      const auto & dataset = state_->dataset;
      // Set started & number of captures before computing, so that it is reported regardless of any exceptions.
      response->infield_correction_started =
        (state_->state == InfieldCorrectionState::State::Started);
      response->number_of_captures = safeCast<int>(dataset.size());
      ensureStarted();
      const auto correction = Calibration::computeCameraCorrection(dataset);
      const auto accuracyEstimate = correction.accuracyEstimate();
      const auto statistics = calculateInfieldCorrectionStatistics(dataset);
      response->trueness_errors = statistics.truenessErrorList;
      response->average_trueness_error = statistics.averageTruenessError;
      response->maximum_trueness_error = statistics.maximumTruenessError;
      response->z_min = static_cast<float>(zividLengthToRos(accuracyEstimate.zMin()));
      response->z_max = static_cast<float>(zividLengthToRos(accuracyEstimate.zMax()));
      response->dimension_accuracy = accuracyEstimate.dimensionAccuracy();
      response->message = "Camera correction computed successfully.\n" +
                          infieldCorrectionEstimateToString(statistics, accuracyEstimate);
    },
    response, node_.get_logger(), "InfieldCorrectionCompute");
}

void InfieldCorrectionController::computeAndWriteServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      auto & dataset = state_->dataset;
      response->infield_correction_started =
        (state_->state == InfieldCorrectionState::State::Started);
      response->number_of_captures = safeCast<int>(dataset.size());
      ensureStarted();
      const auto correction = Calibration::computeCameraCorrection(dataset);
      RCLCPP_DEBUG(node_.get_logger(), "Writing correction to camera");
      Calibration::writeCameraCorrection(camera_, correction);
      const auto accuracyEstimate = correction.accuracyEstimate();
      const auto statistics = calculateInfieldCorrectionStatistics(dataset);
      response->trueness_errors = statistics.truenessErrorList;
      response->average_trueness_error = statistics.averageTruenessError;
      response->maximum_trueness_error = statistics.maximumTruenessError;
      response->z_min = static_cast<float>(zividLengthToRos(accuracyEstimate.zMin()));
      response->z_max = static_cast<float>(zividLengthToRos(accuracyEstimate.zMax()));
      response->dimension_accuracy = accuracyEstimate.dimensionAccuracy();
      response->message = "Camera correction successfully written to camera.\n" +
                          infieldCorrectionEstimateToString(statistics, accuracyEstimate);
      // Clear the infield correction session after a successful write, the captures cannot be used again.
      *state_ = {};
      state_->state = InfieldCorrectionState::State::WriteCompleted;
    },
    response, node_.get_logger(), "InfieldCorrectionComputeAndWrite");
}

void InfieldCorrectionController::ensureStarted() const
{
  switch (state_->state) {
    case InfieldCorrectionState::State::Uninitialized:
      throw std::runtime_error(
        "Infield correction not started. Please call the '" +
        std::string{start_service_->get_service_name()} + "' service first.");
    case InfieldCorrectionState::State::Started:
      break;
    case InfieldCorrectionState::State::WriteCompleted:
      throw std::runtime_error(
        "A new infield correction has been written to the camera. The infield correction session "
        "needs to be restarted before proceeding. This can be done by calling the '" +
        std::string{start_service_->get_service_name()} + "' service");
    default:
      throw std::runtime_error(
        "Internal error. Unhandled infield correction state: " +
        std::to_string(static_cast<int>(state_->state)));
  }
}

}  // namespace zivid_camera
