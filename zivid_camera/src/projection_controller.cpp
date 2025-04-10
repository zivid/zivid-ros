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

#include <Zivid/Projection/Projection.h>

#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <zivid_camera/capture_settings_controller.hpp>
#include <zivid_camera/controller_interface.hpp>
#include <zivid_camera/projection_controller.hpp>
#include <zivid_camera/utility.hpp>
#include <zivid_camera/zivid_camera.hpp>

namespace
{

Zivid::Image<Zivid::ColorBGRA> createImage(
  const zivid_interfaces::srv::ProjectionStart::Request & request,
  const Zivid::Resolution & resolution)
{
  if (!request.image_path.empty() && !request.data.empty()) {
    throw std::runtime_error(
      "Both 'image_path' and 'data' are non-empty! Please set only one of the parameters.");
  } else if (request.image_path.empty() && request.data.empty()) {
    throw std::runtime_error(
      "Both 'image_path' and 'data' are empty! Please set one of the parameters.");
  }

  if (!request.image_path.empty()) {
    return Zivid::Image<Zivid::ColorBGRA>(request.image_path);
  }

  const auto * begin = request.data.data();
#ifdef __clang__
#if __has_warning("-Wunsafe-buffer-usage")
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage"
#endif
#endif
  const auto * end = request.data.data() + request.data.size();
#ifdef __clang__
#if __has_warning("-Wunsafe-buffer-usage")
#pragma clang diagnostic pop
#endif
#endif

  return Zivid::Image<Zivid::ColorBGRA>(resolution, begin, end);
}

}  // namespace

namespace zivid_camera
{
ProjectionController::ProjectionController(
  rclcpp::Node & zivid_camera_node, Zivid::Camera & camera,
  CaptureSettingsController<Zivid::Settings2D> & settings_2d_controller,
  ControllerInterface controller_interface)
: node_{zivid_camera_node},
  camera_{camera},
  settings_2d_controller_{settings_2d_controller},
  controller_interface_{controller_interface}
{
  using namespace std::placeholders;

  state_ = std::make_unique<ProjectionState>();
  start_service_ = node_.create_service<zivid_interfaces::srv::ProjectionStart>(
    "projection/start", std::bind(&ProjectionController::startServiceHandler, this, _1, _2, _3));
  stop_service_ = node_.create_service<std_srvs::srv::Trigger>(
    "projection/stop", std::bind(&ProjectionController::stopServiceHandler, this, _1, _2, _3));
  resolution_service_ = node_.create_service<zivid_interfaces::srv::ProjectionResolution>(
    "projection/resolution",
    std::bind(&ProjectionController::resolutionServiceHandler, this, _1, _2, _3));
  status_service_ = node_.create_service<zivid_interfaces::srv::ProjectionStatus>(
    "projection/status", std::bind(&ProjectionController::statusServiceHandler, this, _1, _2, _3));
  capture_2d_service_ = node_.create_service<std_srvs::srv::Trigger>(
    "projection/capture_2d",
    std::bind(&ProjectionController::capture2DServiceHandler, this, _1, _2, _3));
}

ProjectionController::~ProjectionController() = default;

void ProjectionController::capture2DServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      RCLCPP_INFO_STREAM(node_.get_logger(), "Querying projection");
      if (state_->projected_image) {
        const auto s = settings_2d_controller_.currentSettings();
        const auto frame2D = state_->projected_image->capture2D(s);
        controller_interface_.publishFrame2D(frame2D);
      } else {
        throw std::runtime_error("Camera is not projecting");
      }
      RCLCPP_INFO_STREAM(node_.get_logger(), "Query done");
    },
    response, node_.get_logger(), "ProjectionStatus");
}

void ProjectionController::resolutionServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::ProjectionResolution::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::ProjectionResolution::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      RCLCPP_INFO_STREAM(node_.get_logger(), "Querying projector resolution");
      const auto r = Zivid::Projection::projectorResolution(camera_);
      response->width = r.width();
      response->height = r.height();

      RCLCPP_INFO_STREAM(node_.get_logger(), "Query done");
    },
    response, node_.get_logger(), "ProjectionResolution");
}

void ProjectionController::startServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::ProjectionStart::Request> request,
  std::shared_ptr<zivid_interfaces::srv::ProjectionStart::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      *state_ = {};

      RCLCPP_INFO_STREAM(node_.get_logger(), "Starting projection");

      const auto res = Zivid::Projection::projectorResolution(camera_);
      const auto image = createImage(*request, res);

      state_->projected_image = Zivid::Projection::showImage(camera_, image);
    },
    response, node_.get_logger(), "ProjectionStart");
}

void ProjectionController::stopServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      RCLCPP_INFO_STREAM(node_.get_logger(), "Stopping projection");

      *state_ = {};

      RCLCPP_INFO_STREAM(node_.get_logger(), "Stopped projection");
    },
    response, node_.get_logger(), "ProjectionStop");
}

void ProjectionController::statusServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::ProjectionStatus::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::ProjectionStatus::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      RCLCPP_INFO_STREAM(node_.get_logger(), "Querying projection");

      response->projecting =
        state_->projected_image.has_value() && state_->projected_image->active();
      if (!response->projecting) {
        state_->projected_image.reset();
      }

      RCLCPP_INFO_STREAM(node_.get_logger(), "Query done");
    },
    response, node_.get_logger(), "ProjectionStatus");
}

}  // namespace zivid_camera
