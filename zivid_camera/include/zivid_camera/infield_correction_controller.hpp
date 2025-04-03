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

#include <Zivid/Experimental/Calibration/InfieldCorrection.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>
#include <zivid_camera/controller_interface.hpp>
#include <zivid_interfaces/srv/infield_correction_capture.hpp>
#include <zivid_interfaces/srv/infield_correction_compute.hpp>
#include <zivid_interfaces/srv/infield_correction_read.hpp>

namespace Zivid
{
class Camera;
}

namespace zivid_camera
{
struct InfieldCorrectionState
{
  enum class State
  {
    Uninitialized,
    Started,
    WriteCompleted,
  };

  State state = State::Uninitialized;
  std::vector<Zivid::Experimental::Calibration::InfieldCorrectionInput> dataset;
};

class InfieldCorrectionController
{
public:
  InfieldCorrectionController(
    rclcpp::Node & zivid_camera_node, Zivid::Camera & camera,
    ControllerInterface controller_interface);
  ~InfieldCorrectionController();

private:
  void readServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionRead::Request> request,
    std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionRead::Response> response);
  void resetServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void removeLastCaptureServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void startServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void captureServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCapture::Request> request,
    std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCapture::Response> response);
  void computeServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Request> request,
    std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> response);
  void computeAndWriteServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Request> request,
    std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> response);

  void ensureStarted() const;

  rclcpp::Node & node_;
  Zivid::Camera & camera_;
  ControllerInterface controller_interface_;

  std::unique_ptr<InfieldCorrectionState> state_;
  rclcpp::Service<zivid_interfaces::srv::InfieldCorrectionRead>::SharedPtr read_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_last_capture_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<zivid_interfaces::srv::InfieldCorrectionCapture>::SharedPtr capture_service_;
  rclcpp::Service<zivid_interfaces::srv::InfieldCorrectionCompute>::SharedPtr compute_service_;
  rclcpp::Service<zivid_interfaces::srv::InfieldCorrectionCompute>::SharedPtr
    compute_and_write_service_;
};
}  // namespace zivid_camera
