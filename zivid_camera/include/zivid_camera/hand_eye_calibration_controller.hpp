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

#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Experimental/Calibration/InfieldCorrection.h>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>
#include <zivid_camera/controller_interface.hpp>
#include <zivid_interfaces/msg/hand_eye_calibration_objects.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_calibrate.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_capture.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_load.hpp>
#include <zivid_interfaces/srv/hand_eye_calibration_start.hpp>

namespace Zivid
{
class Camera;
class Settings;
}  // namespace Zivid

namespace zivid_camera
{
template <typename SettingsType>
class CaptureSettingsController;

struct CalibrationBoard
{
};

struct FiducialMarkers
{
  Zivid::Calibration::MarkerDictionary dictionary;
  std::vector<int> ids;
};

using HandEyeCalibrationObjects = std::variant<CalibrationBoard, FiducialMarkers>;

struct HandEyeCalibrationState
{
  enum class State
  {
    Uninitialized,
    CaptureAndCalibrate,
    ReadWorkspace,
  };

  State state = State::Uninitialized;
  std::optional<std::filesystem::path> working_directory;
  std::optional<HandEyeCalibrationObjects> calibration_objects;
  std::vector<Zivid::Calibration::HandEyeInput> input;
};

class HandEyeCalibrationController
{
public:
  HandEyeCalibrationController(
    rclcpp::Node & zivid_camera_node, Zivid::Camera & camera,
    CaptureSettingsController<Zivid::Settings> & settings_controller,
    ControllerInterface controller_interface);
  ~HandEyeCalibrationController();

private:
  void startServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationStart::Request> request,
    std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationStart::Response> response);
  void loadServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationLoad::Request> request,
    std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationLoad::Response> response);
  void captureServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationCapture::Request> request,
    std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationCapture::Response> response);
  void calibrateServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request> request,
    std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Response> response);

  rclcpp::Node & node_;
  Zivid::Camera & camera_;
  CaptureSettingsController<Zivid::Settings> & settings_controller_;
  ControllerInterface controller_interface_;

  std::unique_ptr<HandEyeCalibrationState> state_;
  rclcpp::Service<zivid_interfaces::srv::HandEyeCalibrationStart>::SharedPtr start_service_;
  rclcpp::Service<zivid_interfaces::srv::HandEyeCalibrationLoad>::SharedPtr load_service_;
  rclcpp::Service<zivid_interfaces::srv::HandEyeCalibrationCapture>::SharedPtr capture_service_;
  rclcpp::Service<zivid_interfaces::srv::HandEyeCalibrationCalibrate>::SharedPtr calibrate_service_;
};
}  // namespace zivid_camera
