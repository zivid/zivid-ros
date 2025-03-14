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

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <zivid_camera/utility.hpp>

namespace Zivid
{
class Settings;
class Settings2D;
}  // namespace Zivid

namespace zivid_camera
{
template <typename SettingsType>
class CaptureSettingsController
{
public:
  explicit CaptureSettingsController(rclcpp::Node & node)
  : node_(node),
    file_path_param_{baseName() + std::string{"_file_path"}},
    yaml_string_param_{baseName() + std::string{"_yaml"}}
  {
    for (const auto & param : {yaml_string_param_, file_path_param_}) {
      node_.declare_parameter<std::string>(param, "");
    }
  }

  SettingsType currentSettings() const
  {
    if (cached_settings_.has_value()) {
      RCLCPP_DEBUG_STREAM(node_.get_logger(), "Using cached settings");
      return *cached_settings_;
    }

    const auto settings_file_path = node_.get_parameter(file_path_param_).as_string();
    const auto settings_yaml = node_.get_parameter(yaml_string_param_).as_string();

    if (!settings_file_path.empty() && !settings_yaml.empty()) {
      logErrorToLoggerAndThrowRuntimeException(
        node_.get_logger(),
        "Both '" + file_path_param_ + "' and '" + yaml_string_param_ +
          "' parameters are non-empty! Please set only one of these parameters.");
    } else if (settings_file_path.empty() && settings_yaml.empty()) {
      logErrorToLoggerAndThrowRuntimeException(
        node_.get_logger(), "Both '" + file_path_param_ + "' and '" + yaml_string_param_ +
                              "' parameters are empty! Please set one of these parameters.");
    }

    if (!settings_yaml.empty()) {
      RCLCPP_DEBUG_STREAM(node_.get_logger(), "Using settings from yml string");
      cached_settings_ = deserializeZividDataModel<SettingsType>(settings_yaml);
    } else {
      RCLCPP_DEBUG_STREAM(
        node_.get_logger(), "Using settings from file '" << settings_file_path << "'");
      cached_settings_ = SettingsType{settings_file_path};
    }

    return *cached_settings_;
  }

  void setSettings(const SettingsType & settings)
  {
    RCLCPP_DEBUG_STREAM(node_.get_logger(), "Setting settings from " << settings.name << " object");
    node_.set_parameter(rclcpp::Parameter{yaml_string_param_, serializeZividDataModel(settings)});
    node_.set_parameter(rclcpp::Parameter{file_path_param_, ""});
  }

  void onSetParameter(const std::string & parameterName)
  {
    if (
      cached_settings_.has_value() &&
      (parameterName == file_path_param_ || parameterName == yaml_string_param_)) {
      RCLCPP_DEBUG_STREAM(
        node_.get_logger(), "Resetting cached settings due to updated parameter " << parameterName);
      cached_settings_.reset();
    }
  }

private:
  constexpr auto baseName() const
  {
    if constexpr (std::is_same_v<SettingsType, Zivid::Settings>) {
      return "settings";
    } else if constexpr (std::is_same_v<SettingsType, Zivid::Settings2D>) {
      return "settings_2d";
    } else {
      static_assert(DependentFalse<SettingsType>::value, "Unhandled node type");
    }
  }

  rclcpp::Node & node_;
  std::string file_path_param_;
  std::string yaml_string_param_;
  mutable std::optional<SettingsType> cached_settings_;
};

}  // namespace zivid_camera
