#pragma once

#include "auto_generated_include_wrapper.h"

#include <Zivid/CameraInfo.h>
#include <Zivid/Settings.h>
#include <Zivid/Settings2D.h>

#include <ros/node_handle.h>

namespace Zivid
{
class Camera;
}

namespace zivid_camera
{
template <typename ConfigType, typename ZividSettings>
class ConfigDRServer;

/// <summary>Controller that manages dynamic_reconfigure nodes for Settings and Settings2D</summary>
/// <remarks>
/// This is a templated class that handles Zivid::Settings and Zivid::Settings2D configuration. Note that
/// within the Zivid SDK Settings and Settings2D are similar in structure, but there are some differences
/// in what kind of settings are provided.
///
/// This controller sets up the dynamic_reconfigure nodes "settings/", "settings/acquisition_0" and so
/// on, handles callbacks and updates the internal config state. It also provides methods to convert
/// from/to Zivid::Settings/Settings2D objects.
/// </remarks>
template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
class CaptureSettingsController
{
  static_assert(std::is_same_v<ZividSettingsType, Zivid::Settings> ||
                std::is_same_v<ZividSettingsType, Zivid::Settings2D>);
  static_assert(std::is_same_v<SettingsConfigType, SettingsConfig> ||
                std::is_same_v<SettingsConfigType, Settings2DConfig>);
  static_assert(std::is_same_v<SettingsAcquisitionConfigType, SettingsAcquisitionConfig> ||
                std::is_same_v<SettingsAcquisitionConfigType, Settings2DAcquisitionConfig>);

public:
  CaptureSettingsController(ros::NodeHandle& nh, Zivid::Camera& camera, const std::string& config_node_name,
                            std::size_t num_acquisition_servers);
  ~CaptureSettingsController();
  ZividSettingsType zividSettings() const;
  void setZividSettings(const ZividSettingsType& settings);
  std::size_t numAcquisitionConfigServers() const;

private:
  using SettingsConfigTypeDRServer = ConfigDRServer<SettingsConfigType, ZividSettingsType>;
  using SettingsAcquisitionConfigTypeDRServer =
      ConfigDRServer<SettingsAcquisitionConfigType, typename ZividSettingsType::Acquisition>;
  std::string config_node_name_;
  Zivid::CameraInfo m_cameraInfo;
  std::unique_ptr<SettingsConfigTypeDRServer> general_config_dr_server_;
  std::vector<std::unique_ptr<SettingsAcquisitionConfigTypeDRServer>> acquisition_config_dr_servers_;
};

extern template class CaptureSettingsController<Zivid::Settings, zivid_camera::SettingsConfig,
                                                zivid_camera::SettingsAcquisitionConfig>;
extern template class CaptureSettingsController<Zivid::Settings2D, zivid_camera::Settings2DConfig,
                                                zivid_camera::Settings2DAcquisitionConfig>;

}  // namespace zivid_camera
