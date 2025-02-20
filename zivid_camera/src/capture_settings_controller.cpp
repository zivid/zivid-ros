#include "capture_settings_controller.h"

#include "SettingsConfigUtils.h"
#include "SettingsAcquisitionConfigUtils.h"
#include "Settings2DConfigUtils.h"
#include "Settings2DAcquisitionConfigUtils.h"

#include <Zivid/Camera.h>
#include <Zivid/Experimental/SettingsInfo.h>

#include <dynamic_reconfigure/server.h>

namespace
{
template <typename Node>
void recursivelyFillInUnsetWithCameraDefault(Node& node, const Zivid::CameraInfo& cameraInfo)
{
  if constexpr (Node::nodeType == Zivid::DataModel::NodeType::group ||
                Node::nodeType == Zivid::DataModel::NodeType::leafDataModelList)
  {
    node.forEach([&cameraInfo](auto& child) { recursivelyFillInUnsetWithCameraDefault(child, cameraInfo); });
  }
  else if (!node.hasValue())
  {
    static_assert(Node::nodeType == Zivid::DataModel::NodeType::leafValue);
    node = Zivid::Experimental::SettingsInfo::defaultValue<Node>(cameraInfo);
  }
}

template <typename ZividSettingsType>
ZividSettingsType fillInUnsetWithCameraDefault(const ZividSettingsType& settings, const Zivid::CameraInfo& cameraInfo)
{
  ZividSettingsType copy{ settings };
  recursivelyFillInUnsetWithCameraDefault(copy, cameraInfo);
  return copy;
}

}  // namespace

namespace zivid_camera
{
template <typename ConfigType, typename ZividSettings>
class ConfigDRServer
{
public:
  ConfigDRServer(const std::string& name, ros::NodeHandle& nh, const Zivid::Camera& camera)
    : name_(name), dr_server_(dr_server_mutex_, ros::NodeHandle(nh, name_)), config_(ConfigType::__getDefault__())
  {
    static_assert(std::is_same_v<ZividSettings, Zivid::Settings> || std::is_same_v<ZividSettings, Zivid::Settings2D> ||
                  std::is_same_v<ZividSettings, Zivid::Settings::Acquisition> ||
                  std::is_same_v<ZividSettings, Zivid::Settings2D::Acquisition>);

    const auto config_min = zividSettingsMinConfig<ConfigType, ZividSettings>(camera);
    dr_server_.setConfigMin(config_min);

    const auto config_max = zividSettingsMaxConfig<ConfigType, ZividSettings>(camera);
    dr_server_.setConfigMax(config_max);

    const auto config_default = zividSettingsToConfig<ConfigType>(
        Zivid::Experimental::SettingsInfo::defaultValue<ZividSettings>(camera.info()));
    dr_server_.setConfigDefault(config_default);
    setConfig(config_default);

    auto cb = [this](const ConfigType& config, uint32_t /*level*/) {
      ROS_INFO("Configuration '%s' changed", name_.c_str());
      config_ = config;
    };
    using CallbackType = typename decltype(dr_server_)::CallbackType;
    dr_server_.setCallback(CallbackType(cb));
  }

  void setConfig(const ConfigType& cfg)
  {
    config_ = cfg;
    dr_server_.updateConfig(config_);
  }

  const ConfigType& config() const
  {
    return config_;
  }

  const std::string& name() const
  {
    return name_;
  }

private:
  std::string name_;
  boost::recursive_mutex dr_server_mutex_;
  dynamic_reconfigure::Server<ConfigType> dr_server_;
  ConfigType config_;
};

template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
CaptureSettingsController<ZividSettingsType, SettingsConfigType,
                          SettingsAcquisitionConfigType>::CaptureSettingsController(ros::NodeHandle& nh,
                                                                                    Zivid::Camera& camera,
                                                                                    const std::string& config_node_name,
                                                                                    std::size_t num_acquisition_servers)
  : config_node_name_{ config_node_name }, camera_info_{ camera.info() }
{
  general_config_dr_server_ = std::make_unique<SettingsConfigTypeDRServer>(config_node_name_, nh, camera);

  ROS_INFO("Setting up %ld %s/acquisition_<n> dynamic_reconfigure servers", num_acquisition_servers,
           config_node_name_.c_str());
  for (std::size_t i = 0; i < num_acquisition_servers; i++)
  {
    acquisition_config_dr_servers_.push_back(std::make_unique<SettingsAcquisitionConfigTypeDRServer>(
        config_node_name_ + "/acquisition_" + std::to_string(i), nh, camera));
  }
}

template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
CaptureSettingsController<ZividSettingsType, SettingsConfigType,
                          SettingsAcquisitionConfigType>::~CaptureSettingsController() = default;

template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
ZividSettingsType
CaptureSettingsController<ZividSettingsType, SettingsConfigType, SettingsAcquisitionConfigType>::zividSettings() const
{
  ZividSettingsType settings;
  applyConfigToZividSettings(general_config_dr_server_->config(), settings);

  for (const auto& dr_config_server : acquisition_config_dr_servers_)
  {
    if (dr_config_server->config().enabled)
    {
      ROS_INFO("Config %s is enabled", dr_config_server->name().c_str());
      typename ZividSettingsType::Acquisition acquisition;
      applyConfigToZividSettings(dr_config_server->config(), acquisition);
      settings.acquisitions().emplaceBack(std::move(acquisition));
    }
  }
  return settings;
}

template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
void CaptureSettingsController<ZividSettingsType, SettingsConfigType, SettingsAcquisitionConfigType>::setZividSettings(
    const ZividSettingsType& settings)
{
  const auto numAcquisitions = settings.acquisitions().size();
  if (numAcquisitions == 0)
  {
    throw std::runtime_error("At least one Acquisition must be provided");
  }

  if (numAcquisitions > numAcquisitionConfigServers())
  {
    std::stringstream error;
    error << "The number of acquisitions (" + std::to_string(numAcquisitions) + ") "
          << "is larger than the number of dynamic_reconfigure " + config_node_name_ + "/acquisition_<n> servers ("
          << std::to_string(numAcquisitionConfigServers()) << "). ";
    if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings>)
    {
      error << "Increase launch parameter max_capture_acquisitions. "
            << "See README.md for more information.";
    }
    else if constexpr (std::is_same_v<ZividSettingsType, Zivid::Settings2D>)
    {
      error << "In 2D mode only a single acquisiton is supported.";
    }
    throw std::runtime_error(error.str());
  }

  const auto filledSettings = fillInUnsetWithCameraDefault(settings, camera_info_);
  ROS_INFO_STREAM("Updating settings to " << filledSettings);
  general_config_dr_server_->setConfig(zividSettingsToConfig<SettingsConfigType>(filledSettings));

  const auto& acquisitions = filledSettings.acquisitions();
  for (std::size_t i = 0; i < acquisitions.size(); i++)
  {
    auto config = zividSettingsToConfig<SettingsAcquisitionConfigType>(acquisitions.at(i));
    config.enabled = true;
    acquisition_config_dr_servers_[i]->setConfig(config);
  }

  // Remaining acquisitions that are enabled must be disabled
  for (std::size_t i = acquisitions.size(); i < acquisition_config_dr_servers_.size(); i++)
  {
    if (acquisition_config_dr_servers_[i]->config().enabled)
    {
      ROS_INFO_STREAM("Acquisition " << i << " was enabled, so disabling it");
      auto config = acquisition_config_dr_servers_[i]->config();
      config.enabled = false;
      acquisition_config_dr_servers_[i]->setConfig(config);
    }
  }
}

template <typename ZividSettingsType, typename SettingsConfigType, typename SettingsAcquisitionConfigType>
std::size_t CaptureSettingsController<ZividSettingsType, SettingsConfigType,
                                      SettingsAcquisitionConfigType>::numAcquisitionConfigServers() const
{
  return acquisition_config_dr_servers_.size();
}

template class CaptureSettingsController<Zivid::Settings, zivid_camera::SettingsConfig,
                                         zivid_camera::SettingsAcquisitionConfig>;
template class CaptureSettingsController<Zivid::Settings2D, zivid_camera::Settings2DConfig,
                                         zivid_camera::Settings2DAcquisitionConfig>;

}  // namespace zivid_camera
