#pragma once

// Helper template functions that convert from a ZividSettings object to current/min/max
// Config object. The explicit specializations of these template functions are auto-generated
// during the build.

template <typename ConfigType, typename ZividSettings>
ConfigType zividSettingsToConfig(const ZividSettings& s) = delete;
template <typename ConfigType, typename ZividSettings>
ConfigType zividSettingsToMinConfig(const ZividSettings& s) = delete;
template <typename ConfigType, typename ZividSettings>
ConfigType zividSettingsToMaxConfig(const ZividSettings& s) = delete;
