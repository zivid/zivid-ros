#pragma once

// Helper template functions that convert from a ZividSettings object to min/max/default
// Config objects. The explicit specializations of these template functions are auto-generated
// during the build.

template <typename ConfigType, typename ZividSettings>
ConfigType getMinConfigFromZividSettings(const ZividSettings& s) = delete;
template <typename ConfigType, typename ZividSettings>
ConfigType getMaxConfigFromZividSettings(const ZividSettings& s) = delete;
template <typename ConfigType, typename ZividSettings>
ConfigType getDefaultConfigFromZividSettings(const ZividSettings& s) = delete;
