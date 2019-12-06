#include <Zivid/Settings.h>
#include <Zivid/Settings2D.h>

#include <dynamic_reconfigure/config_tools.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace
{
template <class T>
using NormalizedType = std::remove_const_t<std::remove_reference_t<T>>;

template <class T, class ZividSettingNode>
constexpr bool is_zivid_setting = std::is_same<NormalizedType<T>, ZividSettingNode>::value;

template <class T>
constexpr bool is_filters = is_zivid_setting<T, Zivid::Settings::Filters>;

template <class T>
constexpr bool is_color_balance =
    is_zivid_setting<T, Zivid::Settings::BlueBalance> || is_zivid_setting<T, Zivid::Settings::RedBalance>;

template <class T>
constexpr bool is_general_setting = is_filters<T> || is_color_balance<T>;

template <typename C, typename = void>
struct HasRange : std::false_type
{
};

template <typename C>
struct HasRange<C, std::void_t<decltype(std::declval<C>().range())>> : std::true_type
{
};

template <typename T>
struct DependentFalse : std::false_type
{
};

void writeToFile(const std::string& file_name, const std::string& text)
{
  std::ofstream cfg_file(file_name);
  if (!cfg_file || !cfg_file.is_open())
  {
    throw std::runtime_error("Unable to open file '" + file_name + "' for writing!");
  }
  cfg_file << text;
  cfg_file.close();
  if (!cfg_file)
  {
    throw std::runtime_error("Failed to write to file '" + file_name + "'!");
  }
}

std::string convertSettingsPathToZividClassName(const std::string& zivid_settings_class_name, const std::string& path)
{
  return "Zivid::" + zivid_settings_class_name + "::" + boost::replace_all_copy<std::string>(path, "/", "::");
}

std::string convertSettingsPathToConfigPath(std::string path)
{
  path = boost::replace_all_copy<std::string>(path, "/", "_");
  const std::regex re("([^_^])([A-Z])");
  path = std::regex_replace(path, re, "$1_$2");  // Convert e.g. ExposureTime to Exposure_Time
  return boost::algorithm::to_lower_copy(path);
}

class DynamicReconfigureCfgGenerator
{
public:
  DynamicReconfigureCfgGenerator(const std::string& class_name) : class_name_(class_name), insert_enabled_(false)
  {
  }

  template <class ValueType>
  auto convertValueToRosValue(ValueType value)
  {
    // Convert from our own setting value types to types that ROS params supports (double, int, bool)

    if constexpr (std::is_same_v<ValueType, bool> || std::is_same_v<ValueType, double>)
    {
      return value;
    }
    else if constexpr (std::is_same_v<ValueType, std::size_t>)
    {
      return static_cast<int>(value);
    }
    else if constexpr (std::is_same_v<ValueType, std::chrono::microseconds>)
    {
      return static_cast<int>(value.count());
    }
    else
    {
      static_assert(DependentFalse<ValueType>::value, "Could not convert ValueType to ROS type.");
    }
  }

  template <class RosType>
  std::string rosTypeName()
  {
    if constexpr (std::is_same_v<RosType, bool>)
    {
      return "bool_t";
    }
    else if constexpr (std::is_same_v<RosType, double>)
    {
      return "double_t";
    }
    else if constexpr (std::is_same_v<RosType, int>)
    {
      return "int_t";
    }
    else
    {
      static_assert(DependentFalse<RosType>::value, "Could not convert RosType to a ROS typename string.");
    }
  }

  template <class RosType>
  std::string rosTypeToString(RosType v)
  {
    if constexpr (std::is_same_v<RosType, bool>)
    {
      return v ? "True" : "False";
    }
    else if constexpr (std::is_same_v<RosType, double> || std::is_same_v<RosType, int>)
    {
      return std::to_string(v);
    }
    else
    {
      static_assert(DependentFalse<RosType>::value, "Could not convert RosType to a string value.");
    }
  }

  template <class ValueType>
  std::string valueTypeToRosTypeString(ValueType v)
  {
    return rosTypeToString(convertValueToRosValue(v));
  }

  template <class ZividSettingNode>
  void apply(const ZividSettingNode& s)
  {
    const auto setting_name = convertSettingsPathToConfigPath(s.path);
    const auto level = "0";
    // Newlines must be converted to \\n so that the auto-generated files end up being correct
    const auto description = boost::replace_all_copy<std::string>(s.description, "\n", R"(\\n)");
    const auto type_name = rosTypeName<decltype(convertValueToRosValue(s.value()))>();
    const auto default_value = valueTypeToRosTypeString(s.value());

    ss_ << "gen.add(\"" << setting_name << "\", " << type_name << ", " << level << ", "
        << "\"" << description << "\", " << default_value;

    if constexpr (HasRange<NormalizedType<ZividSettingNode>>::value)
    {
      ss_ << ", " << valueTypeToRosTypeString(s.range().min()) << ", " << valueTypeToRosTypeString(s.range().max());
    }
    ss_ << ")\n";
  }

  void insertEnabled()
  {
    insert_enabled_ = true;
  }

  std::string str()
  {
    std::stringstream res;
    res << "#!/usr/bin/env python\n\n"
           "# This is an auto-generated cfg file. Do not edit!\n\n"
           "PACKAGE = \"zivid_camera\"\n"
           "import roslib\n"
           "roslib.load_manifest(PACKAGE);\n"
           "from dynamic_reconfigure.parameter_generator_catkin import *\n\n";

    res << "gen = ParameterGenerator()\n";
    if (insert_enabled_)
    {
      res << "gen.add(\"enabled\", bool_t, 0, \"When frame is enabled it will be included in captures\", "
             "False)\n";
    }
    res << ss_.str();
    res << "gen.generate(PACKAGE, \"zivid_camera\", \"" + class_name_ + "\")\n";
    return res.str();
  }

private:
  std::string class_name_;
  bool insert_enabled_;
  std::stringstream ss_;
};

class ApplyConfigToZividSettingsGenerator
{
public:
  ApplyConfigToZividSettingsGenerator(const std::string& zivid_settings_class_name,
                                      const std::string& config_class_name)
    : zivid_settings_class_name_(zivid_settings_class_name), config_class_name_(config_class_name)
  {
  }

  template <class ZividSettingNode>
  void apply(const ZividSettingNode& s)
  {
    using T = NormalizedType<decltype(s)>;
    using VT = typename T::ValueType;

    const auto cfg_id = "cfg." + convertSettingsPathToConfigPath(s.path);
    const auto setting_node_class_name = convertSettingsPathToZividClassName(zivid_settings_class_name_, s.path);
    ss_ << "  s.set(" + setting_node_class_name + "{ ";

    if constexpr (std::is_same_v<VT, std::size_t>)
    {
      ss_ << "static_cast<std::size_t>(" + cfg_id + ")";
    }
    else if constexpr (std::is_same_v<VT, std::chrono::microseconds>)
    {
      ss_ << "std::chrono::microseconds(" + cfg_id + ")";
    }
    else
    {
      ss_ << cfg_id;
    }
    ss_ << " });\n";
  }

  std::string str()
  {
    std::stringstream res;
    res << "static void apply" << config_class_name_
        << "ConfigToZividSettings(const zivid_camera::" << config_class_name_
        << "Config& cfg, Zivid::" << zivid_settings_class_name_ << "& s)\n";
    res << "{\n";
    res << ss_.str();
    res << "}\n";
    return res.str();
  }

private:
  std::string zivid_settings_class_name_;
  std::string config_class_name_;
  std::stringstream ss_;
};

class ZividSettingsToMinMaxCurrentValueConfigGenerator
{
public:
  enum class Type
  {
    CurrentValue,
    Min,
    Max,
  };

  ZividSettingsToMinMaxCurrentValueConfigGenerator(const std::string& zivid_settings_class_name,
                                                   const std::string& config_class_name, Type type)
    : zivid_settings_class_name_(zivid_settings_class_name), config_class_name_(config_class_name), type_(type)
  {
  }

  template <class ZividSettingNode>
  void apply(const ZividSettingNode& s)
  {
    using T = NormalizedType<decltype(s)>;
    if (type_ == Type::CurrentValue || HasRange<T>::value)
    {
      using VT = typename T::ValueType;
      const auto cfg_id = "cfg." + convertSettingsPathToConfigPath(s.path);
      const auto zivid_class_name = convertSettingsPathToZividClassName(zivid_settings_class_name_, s.path);

      const auto valueStr = [&]() {
        std::string prefix = "s.get<" + zivid_class_name + ">()";
        switch (type_)
        {
          case Type::CurrentValue:
            return prefix + ".value()";
          case Type::Min:
            return prefix + ".range().min()";
          case Type::Max:
            return prefix + ".range().max()";
        }
        throw std::runtime_error(std::string(__func__) + ": Unhandled enum: " + toString(type_));
      }();

      if constexpr (std::is_same_v<VT, std::chrono::microseconds>)
      {
        ss_ << "  " + cfg_id + " = static_cast<int>(" + valueStr + ".count());\n";
      }
      else if constexpr (std::is_same_v<VT, std::size_t>)
      {
        ss_ << "  " + cfg_id + " = static_cast<int>(" + valueStr + ");\n";
      }
      else
      {
        ss_ << "  " + cfg_id + " = " + valueStr + ";\n";
      }
    }
  }

  std::string initializeConfigFunction() const
  {
    switch (type_)
    {
      case Type::CurrentValue:
        return "__getDefault__";
      case Type::Min:
        return "__getMin__";
      case Type::Max:
        return "__getMax__";
    }
    throw std::runtime_error(std::string(__func__) + ": Unhandled enum: " + toString(type_));
  }

  std::string str()
  {
    const auto full_class_name = "zivid_camera::" + config_class_name_ + "Config";

    const auto functionNameConfigType = [&]() {
      switch (type_)
      {
        case Type::CurrentValue:
          return "";
        case Type::Min:
          return "Min";
        case Type::Max:
          return "Max";
      }
      throw std::runtime_error(std::string(__func__) + ": Unhandled enum: " + toString(type_));
    }();

    std::stringstream res;
    res << "template<> " << full_class_name << " zividSettingsTo" << functionNameConfigType << "Config";
    res << "<" << full_class_name << ">(const Zivid::" << zivid_settings_class_name_ << "& s)\n";
    res << "{\n";
    res << "  auto cfg = " + full_class_name << "::" << initializeConfigFunction() << "();\n";
    res << ss_.str();
    res << "  return cfg;\n";
    res << "}\n";
    return res.str();
  }

  static std::string toString(Type t)
  {
    return std::to_string(static_cast<std::underlying_type_t<Type>>(t));
  }

private:
  std::string zivid_settings_class_name_;
  std::string config_class_name_;
  Type type_;
  std::stringstream ss_;
};

class ConfigUtilsHeaderGenerator
{
public:
  ConfigUtilsHeaderGenerator(const std::string& zivid_settings_class_name, const std::string& config_class_name)
    : apply_config_zivid_settings_gen(zivid_settings_class_name, config_class_name)
    , zivid_settings_to_config_gen(zivid_settings_class_name, config_class_name,
                                   ZividSettingsToMinMaxCurrentValueConfigGenerator::Type::CurrentValue)
    , zivid_settings_to_min_config_gen(zivid_settings_class_name, config_class_name,
                                       ZividSettingsToMinMaxCurrentValueConfigGenerator::Type::Min)
    , zivid_settings_to_max_config_gen(zivid_settings_class_name, config_class_name,
                                       ZividSettingsToMinMaxCurrentValueConfigGenerator::Type::Max)
  {
  }

  template <class ZividSetting>
  void apply(const ZividSetting& s)
  {
    apply_config_zivid_settings_gen.apply(s);
    zivid_settings_to_config_gen.apply(s);
    zivid_settings_to_min_config_gen.apply(s);
    zivid_settings_to_max_config_gen.apply(s);
  }

  std::string str()
  {
    std::stringstream res;
    res << "#pragma once\n\n";
    res << "// This is an auto-generated header. Do not edit.\n\n";
    res << "#include \"config_utils_common.h\"\n\n";
    res << apply_config_zivid_settings_gen.str() << "\n";
    res << zivid_settings_to_config_gen.str() << "\n";
    res << zivid_settings_to_min_config_gen.str() << "\n";
    res << zivid_settings_to_max_config_gen.str() << "\n";
    return res.str();
  }

  ApplyConfigToZividSettingsGenerator apply_config_zivid_settings_gen;
  ZividSettingsToMinMaxCurrentValueConfigGenerator zivid_settings_to_config_gen;
  ZividSettingsToMinMaxCurrentValueConfigGenerator zivid_settings_to_min_config_gen;
  ZividSettingsToMinMaxCurrentValueConfigGenerator zivid_settings_to_max_config_gen;
};

class Generator
{
public:
  template <class ZividSettings>
  Generator(const ZividSettings& settings, const std::string& config_class_name)
    : zivid_settings_class_name_(settings.name)
    , config_class_name_(config_class_name)
    , dynamic_reconfigure_cfg_gen_(config_class_name)
    , config_utils_header_gen_(zivid_settings_class_name_, config_class_name)
  {
  }

  template <class ZividSettingNode>
  void apply(const ZividSettingNode& s)
  {
    dynamic_reconfigure_cfg_gen_.apply(s);
    config_utils_header_gen_.apply(s);
  }

  void insertEnabled()
  {
    dynamic_reconfigure_cfg_gen_.insertEnabled();
  }

  void writeToFiles()
  {
    writeToFile(config_class_name_ + ".cfg", dynamic_reconfigure_cfg_gen_.str());
    writeToFile("generated_headers/" + config_class_name_ + "ConfigUtils.h", config_utils_header_gen_.str());
  }

private:
  std::string zivid_settings_class_name_;
  std::string config_class_name_;
  DynamicReconfigureCfgGenerator dynamic_reconfigure_cfg_gen_;
  ConfigUtilsHeaderGenerator config_utils_header_gen_;
};

template <typename ZividSettingNode>
void traverseGeneralSettingsTree(const ZividSettingNode& s, Generator& general_settings)
{
  if constexpr (ZividSettingNode::isContainer)
  {
    s.forEach([&](const auto& c) { traverseGeneralSettingsTree(c, general_settings); });
  }
  else
  {
    general_settings.apply(s);
  }
}

template <typename ZividSettingNode>
void traverseSettingsTree(const ZividSettingNode& s, Generator& general_settings, Generator& frame_settings)
{
  if constexpr (is_general_setting<ZividSettingNode>)
  {
    traverseGeneralSettingsTree(s, general_settings);
  }
  else if constexpr (ZividSettingNode::isContainer)
  {
    s.forEach([&](const auto& c) { traverseSettingsTree(c, general_settings, frame_settings); });
  }
  else
  {
    frame_settings.apply(s);
  }
}

void traverseSettings2DTree(const Zivid::Settings2D& s, Generator& frame_settings)
{
  s.forEach([&](const auto& c) { frame_settings.apply(c); });
}

}  // namespace

int main(int /*argc*/, char** /*argv*/)
{
  const auto settings = Zivid::Settings{};
  Generator capture_general_gen(settings, "CaptureGeneral");
  Generator capture_frame_gen(settings, "CaptureFrame");

  traverseSettingsTree(settings, capture_general_gen, capture_frame_gen);
  capture_frame_gen.insertEnabled();
  capture_general_gen.writeToFiles();
  capture_frame_gen.writeToFiles();

  const auto settings2D = Zivid::Settings2D{};
  Generator capture2D_frame_gen(settings2D, "Capture2DFrame");
  traverseSettings2DTree(settings2D, capture2D_frame_gen);
  capture2D_frame_gen.insertEnabled();
  capture2D_frame_gen.writeToFiles();

  return 0;
}
