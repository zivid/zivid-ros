// This program generates dynamic_reconfigure .cfg files, as well as c++ header files,
// for our settings types (Settings, Settings2D, Settings::Acquisition and
// Settings2D::Acquisition). The utility header files are used by the zivid_camera library.
// This program is compiled and run during the build stage of the zivid_camera library.

#include <Zivid/Point.h>
#include <Zivid/Settings.h>
#include <Zivid/Settings2D.h>

#include <dynamic_reconfigure/config_tools.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
template <typename T>
struct DependentFalse : std::false_type
{
};

template <typename T, typename...>
struct IsInList : std::false_type
{
};

template <typename T, typename First, typename... Rest>
struct IsInList<T, First, Rest...>
  : std::integral_constant<bool, std::is_same<T, First>::value || IsInList<T, Rest...>::value>
{
};

template <typename T, typename Tuple>
struct IsInTuple;

template <typename T, typename... Ts>
struct IsInTuple<T, std::tuple<Ts...>> : IsInList<T, Ts...>
{
};

template <typename SettingsNode>
struct IsEnumSetting : std::is_enum<typename SettingsNode::ValueType>
{
};

void writeToFile(const std::string& file_name, const std::string& text)
{
  if (boost::filesystem::exists(file_name))
  {
    std::ifstream file(file_name);
    if (!file.is_open())
    {
      throw std::runtime_error("Unable to open file '" + file_name + "' to check its contents!");
    }
    auto file_contents_ss = std::ostringstream{};
    file_contents_ss << file.rdbuf();
    if (file_contents_ss.str() == text)
    {
      // If the file already exists, and content is identical to "text" then do not modify the file.
      // This ensures that we avoid unnecessary rebuilds.
      return;
    }
  }

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

std::string toUpperCaseFirst(std::string value)
{
  if (value.empty())
  {
    throw std::invalid_argument("value is empty");
  }
  value[0] = std::toupper(value[0]);
  return value;
}

template <typename SettingsRootGroup, typename SettingsNode>
std::string convertSettingsPathToConfigPath()
{
  auto path = std::string(SettingsNode::path);
  const auto root_path = std::string(SettingsRootGroup::path);

  if (!root_path.empty())
  {
    const auto expected_prefix = root_path + "/";
    if (path.substr(0, expected_prefix.length()) != expected_prefix)
    {
      throw std::runtime_error("Expected path '" + path + "' to begin with '" + expected_prefix + "'");
    }
    path = path.substr(expected_prefix.length());
  }

  path = boost::replace_all_copy<std::string>(path, "/", "_");
  const std::regex re("([^_^])([A-Z])");
  path = std::regex_replace(path, re, "$1_$2");  // Convert e.g. ExposureTime to Exposure_Time
  return boost::algorithm::to_lower_copy(path);
}

template <typename SettingsRootGroup>
std::string zividSettingsTypeName()
{
  if constexpr (std::is_same_v<SettingsRootGroup, Zivid::Settings> ||
                IsInTuple<SettingsRootGroup, Zivid::Settings::Descendants>::value ||
                std::is_same_v<SettingsRootGroup, Zivid::Settings::Acquisition> ||
                IsInTuple<SettingsRootGroup, Zivid::Settings::Acquisition::Descendants>::value)
  {
    return Zivid::Settings::name;
  }
  else if constexpr (std::is_same_v<SettingsRootGroup, Zivid::Settings2D> ||
                     IsInTuple<SettingsRootGroup, Zivid::Settings2D::Descendants>::value ||
                     std::is_same_v<SettingsRootGroup, Zivid::Settings2D::Acquisition> ||
                     IsInTuple<SettingsRootGroup, Zivid::Settings2D::Acquisition::Descendants>::value)
  {
    return Zivid::Settings2D::name;
  }
  else
  {
    static_assert(DependentFalse<SettingsRootGroup>::value, "Unhandled type.");
  }
}

template <typename SettingsNode>
std::string fullyQualifiedZividTypeName()
{
  std::stringstream ss;
  ss << "Zivid::" + zividSettingsTypeName<SettingsNode>();
  const auto path = std::string(SettingsNode::path);
  if (!path.empty())
  {
    ss << "::" << boost::replace_all_copy<std::string>(path, "/", "::");
  }
  return ss.str();
}

template <typename SettingsNode>
std::string settingEnumValueToRosEnumName(typename SettingsNode::ValueType value)
{
  // During build dynamic_reconfigure will auto-generate variables for C++ and Python
  // for all enum values, scoped directly under the top level .cfg name. So, for example,
  // an enum value named X listed in Settings.cfg would auto-generate an const int variable
  // zivid_camera::Settings_X. Since we may have several different enum settings with the
  // same enum value X, we need to scope the enum name with the full path to the setting.
  // This ensures there are no collisions now or in the future.
  const auto value_str = toUpperCaseFirst(SettingsNode{ value }.toString());
  const auto path = boost::replace_all_copy<std::string>(SettingsNode::path, "/", "");
  return path + value_str;
}

template <typename SettingsNode>
std::string generateEnumConstant(typename SettingsNode::ValueType value)
{
  static_assert(IsEnumSetting<SettingsNode>::value);
  const auto name = settingEnumValueToRosEnumName<SettingsNode>(value);
  const auto int_value = static_cast<std::size_t>(value);
  const auto description = toUpperCaseFirst(SettingsNode{ value }.toString());
  return (boost::format(R"(gen.const("%1%", int_t, %2%, "%3%"))") % name % int_value % description).str();
}

template <typename SettingsRootGroup>
class DynamicReconfigureCfgGenerator
{
public:
  DynamicReconfigureCfgGenerator(const std::string& class_name) : class_name_(class_name), insert_enabled_(false)
  {
  }

  template <typename SettingsNode>
  auto cfgFileDefaultValue()
  {
    // The default value of settings varies per Zivid camera model. Thus, we cannot set a
    // correct static hard-coded default value for the setting here. To correctly use the
    // Zivid driver the user must load the default values via dynamic_reconfigure at runtime.
    using ValueType = typename SettingsNode::ValueType;
    if constexpr (std::is_same_v<ValueType, bool>)
    {
      return false;
    }
    else if constexpr (Zivid::DataModel::HasValidRange<SettingsNode>::value)
    {
      return SettingsNode::validRange().min();
    }
    else if constexpr (IsEnumSetting<SettingsNode>::value)
    {
      return *SettingsNode::validValues().begin();
    }
    else if constexpr (std::is_same_v<ValueType, Zivid::Range<double>>)
    {
      return ValueType{ 0.0, 0.0 };
    }
    else if constexpr (std::is_same<ValueType, Zivid::PointXYZ>::value)
    {
      return ValueType{ 0.0f, 0.0f, 0.0f };
    }
    else
    {
      return ValueType{ 0 };
    }
  }

  template <typename ValueType>
  auto convertValueToRosValue(ValueType value)
  {
    if constexpr (std::is_same_v<ValueType, bool> || std::is_same_v<ValueType, double> ||
                  std::is_same_v<ValueType, int>)
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
    else if constexpr (std::is_enum_v<ValueType>)
    {
      return static_cast<int>(value);
    }
    else if constexpr (std::is_same_v<ValueType, Zivid::Range<double>>)
    {
      return value;
    }
    else if constexpr (std::is_same_v<ValueType, Zivid::PointXYZ>)
    {
      return value;
    }
    else
    {
      static_assert(DependentFalse<ValueType>::value, "Could not convert ValueType to ROS type.");
    }
  }

  template <typename RosType>
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
    else if constexpr (std::is_same_v<RosType, Zivid::Range<double>> || std::is_same_v<RosType, Zivid::PointXYZ>)
    {
      return "double_t";
    }
    else
    {
      static_assert(DependentFalse<RosType>::value, "Could not convert RosType to a ROS typename string.");
    }
  }

  template <typename RosType>
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
    else if constexpr (std::is_same_v<RosType, Zivid::Range<double>> || std::is_same_v<RosType, Zivid::PointXYZ>)
    {
      return "0.0";
    }
    else
    {
      static_assert(DependentFalse<RosType>::value, "Could not convert RosType to a string value.");
    }
  }

  template <typename ValueType>
  std::string valueTypeToRosTypeString(ValueType v)
  {
    return rosTypeToString(convertValueToRosValue(v));
  }

  std::string rosGeneratedEnumVariableName(const std::string& setting_name)
  {
    return setting_name + "_enum";
  }

  template <typename SettingsNode>
  void apply(const SettingsNode& node)
  {
    const auto setting_name = convertSettingsPathToConfigPath<SettingsRootGroup, SettingsNode>();
    const auto level = "0";
    // Newlines must be converted to \\n so that the auto-generated files end up being correct
    const auto description = boost::replace_all_copy<std::string>(node.description, "\n", R"(\\n)");
    const auto default_value = cfgFileDefaultValue<SettingsNode>();
    const auto type_name = rosTypeName<decltype(convertValueToRosValue(default_value))>();
    const auto default_value_str = valueTypeToRosTypeString(default_value);

    if constexpr (IsEnumSetting<SettingsNode>::value)
    {
      const auto valid_values = node.validValues();
      std::vector<std::string> enum_constants;
      std::transform(valid_values.cbegin(), valid_values.cend(), std::back_inserter(enum_constants),
                     [&](const auto value) { return generateEnumConstant<SettingsNode>(value); });
      ss_ << rosGeneratedEnumVariableName(setting_name) << " = gen.enum([\n    "
          << boost::algorithm::join(enum_constants, ",\n    ") << "\n],\n \"" << description << "\")\n";
    }

    if constexpr (std::is_same_v<std::decay_t<decltype(default_value)>, Zivid::PointXYZ>)
    {
      for (const auto comp : std::array{ "x", "y", "z" })
      {
        const auto comp_name = setting_name + "_" + comp;
        ss_ << "gen.add(\"" << comp_name << "\", " << type_name << ", " << level << ", "
            << "\"" << description + " [" + comp + "]"
            << "\", " << default_value_str << ")\n";
      }
    }
    else if constexpr (std::is_same_v<std::decay_t<decltype(default_value)>, Zivid::Range<double>>)
    {
      for (const auto comp : std::array{ "min", "max" })
      {
        const auto comp_name = setting_name + "_" + comp;
        ss_ << "gen.add(\"" << comp_name << "\", " << type_name << ", " << level << ", "
            << "\"" << description + " [" + comp + "]"
            << "\", " << default_value_str << ")\n";
      }
    }
    else
    {
      ss_ << "gen.add(\"" << setting_name << "\", " << type_name << ", " << level << ", "
          << "\"" << description << "\", " << default_value_str;

      if constexpr (Zivid::DataModel::HasValidRange<SettingsNode>::value)
      {
        ss_ << ", " << valueTypeToRosTypeString(node.validRange().min()) << ", "
            << valueTypeToRosTypeString(node.validRange().max());
      }
      else if constexpr (IsEnumSetting<SettingsNode>::value)
      {
        const auto min_index = 0;
        const auto max_index = node.validValues().size() - 1;
        ss_ << ", " << min_index << ", " << max_index << ", edit_method=" << rosGeneratedEnumVariableName(setting_name);
      }
      ss_ << ")\n";
    }
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
      res << "gen.add(\"enabled\", bool_t, 0, \"When this acquisition is enabled it will be included in captures\", "
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

template <typename SettingsRootGroup>
class ApplyConfigToZividSettingsGenerator
{
public:
  ApplyConfigToZividSettingsGenerator(const std::string& config_class_name) : config_class_name_(config_class_name)
  {
  }

  template <typename SettingsNode>
  void apply(const SettingsNode& node)
  {
    using ValueType = typename SettingsNode::ValueType;

    const auto cfg_id = "cfg." + convertSettingsPathToConfigPath<SettingsRootGroup, SettingsNode>();
    const auto setting_node_class_name = fullyQualifiedZividTypeName<SettingsNode>();
    ss_ << "  s.set(" + setting_node_class_name + "{ ";

    if constexpr (std::is_same_v<ValueType, std::size_t>)
    {
      ss_ << "static_cast<std::size_t>(" + cfg_id + ")";
    }
    else if constexpr (std::is_same_v<ValueType, std::chrono::microseconds>)
    {
      ss_ << "std::chrono::microseconds(" + cfg_id + ")";
    }
    else if constexpr (IsEnumSetting<SettingsNode>::value)
    {
      ss_ << "\n"
          << "  [value = " + cfg_id + "](){\n"
          << "    switch(value) {\n";
      const auto valid_values = node.validValues();
      for (const auto& enum_value : valid_values)
      {
        ss_ << "    case zivid_camera::" << config_class_name_ << "_"
            << settingEnumValueToRosEnumName<SettingsNode>(enum_value) << ":\n"
            << "      return " + setting_node_class_name + "::" + SettingsNode{ enum_value }.toString() + ";\n";
      }
      ss_ << "    }\n"
          << "    throw std::runtime_error(\"Could not convert int value \" + std::to_string(value) + \""
          << " to setting of type " + setting_node_class_name + ".\");\n"
          << "  }()\n";
    }
    else if constexpr (std::is_same_v<ValueType, Zivid::PointXYZ>)
    {
      ss_ << "static_cast<float>(" << cfg_id << "_x), static_cast<float>(" << cfg_id << "_y), static_cast<float>("
          << cfg_id << "_z)";
    }
    else if constexpr (std::is_same_v<ValueType, Zivid::Range<double>>)
    {
      ss_ << cfg_id << "_min, " << cfg_id << "_max ";
    }
    else
    {
      ss_ << cfg_id;
    }
    ss_ << " });\n";
  }

  std::string str()
  {
    const auto root_type_fq = fullyQualifiedZividTypeName<SettingsRootGroup>();

    std::stringstream res;
    res << "inline static void applyConfigToZividSettings(const zivid_camera::" << config_class_name_ << "Config& cfg, "
        << root_type_fq << "& s)\n";
    res << "{\n";
    res << ss_.str();
    res << "}\n";
    return res.str();
  }

private:
  std::string config_class_name_;
  std::stringstream ss_;
};

template <typename SettingsRootGroup>
class ZividSettingsToConfigGenerator
{
public:
  ZividSettingsToConfigGenerator(const std::string& config_class_name) : config_class_name_(config_class_name)
  {
  }

  template <typename SettingsNode>
  void apply(const SettingsNode&)
  {
    using ValueType = typename SettingsNode::ValueType;
    const auto cfg_id = "cfg." + convertSettingsPathToConfigPath<SettingsRootGroup, SettingsNode>();
    const auto zivid_node_class_name = fullyQualifiedZividTypeName<SettingsNode>();
    const auto value_str = "s.get<" + zivid_node_class_name + ">().value()";

    if constexpr (std::is_same_v<ValueType, Zivid::PointXYZ>)
    {
      for (const auto comp : std::array{ "x", "y", "z" })
      {
        ss_ << "  " + cfg_id + "_" + comp + " = "
            << "static_cast<double>(" + value_str + "." + comp + ");\n";
      }
    }
    else if constexpr (std::is_same_v<ValueType, Zivid::Range<double>>)
    {
      for (const auto comp : std::array{ "min", "max" })
      {
        ss_ << "  " + cfg_id + "_" + comp + " = " << value_str + "." + comp + "();\n";
      }
    }
    else
    {
      ss_ << "  " + cfg_id + " = ";
      if constexpr (std::is_same_v<ValueType, std::chrono::microseconds>)
      {
        ss_ << "static_cast<int>(" + value_str + ".count());\n";
      }
      else if constexpr (std::is_same_v<ValueType, std::size_t>)
      {
        ss_ << "static_cast<int>(" + value_str + ");\n";
      }
      else if constexpr (IsEnumSetting<SettingsNode>::value)
      {
        ss_ << "static_cast<int>(" + value_str + ");\n";
      }
      else
      {
        ss_ << value_str + ";\n";
      }
    }
  }

  std::string str() const
  {
    const auto full_class_name = "zivid_camera::" + config_class_name_ + "Config";
    const auto root_type_fq = fullyQualifiedZividTypeName<SettingsRootGroup>();

    std::stringstream res;
    res << "template<> inline " << full_class_name << " zividSettingsToConfig";
    res << "<" << full_class_name << ">(const " << root_type_fq << "& s)\n";
    res << "{\n";
    res << "  auto cfg = " + full_class_name << "::__getDefault__();\n";
    res << ss_.str();
    res << "  return cfg;\n";
    res << "}\n";
    return res.str();
  }

private:
  std::string config_class_name_;
  std::stringstream ss_;
};

template <typename SettingsRootGroup>
class MinMaxConfigGenerator
{
public:
  enum class Type
  {
    Min,
    Max,
  };

  MinMaxConfigGenerator(const std::string& config_class_name, Type type)
    : config_class_name_(config_class_name), type_(type)
  {
  }

  template <typename SettingsNode>
  void apply(const SettingsNode&)
  {
    if constexpr (Zivid::DataModel::HasValidRange<SettingsNode>::value)
    {
      using ValueType = typename SettingsNode::ValueType;
      const auto cfg_id = "cfg." + convertSettingsPathToConfigPath<SettingsRootGroup, SettingsNode>();
      const auto zivid_node_class_name = fullyQualifiedZividTypeName<SettingsNode>();

      const auto value_str = [&]() {
        std::string prefix =
            "Zivid::Experimental::SettingsInfo::validRange<" + zivid_node_class_name + ">(camera.info())";
        switch (type_)
        {
          case Type::Min:
            return prefix + ".min()";
          case Type::Max:
            return prefix + ".max()";
        }
        throw std::runtime_error(std::string(__func__) + ": Unhandled enum: " + toString(type_));
      }();

      ss_ << "  " + cfg_id + " = ";

      if constexpr (std::is_same_v<ValueType, std::chrono::microseconds>)
      {
        ss_ << "static_cast<int>(" + value_str + ".count());\n";
      }
      else if constexpr (std::is_same_v<ValueType, std::size_t>)
      {
        ss_ << "static_cast<int>(" + value_str + ");\n";
      }
      else
      {
        ss_ << value_str + ";\n";
      }
    }
  }

  std::string initializeConfigFunction() const
  {
    switch (type_)
    {
      case Type::Min:
        return "__getMin__";
      case Type::Max:
        return "__getMax__";
    }
    throw std::runtime_error(std::string(__func__) + ": Unhandled enum: " + toString(type_));
  }

  std::string str() const
  {
    const auto full_class_name = "zivid_camera::" + config_class_name_ + "Config";

    const auto function_name_config_type = [&]() {
      switch (type_)
      {
        case Type::Min:
          return "Min";
        case Type::Max:
          return "Max";
      }
      throw std::runtime_error(std::string(__func__) + ": Unhandled enum: " + toString(type_));
    }();

    const auto root_type_fq = fullyQualifiedZividTypeName<SettingsRootGroup>();

    std::stringstream res;
    res << "template<> inline " << full_class_name << " zividSettings" << function_name_config_type << "Config";
    res << "<" << full_class_name << ", " << root_type_fq << ">(const Zivid::Camera& camera)\n";
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
  std::string config_class_name_;
  Type type_;
  std::stringstream ss_;
};

template <typename SettingsRootGroup>
class ConfigUtilsHeaderGenerator
{
public:
  ConfigUtilsHeaderGenerator(const std::string& config_class_name)
    : apply_config_zivid_settings_gen(config_class_name)
    , zivid_settings_to_config_gen(config_class_name)
    , min_config_gen(config_class_name, MinMaxConfigGenerator<SettingsRootGroup>::Type::Min)
    , max_config_gen(config_class_name, MinMaxConfigGenerator<SettingsRootGroup>::Type::Max)
  {
  }

  template <typename SettingsNode>
  void apply(const SettingsNode& node)
  {
    apply_config_zivid_settings_gen.apply(node);
    zivid_settings_to_config_gen.apply(node);
    min_config_gen.apply(node);
    max_config_gen.apply(node);
  }

  std::string str()
  {
    std::stringstream res;
    res << "#pragma once\n\n";
    res << "// This is an auto-generated header. Do not edit.\n\n";
    res << "#include <Zivid/Camera.h>\n";
    res << "#include <Zivid/Experimental/SettingsInfo.h>\n\n";
    res << "#include \"config_utils_common.h\"\n\n";

    res << apply_config_zivid_settings_gen.str() << "\n";
    res << zivid_settings_to_config_gen.str() << "\n";
    res << min_config_gen.str() << "\n";
    res << max_config_gen.str() << "\n";
    return res.str();
  }

  ApplyConfigToZividSettingsGenerator<SettingsRootGroup> apply_config_zivid_settings_gen;
  ZividSettingsToConfigGenerator<SettingsRootGroup> zivid_settings_to_config_gen;
  MinMaxConfigGenerator<SettingsRootGroup> min_config_gen;
  MinMaxConfigGenerator<SettingsRootGroup> max_config_gen;
};

template <typename SettingsRootGroup>
class Generator
{
public:
  Generator(const std::string& config_class_name)
    : config_class_name_(config_class_name)
    , dynamic_reconfigure_cfg_gen_(config_class_name)
    , config_utils_header_gen_(config_class_name)
  {
  }

  template <typename SettingsNode>
  void apply(const SettingsNode& node)
  {
    dynamic_reconfigure_cfg_gen_.apply(node);
    config_utils_header_gen_.apply(node);
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
  std::string config_class_name_;
  DynamicReconfigureCfgGenerator<SettingsRootGroup> dynamic_reconfigure_cfg_gen_;
  ConfigUtilsHeaderGenerator<SettingsRootGroup> config_utils_header_gen_;
};

template <typename SettingsType, typename SettingsNode, typename GeneratorType>
void traverseSettingsTree(const SettingsNode& node, GeneratorType& generator)
{
  if constexpr (std::is_same_v<SettingsNode, typename SettingsType::Acquisitions>)
  {
    // Acquisitions ignored here. Acqusitition is handled separately.
  }
  else if constexpr (SettingsNode::nodeType == Zivid::DataModel::NodeType::group)
  {
    node.forEach([&](const auto& child) { traverseSettingsTree<SettingsType>(child, generator); });
  }
  else
  {
    generator.apply(node);
  }
}

template <typename SettingsType>
void addSettingsType(const std::string& cfgPrefix)
{
  Generator<SettingsType> capture_general_gen(cfgPrefix);
  traverseSettingsTree<SettingsType>(SettingsType{}, capture_general_gen);
  capture_general_gen.writeToFiles();

  Generator<typename SettingsType::Acquisition> capture_acquisition_gen(cfgPrefix + "Acquisition");
  traverseSettingsTree<SettingsType>(typename SettingsType::Acquisition{}, capture_acquisition_gen);
  capture_acquisition_gen.insertEnabled();
  capture_acquisition_gen.writeToFiles();
}

}  // namespace

int main(int /*argc*/, char** /*argv*/)
{
  addSettingsType<Zivid::Settings>("Settings");
  addSettingsType<Zivid::Settings2D>("Settings2D");
  return 0;
}
