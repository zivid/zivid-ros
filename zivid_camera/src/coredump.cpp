#include "coredump.h"

#include <ros/console.h>

#include <sys/resource.h>

#include <fstream>

namespace
{
    std::string readFileAsText(const std::string &filename)
    {
        std::ifstream fileStream{ filename };
        return std::string{ std::istreambuf_iterator<char>{ fileStream }, std::istreambuf_iterator<char>{} };
    }
}

namespace zivid_coredump
{
  void enableCoreDump()
  {
    rlimit rlim{};
    if (getrlimit(RLIMIT_CORE, &rlim) != 0)
    {
      throw std::system_error(errno, std::system_category(), "Failed to get core dump file size limit");
    }

    if (rlim.rlim_max == 0)
    {
      ROS_WARN_STREAM("Maximum core dump file size limit is set to 0 bytes. Core dump files will not be generated.");
      return;
    }

    rlim.rlim_cur = rlim.rlim_max;
    if (setrlimit(RLIMIT_CORE, &rlim) != 0)
    {
      throw std::system_error(errno, std::system_category(), "Failed to set core dump file size");
    }

    ROS_INFO_STREAM("Successfully set core dump file size limit to " << rlim.rlim_cur << " bytes");
  }

  void checkCoreDumpLocation()
  {
    auto corePattern = readFileAsText("/proc/sys/kernel/core_pattern");
    if (corePattern.empty())
    {
      ROS_WARN_STREAM("/proc/sys/kernel/core_pattern is empty.");
      return;
    }

    // Extract only the path from the core pattern, separated by a space
    const auto path = corePattern.substr(0, corePattern.find(' '));

    if (path.at(0) == '|')
    {
      ROS_INFO_STREAM("Core dump location is set to a pipe: " << path.substr(1));
    }
    else
    {
      ROS_INFO_STREAM("Core dump location is set to: " << path);
    }
  }
}
