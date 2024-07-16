#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?

cd ~/ros2_ws || exit $?

echo "Installing Zivid API config file"
install -D "$SCRIPT_DIR"/ZividAPIConfigCPU.yml "$HOME"/.config/Zivid/API/Config.yml || exit $?

echo "Download and install zivid sample data (file camera)"
wget -q https://www.zivid.com/software/FileCameraZivid2M70.zip || exit $?
mkdir -p /usr/share/Zivid/data/ || exit $?
unzip ./FileCameraZivid2M70.zip -d /usr/share/Zivid/data/ || exit $?
rm ./FileCameraZivid2M70.zip || exit $?

echo "Running tests"

# We exclude `clang_format` here since it has variations between versions, instead we check it during code analysis.
excludeTests="clang_format"

if [ "$CI_TEST_OS" == "ros:humble-ros-base-jammy" ] || [ "$CI_TEST_OS" == "ros:iron-ros-core-jammy" ]; then
  # The listed OSes have issues invoking clang-tidy correctly:
  #   - Humble seems to not invoke it with the correct C++ language version, resulting in it not finding
  #     std::optional and std::variant.
  #   - Iron does not find the compile commands.
  # On the other hand, Jazzy seems to work well, so we get the checks when testing on this OS.
  echo "Skipping clang-tidy tests since OS is '$CI_TEST_OS'"
  excludeTests+="|clang_tidy"
fi

export GTEST_BREAK_ON_FAILURE=1;
colcon test --event-handlers console_direct+ --ctest-args tests --exclude-regex $excludeTests --output-on-failure --ros-args --log-level debug || exit $?

echo "Check for test errors"
colcon test-result --all || exit $?

echo Success! ["$(basename $0)"]
