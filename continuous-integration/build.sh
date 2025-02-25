#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?
ROOT_DIR=$(realpath "$SCRIPT_DIR/..") || exit $?

echo "Creating workspace"
mkdir -p ~/ros2_ws/src || exit $?
cd ~/ros2_ws || exit $?

if [ -z "$(ls --almost-all ~/ros2_ws/src)" ]; then
  echo "Adding link to the source folder"
  ln -s "$ROOT_DIR" ~/ros2_ws/src || exit $?
else
  echo "Skipping link to source folder since it already exists"
fi

if [ "$ROS_DISTRO" != "humble" ]; then
  echo "Initializing rosdep"
  rosdep init || exit $?
fi

echo "Updating rosdep"
rosdep update || exit $?

echo "Installing dependencies"
rosdep install -i --from-path src -y || exit $?

echo "Building with compiler=$CI_TEST_COMPILER"

colcon build --symlink --event-handlers console_direct+ --cmake-args -DCOMPILER_WARNINGS=ON -DCMAKE_CXX_COMPILER=/usr/bin/$CI_TEST_COMPILER -DCMAKE_EXPORT_COMPILE_COMMANDS=ON || exit $?

echo "Check that the expected packages are found"
source ~/ros2_ws/install/setup.bash
for package in zivid_camera zivid_interfaces zivid_samples
do
    echo "Check that $package is found by ros2 pkg list"
    ros2 pkg list | grep -q $package || exit $?
done

echo Success! ["$(basename $0)"]
