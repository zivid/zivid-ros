#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?
ROOT_DIR=$(realpath "$SCRIPT_DIR/..") || exit $?

echo "Creating catkin workspace"
mkdir -p ~/catkin_ws/src || exit $?
cd ~/catkin_ws || exit $?
catkin build || exit $?

echo "Adding link to the source folder"
ln -s "$ROOT_DIR" ~/catkin_ws/src || exit $?

for package in zivid_camera zivid_samples
do
    echo "Verify that $package is found by catkin list"
    catkin list --unformatted | grep -q $package || exit $?
done

echo "Installing dependencies"
rosdep update && rosdep install --from-paths src --ignore-src -r -y || exit $?

echo "Building with compiler=$CI_TEST_COMPILER"

catkin build -DCOMPILER_WARNINGS=ON -DCMAKE_CXX_COMPILER=/usr/bin/$CI_TEST_COMPILER || exit $?

echo Success! ["$(basename $0)"]
