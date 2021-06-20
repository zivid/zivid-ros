#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?
ROOT_DIR=$(realpath "$SCRIPT_DIR/..") || exit $?

echo "Creating catkin workspace"
mkdir -p ~/catkin_ws || exit $?
cd ~/catkin_ws || exit $?
catkin init || exit $?

echo "SCRIPT_DIR is $SCRIPT_DIR"
echo "ROOT_DIR is $ROOT_DIR"

echo "Adding link to the source folder"
ln -s "$ROOT_DIR" ~/catkin_ws/src || exit $?

echo "ls -l ROOT_DIR"
echo "*************************"
ls -l "$ROOT_DIR"
echo "*************************"

echo "*************************"
echo "Doing ls -l ~/catkin_ws/"
ls -l ~/catkin_ws/
echo "*************************"


echo "*************************"
echo "Doing ls -l ~/catkin_ws/src"
ls -l ~/catkin_ws/src
echo "*************************"

echo "*************************"
echo "Doing ls -l ~/catkin_ws/src/"
ls -l ~/catkin_ws/src/
echo "*************************"

echo "*************************"
echo "Doing ls -l ~/catkin_ws/src/*"
ls -l ~/catkin_ws/src/*
echo "*************************"


for package in zivid_camera zivid_samples
do
    echo "Verifying that $package is found by catkin list"
    catkin list --unformatted | grep -q $package || exit $?
done

echo "Installing dependencies"
rosdep update && rosdep install --from-paths src --ignore-src -r -y || exit $?

echo "Building with compiler=$CI_TEST_COMPILER"

catkin build -DCOMPILER_WARNINGS=ON -DCMAKE_CXX_COMPILER=/usr/bin/$CI_TEST_COMPILER || exit $?

echo Success! ["$(basename $0)"]
