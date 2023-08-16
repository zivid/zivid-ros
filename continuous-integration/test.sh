#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?

cd ~/catkin_ws || exit $?

echo "Installing Zivid API config file"
install -D "$SCRIPT_DIR"/ZividAPIConfigCPU.yml "$HOME"/.config/Zivid/API/Config.yml || exit $?

echo "Download and install zivid sample data (file camera)"
wget -q https://www.zivid.com/software/FileCameraZivid2M70.zip || exit $?
mkdir -p /usr/share/Zivid/data/ || exit $?
unzip ./FileCameraZivid2M70.zip -d /usr/share/Zivid/data/ || exit $?
rm ./FileCameraZivid2M70.zip || exit $?

echo "Running tests"
catkin run_tests -DCOMPILER_WARNINGS=ON -DCMAKE_CXX_COMPILER=/usr/bin/$CI_TEST_COMPILER  || exit $?

echo "Check for test errors"
catkin_test_results || exit $?

echo Success! ["$(basename $0)"]
