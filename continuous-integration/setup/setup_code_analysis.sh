#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export DEBIAN_FRONTEND=noninteractive

function apt-yes {
    apt-get --assume-yes "$@"
}

apt-yes install curl gnupg2 lsb-release || exit $?

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
  > /etc/apt/sources.list.d/ros-latest.list

apt-yes update || exit $?
apt-yes dist-upgrade || exit $?

apt-yes install \
    clang-format \
    shellcheck \
    python3-pip \
    || exit $?

pip3 install -r $SCRIPT_DIR/code_analysis_requirements.txt || exit $?

echo Success! ["$(basename $0)"]
