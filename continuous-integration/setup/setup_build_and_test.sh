#!/bin/bash

echo Start ["$(basename $0)"]

UBUNTU_VERSION="$(lsb_release -rs)" || exit $?

export DEBIAN_FRONTEND=noninteractive

function apt-yes {
    apt-get --assume-yes "$@"
}

apt-yes update || exit $?
apt-yes dist-upgrade || exit $?

apt-yes install \
    alien \
    wget \
    python3-pip \
    python-catkin-tools \
    unzip \
    || exit $?

function install_opencl_cpu_runtime {
    TMP_DIR=$(mktemp --tmpdir --directory zivid-setup-opencl-cpu-XXXX) || exit $?
    pushd $TMP_DIR || exit $?
    wget -q https://www.dropbox.com/s/0cvg8fypylgal2m/opencl_runtime_16.1.1_x64_ubuntu_6.4.0.25.tgz || exit $?
    tar -xf opencl_runtime_16.1.1_x64_ubuntu_6.4.0.25.tgz || exit $?
    alien -i opencl_runtime_*/rpm/*.rpm || exit $?
    mkdir -p /etc/OpenCL/vendors || exit $?
    ls /opt/intel/opencl*/lib64/libintelocl.so > /etc/OpenCL/vendors/intel.icd || exit $?
    popd || exit $?
    rm -r $TMP_DIR || exit $?
}

install_opencl_cpu_runtime || exit $?

function install_www_deb {
    TMP_DIR=$(mktemp --tmpdir --directory install_www_deb-XXXX) || exit $?
    pushd $TMP_DIR || exit $?
    wget -q "$@" || exit $?
    echo "Installing Zivid debian package $1"
    apt-yes install --fix-broken ./*deb || exit $?
    popd || exit $?
    rm -r $TMP_DIR || exit $?
}

echo "Installing $CI_TEST_COMPILER"
if [[ "$CI_TEST_COMPILER" == "g++-7" ]] ||
   [[ "$CI_TEST_COMPILER" == "g++-8" ]] ||
   [[ "$CI_TEST_COMPILER" == "g++-9" ]]; then
    apt-yes install software-properties-common || exit $?
    add-apt-repository -y ppa:ubuntu-toolchain-r/test || exit $?
    apt-yes update || exit $?
    apt-yes install $CI_TEST_COMPILER || exit $?
elif [[ "$CI_TEST_COMPILER" == "clang++-7" ]]; then
    apt-yes install clang-7 || exit $?
else
    echo "Unhandled CI_TEST_COMPILER $CI_TEST_COMPILER"
    exit 1
fi

echo "Install Zivid debian packages"

if [[ "$UBUNTU_VERSION" == "16.04" ]]; then

    install_www_deb https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/u16/telicam-sdk_2.0.0.1-1_amd64.deb || exit $?
    install_www_deb https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/u16/zivid_1.3.0+bb9ee328-10_amd64.deb || exit $?

elif [[ "$UBUNTU_VERSION" == "18.04" ]]; then

    install_www_deb https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/u18/telicam-sdk_2.0.0.1-1_amd64.deb || exit $?
    install_www_deb https://www.zivid.com/hubfs/softwarefiles/releases/1.3.0+bb9ee328-10/u18/zivid_1.3.0+bb9ee328-10_amd64.deb || exit $?

else
    echo "Unhandled OS $UBUNTU_VERSION"
    exit 1
fi

echo Success! ["$(basename $0)"]
