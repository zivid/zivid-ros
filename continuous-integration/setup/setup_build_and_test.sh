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
    clinfo \
    wget \
    python3-pip \
    unzip \
    || exit $?

if [[ "$UBUNTU_VERSION" == "20.04" ]]; then
    apt-yes install python3-catkin-tools python3-osrf-pycommon || exit $?
else
    apt-yes install python-catkin-tools || exit $?
fi

function install_opencl_cpu_runtime {
    # Download the key to system keyring
    INTEL_KEY_URL=https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
    wget -O- $INTEL_KEY_URL | gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null || exit $?

    # Add signed entry to apt sources and configure the APT client to use Intel repository
    echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | tee /etc/apt/sources.list.d/oneAPI.list || exit $?
    apt update || exit $?

    # Install the OpenCL runtime
    # TODO: remove libxml2 once Intel sorts out its package dependencies
    apt --assume-yes install libxml2 intel-oneapi-runtime-opencl-2024 intel-oneapi-runtime-compilers-2024 || exit $?
}

install_opencl_cpu_runtime || exit $?

echo "clinfo:"
clinfo || exit $?

function install_www_deb {
    TMP_DIR=$(mktemp --tmpdir --directory install_www_deb-XXXX) || exit $?
    pushd $TMP_DIR || exit $?
    echo "Downloading Zivid debian package $1"
    wget -q "$@" || exit $?
    echo "Installing Zivid debian package $1"
    apt-yes install --fix-broken ./*deb || exit $?
    popd || exit $?
    rm -r $TMP_DIR || exit $?
}

echo "Installing compiler $CI_TEST_COMPILER"

if [[ "$CI_TEST_COMPILER" == "g++"    ||
      "$CI_TEST_COMPILER" == "g++-7"  ||
      "$CI_TEST_COMPILER" == "g++-8"  ||
      "$CI_TEST_COMPILER" == "g++-9"  ||
      "$CI_TEST_COMPILER" == "g++-10" ||
      "$CI_TEST_COMPILER" == "g++-11"
      ]]; then

    apt-yes install software-properties-common || exit $?
    add-apt-repository -y ppa:ubuntu-toolchain-r/test || exit $?
    apt-yes update || exit $?
    apt-yes install $CI_TEST_COMPILER || exit $?

elif [[ "$CI_TEST_COMPILER" == "clang++"    ||
        "$CI_TEST_COMPILER" == "clang++-7"  ||
        "$CI_TEST_COMPILER" == "clang++-8"  ||
        "$CI_TEST_COMPILER" == "clang++-9"  ||
        "$CI_TEST_COMPILER" == "clang++-10" ||
        "$CI_TEST_COMPILER" == "clang++-11" ||
        "$CI_TEST_COMPILER" == "clang++-12" ]]; then

    apt-yes install ${CI_TEST_COMPILER//\+/} || exit $?

else
    echo "Unhandled CI_TEST_COMPILER $CI_TEST_COMPILER"
    exit 1
fi

echo "Install Zivid and Telicam debian packages"

ZIVID_RELEASE_DIR="https://downloads.zivid.com/sdk/releases/$CI_TEST_ZIVID_VERSION"
ZIVID_TELICAM_SDK_DEB="zivid-telicam-driver_3.0.1.1-3_amd64.deb"

if [[ "$UBUNTU_VERSION" == "18.04" ]]; then

    install_www_deb "$ZIVID_RELEASE_DIR/u18/${ZIVID_TELICAM_SDK_DEB}" || exit $?
    install_www_deb "$ZIVID_RELEASE_DIR/u18/zivid_${CI_TEST_ZIVID_VERSION}_amd64.deb" || exit $?

elif [[ "$UBUNTU_VERSION" == "20.04" ]]; then

    install_www_deb "$ZIVID_RELEASE_DIR/u20/${ZIVID_TELICAM_SDK_DEB}" || exit $?
    install_www_deb "$ZIVID_RELEASE_DIR/u20/zivid_${CI_TEST_ZIVID_VERSION}_amd64.deb" || exit $?

else

    echo "Unhandled OS $UBUNTU_VERSION"
    exit 1

fi

echo Success! ["$(basename $0)"]
