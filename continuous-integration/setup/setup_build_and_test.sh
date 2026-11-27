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
    python3-rosdep \
    python3-colcon-common-extensions \
    unzip \
    || exit $?

function install_opencl_cpu_runtime {
    # Download the key to system keyring
    INTEL_KEY_URL=https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
    wget -O- $INTEL_KEY_URL | gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null || exit $?

    # Add signed entry to apt sources and configure the APT client to use Intel repository
    echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | tee /etc/apt/sources.list.d/oneAPI.list || exit $?
    apt update || exit $?

    # Install the OpenCL runtime. The version is pinned because runtime 2026.2.0 and newer (OpenCL 3.1)
    # crashes with a segmentation fault on the first capture on AMD Zen 3 CPUs, such as the AMD EPYC 7763
    # used by many GitHub-hosted runners. All runtime packages are pinned, since their dependencies only
    # require a minimum version and would otherwise pull in the newest CPU device implementation.
    INTEL_OPENCL_RUNTIME_VERSION=2026.1.1-325
    apt --assume-yes install \
        intel-oneapi-runtime-opencl=$INTEL_OPENCL_RUNTIME_VERSION \
        intel-oneapi-runtime-dpcpp-sycl-opencl-cpu=$INTEL_OPENCL_RUNTIME_VERSION \
        intel-oneapi-runtime-compilers=$INTEL_OPENCL_RUNTIME_VERSION \
        intel-oneapi-runtime-compilers-common=$INTEL_OPENCL_RUNTIME_VERSION \
        intel-oneapi-runtime-openmp=$INTEL_OPENCL_RUNTIME_VERSION \
        || exit $?
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
      "$CI_TEST_COMPILER" == "g++-12"  ||
      "$CI_TEST_COMPILER" == "g++-13"  ||
      "$CI_TEST_COMPILER" == "g++-14"  ||
      "$CI_TEST_COMPILER" == "g++-15"  ||
      "$CI_TEST_COMPILER" == "g++-16"
      ]]; then

    apt-yes install software-properties-common || exit $?
    add-apt-repository -y ppa:ubuntu-toolchain-r/test || exit $?
    apt-yes update || exit $?
    apt-yes install $CI_TEST_COMPILER || exit $?

elif [[ "$CI_TEST_COMPILER" == "clang++"    ||
        "$CI_TEST_COMPILER" == "clang++-14"  ||
        "$CI_TEST_COMPILER" == "clang++-15"  ||
        "$CI_TEST_COMPILER" == "clang++-16"  ||
        "$CI_TEST_COMPILER" == "clang++-17" ||
        "$CI_TEST_COMPILER" == "clang++-18" ||
        "$CI_TEST_COMPILER" == "clang++-19" ||
        "$CI_TEST_COMPILER" == "clang++-20" ||
        "$CI_TEST_COMPILER" == "clang++-21" ||
        "$CI_TEST_COMPILER" == "clang++-22" ]]; then

    apt-yes install ${CI_TEST_COMPILER//\+/} || exit $?

else
    echo "Unhandled CI_TEST_COMPILER \"$CI_TEST_COMPILER\""
    exit 1
fi

echo "Install Zivid debian packages"

ZIVID_RELEASE_DIR="https://downloads.zivid.com/sdk/releases/$CI_TEST_ZIVID_VERSION"

if [[ "$UBUNTU_VERSION" == "20.04" || "$UBUNTU_VERSION" == "22.04" || "$UBUNTU_VERSION" == "24.04" || "$UBUNTU_VERSION" == "26.04" ]]; then

    if [[ "$CI_TEST_ZIVID_VERSION" < "2.18.0" ]]; then
        install_www_deb "$ZIVID_RELEASE_DIR/u20/zivid_${CI_TEST_ZIVID_VERSION}_amd64.deb" || exit $?
    else
        install_www_deb "$ZIVID_RELEASE_DIR/u20/zivid-opencl_${CI_TEST_ZIVID_VERSION}_amd64.deb" || exit $?
    fi

else

    echo "Unhandled Ubuntu OS $UBUNTU_VERSION"
    exit 1

fi

echo Success! ["$(basename $0)"]
