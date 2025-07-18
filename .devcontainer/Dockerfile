ARG DOCKER_REPO=osrf/ros
ARG ROS_DISTRO=jazzy
ARG IMAGE_SUFFIX=-desktop-full

FROM ${DOCKER_REPO}:${ROS_DISTRO}${IMAGE_SUFFIX}

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ZIVID_VERSION

# Setup the user
RUN apt-get update && apt-get install -y sudo && \
    # Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
    if id -u $USER_UID &> /dev/null; then \
        userdel -f $(id -un $USER_UID); \
    fi && \
    # Create the user and group
    groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    # Add sudo support
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Install APT packages
RUN apt-get update && \
    apt-get install --assume-yes \
        clinfo \
        ninja-build \
        ocl-icd-libopencl1 \
        python3-pip \
        unzip \
        wget

ENV SHELL=/bin/bash

# Setup OpenCL for Nvidia
RUN mkdir --parents /etc/OpenCL/vendors && \
    echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

# Download and install Zivid SDK
RUN wget -q --tries=3 https://downloads.zivid.com/sdk/releases/${ZIVID_VERSION}/u24/amd64/zivid_${ZIVID_VERSION}_amd64.deb && \
    apt-get update && \
    apt-get install --assume-yes ./zivid_${ZIVID_VERSION}_amd64.deb && \
    rm ./zivid_${ZIVID_VERSION}_amd64.deb

# Download sample data for running tests
RUN mkdir -p /usr/share/Zivid/data/
RUN for sample in "FileCameraZivid2M70.zip" "BinWithCalibrationBoard.zip"; do \
        wget -q --tries=3 "https://www.zivid.com/software/${sample}" && \
        unzip -q "${sample}" -d /usr/share/Zivid/data/ && \
        rm "${sample}"; \
    done

USER $USERNAME
CMD ["/bin/bash"]
