# Zivid ROS driver

This is the official ROS driver for [Zivid 3D cameras](https://www.zivid.com/).

**Note:** Current version is 0.9.0. API and behavior are still subject to changes. When 1.0.0 is
released the API will be stable.

[![Build Status][ci-badge]][ci-url]

---

*Contents:*
[**Installation**](#installation) |
[**Getting Started**](#getting-started) |
[**Launching**](#launching-the-driver) |
[**Services**](#services) |
[**Topics**](#topics) |
[**Configuration**](#configuration) |
[**Samples**](#samples) |
[**FAQ**](#frequently-asked-questions)

---

<p align="center">
    <img src="https://www.zivid.com/software/zivid-ros/ros_zivid_camera.png" width="600" height="273">
</p>

## Installation

### ROS

This driver supports Ubuntu 16.04 with ROS Kinetic and Ubuntu 18.04 with ROS Melodic. Follow the
official ROS Wiki instructions to install [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation) (for Ubuntu 16.04)
or [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation) (for Ubuntu 18.04).

Also install catkin and git:

```bash
sudo apt-get update
sudo apt-get install -y python-catkin-tools git
```

### Zivid SDK

To use the ROS driver you need to download and install the "Zivid Core" package.
Follow [this guide](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/59080712/Zivid+Software+Installation)
to install Zivid for your version of Ubuntu. "Zivid Studio" and "Zivid Tools" packages are
not required by the ROS driver, but can be useful for testing that your system has been setup correctly
and that the camera is detected.

An OpenCL 1.2 compatible GPU and OpenCL driver is required by the Zivid SDK. Follow
[this guide](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/426519/Install+OpenCL+drivers+on+Ubuntu) to
install OpenCL drivers for your system.

### C++ compiler

A C++17 compiler is required.

Ubuntu 16.04:
```bash
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install -y g++-8
```

Ubuntu 18.04:
```bash
sudo apt-get install -y g++
```

### Downloading and building Zivid ROS driver

First, load the setup.bash script into the current session.

Ubuntu 16.04:
```bash
source /opt/ros/kinetic/setup.bash
```

Ubuntu 18.04:
```bash
source /opt/ros/melodic/setup.bash
```

Create the workspace and src directory:
```bash
mkdir -p ~/catkin_ws/src
```

Clone the Zivid ROS project into the src directory:
```bash
cd ~/catkin_ws/src
git clone https://github.com/zivid/zivid-ros.git
```

Install dependencies:
```bash
cd ~/catkin_ws
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Finally, build the driver.

Ubuntu 16.04:
```bash
catkin build -DCMAKE_CXX_COMPILER=/usr/bin/g++-8
```

Ubuntu 18.04:
```bash
catkin build
```

## Getting started

Connect the Zivid camera to your USB3 port on your PC. You can use the ZividListCameras command-line
tool available in the "Zivid Tools" package to confirm that your system has been configured correctly, and
that the camera is discovered by your PC. You can also open Zivid Studio and connect to the camera.
Close Zivid Studio before continuing with the rest of this guide.

Launch sample_capture_cpp to test that everything is working:

```bash
cd ~/catkin_ws && source devel/setup.bash
roslaunch zivid_samples sample.launch type:=sample_capture_cpp
```

This launch file starts the `zivid_camera` driver node, the `sample_capture_cpp` node, as well as
[rviz](https://wiki.ros.org/rviz) to visualize the point cloud and the 2D color and depth images
and [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) to adjust the camera settings.

The `sample_capture_cpp` node will first configure the capture settings of the camera and then
trigger captures repeatedly. If everything is working, the output should be visible in rviz. Try to
adjust the exposure time or the iris in rqt_reconfigure and observe that the visualization in
rviz changes. Note: sometimes it is necessary to click "Refresh" in rqt_reconfigure to load the
configuration tree.

<p align="center">
    <img src="https://www.zivid.com/software/zivid-ros/ros_rviz_miscobjects.png?" width="750" height="445">
</p>

A more detailed description of the `zivid_camera` driver follows below.

For sample code in C++ and Python, see the [Samples](#samples) section.

## Launching the driver

It is required to specify a [namespace](http://wiki.ros.org/Nodes#Remapping_Arguments.A.22Pushing_Down.22)
when starting the driver. All the services, topics and configurations will be pushed into this namespace.

To launch the driver, first start `roscore` in a seperate terminal window:

```bash
roscore
```

In another terminal run:
```bash
cd ~/catkin_ws && source devel/setup.bash
```

Then, launch the driver either as a node or a [nodelet](http://wiki.ros.org/nodelet). To launch as a node:

```bash
ROS_NAMESPACE=zivid_camera rosrun zivid_camera zivid_camera_node
```

To launch as a [nodelet](http://wiki.ros.org/nodelet):

```bash
ROS_NAMESPACE=zivid_camera rosrun nodelet nodelet standalone zivid_camera/nodelet
```

### Launch Parameters (advanced)

The following parameters can be specified when starting the driver. Note that all the parameters are
optional, and typically not required to set.

For example, to launch the driver with `frame_id` specified, append `_frame_id:=camera1` to the
rosrun command:

```bash
ROS_NAMESPACE=zivid_camera rosrun zivid_camera zivid_camera_node _frame_id:=zivid
```

`file_camera_path` (string, default: "")
> Specify the path to a file camera to use instead of a real Zivid camera. This can be used to
> develop without access to hardware. The file camera returns the same point cloud for every capture.
> [Click here to download a file camera.](https://www.zivid.com/software/ZividSampleData.zip)

`frame_id` (string, default: "zivid_optical_frame")
> Specify the frame_id used for all published images and point clouds.

`num_capture_frames` (int, default: 10)
> Specify the number of dynamic_reconfigure `capture/frame_<n>` nodes that are created. This number
> defines the maximum number of frames that can be a part of a 3D HDR capture. All `capture/frame_<n>`
> nodes are by default enabled=false (see section [Configuration](#configuration)). If you need to
> perform 3D HDR capture with more than 10 enabled frames then increase this number. Otherwise it can
> be left as default.

`serial_number` (string, default: "")
> Specify the serial number of the Zivid camera to use. Important: When passing this value via
> the command line or rosparam the serial number must be prefixed with a colon (`:12345`).
> This parameter is optional. By default the driver will connect to the first available camera.

## Services

`capture` ([zivid_camera/Capture](./zivid_camera/srv/Capture.srv))
> Invoke this service to trigger a 3D capture. See section [Configuration](#configuration) for how to
> configure the 3D capture settings. The resulting point cloud is published on topic `points`, color image
> is published on topic `color/image_color`, and depth image is published on topic `depth/image_raw`.
> Camera calibration is published on topics `color/camera_info` and `depth/camera_info`.

`capture_2d` ([zivid_camera/Capture2D](./zivid_camera/srv/Capture2D.srv))
> Invoke this service to trigger a 2D capture. See section [Configuration](#configuration) for how to
> configure the 2D capture settings. The resulting 2D image is published to topic `color/image_color`.
> Note: 2D RGB image is also published as a part of 3D capture (see `capture` above.)

`camera_info/model_name` ([zivid_camera/CameraInfoModelName](./zivid_camera/srv/CameraInfoModelName.srv))
> Returns the camera's model name.

`camera_info/serial_number` ([zivid_camera/CameraInfoSerialNumber](./zivid_camera/srv/CameraInfoSerialNumber.srv))
> Returns the camera's serial number.

`is_connected` ([zivid_camera/IsConnected](./zivid_camera/srv/IsConnected.srv))
> Returns if the camera is currently in `Connected` state (from the perspective of the ROS driver).
> The connection status is updated by the driver every 10 seconds and before each `capture` service
> call. If the camera is not in `Connected` state the driver will attempt to re-connect to the camera
> when it detects that the camera is available. This can happen if the camera is power-cycled or the
> USB cable is unplugged and then replugged.

## Topics

`color/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
> Camera calibration and metadata.

`color/image_color` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
> Color/RGB image. For 3D captures (triggered via `capture` service) the image is encoded as "rgb8".
> For 2D captures (triggered via `capture_2d` service) the image is encoded as "rgba8", where the
> alpha channel is always 255.

`depth/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
> Camera calibration and metadata.

`depth/image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
> Depth image. Each pixel contains the z-value (along the camera Z axis) in meters.
> The image is encoded as 32-bit float. Pixels where z-value is missing are NaN.

`points` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))
> Point cloud data. Each time a capture is invoked the resulting point cloud is published
> on this topic. The included point fields are x, y, z (in meters), c (contrast value),
> and r, g, b (colors). The output is in the camera's optical frame, where x is right, y is
> down and z is forward.

## Configuration

The `zivid_camera` node supports both single-capture (2D and 3D) and HDR-capture (3D). 3D HDR-capture works by taking
several individual captures (called frames) with different settings (for example different exposure time)
and combining the captures into one high-quality point cloud. For more information about HDR capture, visit our
[knowledge base](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/428143/HDR+Imaging+for+Challenging+Objects).

The capture settings available in the `zivid_camera` node matches the settings in the Zivid API.
To become more familiar with the available settings and what they do, see the API reference for the
[Settings](http://www.zivid.com/hubfs/softwarefiles/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings.html)
and [Settings2D](http://www.zivid.com/hubfs/softwarefiles/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings2D.html)
classes, or use Zivid Studio.

The settings can be configured using [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure).
Use [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) to view or change the settings using a GUI.

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Note that the min, max and default value of the settings can change dependent on what Zivid camera
model you are using. Therefore you should **not** use the static `__getMin()__`, `__getMax()__` and
`__getDefault()__` methods of the auto-generated C++ config classes (`zivid_camera::CaptureGeneralConfig`,
`zivid_camera::CaptureFrameConfig` and `zivid_camera::Capture2DFrameConfig`). Instead, you should
query the server for the default values using `dynamic_reconfigure::Client<T>::getDefaultConfiguration()`.
See the [C++ samples](#samples) for how to do this.

The available capture settings are organized into a hierarchy of configuration nodes. 3D settings are available
under the `/capture` namespace, while 2D settings are available under `/capture_2d`.

```
/capture
    /frame_0
        ...
    /frame_1
        ...
    ...
    /frame_9
        ...
    /general
        ...
/capture_2d
    /frame_0
```

### Frame settings for 3D

`capture/frame_<n>/` contains settings for an individual frame. By default `<n>` can be 0 to 9 for a
total of 10 frames. The total number of frames can be configured using the launch parameter `num_capture_frames`
(see section [Launch Parameters](#launch-parameters-advanced) above).

`capture/frame_<n>/enabled` controls if frame `<n>` will be included when the `capture/` service is
invoked. If only one frame is enabled the `capture/` service performs a 3D single-capture. If more than
one frame is enabled the `capture/` service will perform a 3D HDR-capture. By default enabled is false.
In order to capture a point cloud at least one frame needs to be enabled.

| Name                               | Type   |  Zivid API Setting             |      Note        |
|------------------------------------|--------|--------------------------------|------------------|
| `capture/frame_<n>/bidirectional`  | bool   | [Zivid::Settings::Bidirectional](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Bidirectional.html)
| `capture/frame_<n>/brightness`     | double | [Zivid::Settings::Brightness](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Brightness.html)
| `capture/frame_<n>/enabled`        | bool   |  |
| `capture/frame_<n>/exposure_time`  | int    | [Zivid::Settings::ExposureTime](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1ExposureTime.html) | Specified in microseconds (µs)
| `capture/frame_<n>/gain`           | double | [Zivid::Settings::Gain](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Gain.html)
| `capture/frame_<n>/iris`           | int    | [Zivid::Settings::Iris](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Iris.html)

### General settings for 3D

`capture/general` contains settings that apply to all frames in a 3D capture.

| Name                                          | Type   |  Zivid API Setting                     |
|-----------------------------------------------|--------|----------------------------------------|
| `capture/general/blue_balance`                | double | [Zivid::Settings::BlueBalance](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1BlueBalance.html)
| `capture/general/filters_contrast_enabled`    | bool   | [Zivid::Settings::Filters::Contrast::Enabled](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Contrast_1_1Enabled.html)
| `capture/general/filters_contrast_threshold`  | double | [Zivid::Settings::Filters::Contrast::Threshold](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Contrast_1_1Threshold.html)
| `capture/general/filters_gaussian_enabled`    | bool   | [Zivid::Settings::Filters::Gaussian::Enabled](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Gaussian_1_1Enabled.html)
| `capture/general/filters_gaussian_sigma`      | double | [Zivid::Settings::Filters::Gaussian::Sigma](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Gaussian_1_1Sigma.html)
| `capture/general/filters_outlier_enabled`     | bool   | [Zivid::Settings::Filters::Outlier::Enabled](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Outlier_1_1Enabled.html)
| `capture/general/filters_outlier_threshold`   | double | [Zivid::Settings::Filters::Outlier::Threshold](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Outlier_1_1Threshold.html)
| `capture/general/filters_reflection_enabled`  | bool   | [Zivid::Settings::Filters::Reflection::Enabled](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Reflection_1_1Enabled.html)
| `capture/general/filters_saturated_enabled`   | bool   | [Zivid::Settings::Filters::Saturated::Enabled](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1Filters_1_1Saturated_1_1Enabled.html)
| `capture/general/red_balance`                 | double | [Zivid::Settings::RedBalance](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings_1_1RedBalance.html)

### Frame settings for 2D

2D settings are available under `capture_2d/frame_0/`. To trigger a 2D capture, invoke the `capture_2d`
service. Note that `capture_2d/frame_0/enabled` is default false, and must be set to true before
calling `capture_2d` service, otherwise the service will return an error.

| Name                               | Type   |  Zivid API Setting             |      Note        |
|------------------------------------|--------|--------------------------------|------------------|
| `capture_2d/frame_0/brightness`    | double | [Zivid::Settings2D::Brightness](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings2D_1_1Brightness.html)
| `capture_2d/frame_0/enabled`       | bool   | |
| `capture_2d/frame_0/exposure_time` | int    | [Zivid::Settings2D::ExposureTime](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings2D_1_1ExposureTime.html) | Specified in microseconds (µs)
| `capture_2d/frame_0/gain`          | double | [Zivid::Settings2D::Gain](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings2D_1_1Gain.html)
| `capture_2d/frame_0/iris`          | int    | [Zivid::Settings2D::Iris](https://www.zivid.com/software/releases/1.7.0+a115eaa4-4/doc/cpp/classZivid_1_1Settings2D_1_1Iris.html)

## Samples

In the `zivid_samples` package we have added samples in C++ and Python that demonstrate how to use
the Zivid ROS driver. These samples can be used as a starting point for your project.

### Sample Capture

This sample performs single 3D captures repeatedly. This sample shows how to [configure](#configuration)
the capture settings, how to subscribe to the `points` topic, and how to invoke the `capture` service.

Source code: [C++](./zivid_samples/src/sample_capture.cpp), [Python](./zivid_samples/scripts/sample_capture.py)

Using roslaunch (also launches `roscore`, `zivid_camera`, `rviz` and `rqt_reconfigure`):
```bash
roslaunch zivid_samples sample.launch type:=sample_capture_cpp
roslaunch zivid_samples sample.launch type:=sample_capture.py
```
Using rosrun (when `roscore` and `zivid_camera` are running):
```bash
rosrun zivid_samples sample_capture_cpp
rosrun zivid_samples sample_capture.py
```

### Sample Capture 2D

This sample performs 2D captures repeatedly. This sample shows how to [configure](#configuration)
the 2D capture settings, how to subscribe to the `color/image_color` topic, and how to invoke
the `capture_2d` service.

Source code: [C++](./zivid_samples/src/sample_capture_2d.cpp), [Python](./zivid_samples/scripts/sample_capture_2d.py)

Using roslaunch (also launches `roscore`, `zivid_camera`, `rviz` and `rqt_reconfigure`):
```bash
roslaunch zivid_samples sample.launch type:=sample_capture_2d_cpp
roslaunch zivid_samples sample.launch type:=sample_capture_2d.py
```
Using rosrun (when `roscore` and `zivid_camera` are running):
```bash
rosrun zivid_samples sample_capture_2d_cpp
rosrun zivid_samples sample_capture_2d.py
```

## Sample .launch files

### zivid_camera_with_settings.launch

[zivid_camera_with_settings.launch](./zivid_samples/launch/zivid_camera_with_settings.launch) starts
the `zivid_camera` node with 3D capture settings set to 3-frame HDR. Modify the settings in this .launch
file to match your scene.

```bash
roslaunch zivid_samples zivid_camera_with_settings.launch
```

## Frequently Asked Questions

### How to visualize the output from the camera in rviz

```bash
ROS_NAMESPACE=zivid_camera roslaunch zivid_camera visualize.launch
```

### How to use multiple cameras

You can use multiple Zivid cameras simultaneously by starting one node per camera and specifying
unique namespaces per node:

```bash
ROS_NAMESPACE=camera1 rosrun zivid_camera zivid_camera_node
```

```bash
ROS_NAMESPACE=camera2 rosrun zivid_camera zivid_camera_node
```

By default the zivid_camera node will connect to the first available/unused camera. We recommend that
you first start the first node, wait for it to be ready (for example, by waiting for the `capture`
service to be available), then start the second node. This avoids any race conditions where both nodes
may try to connect to the same camera at the same time.

### How to run the unit and module tests

This project comes with a set of unit and module tests to verify the provided functionality. To run
the tests locally, first download and install the file camera used for testing:
```bash
wget -q https://www.zivid.com/software/ZividSampleData.zip
unzip ./ZividSampleData.zip
rm ./ZividSampleData.zip
sudo mkdir -p /usr/share/Zivid/data/
sudo cp ./MiscObjects.zdf /usr/share/Zivid/data/
rm ./MiscObjects.zdf
```

Then run the tests:
```bash
cd ~/catkin_ws && source devel/setup.bash
catkin run_tests && catkin_test_results ~/catkin_ws
```

The tests can also be run via [docker](https://www.docker.com/). See the
[Azure Pipelines configuration file](./azure-pipelines.yml) for details.

### How to enable debug logging

The node logs extra information at log level debug, including the settings used when capturing.
Enable debug logging to troubleshoot issues.

```bash
rosconsole set /<namespace>/zivid_camera ros.zivid_camera debug
```

For example, if ROS_NAMESPACE=zivid_camera,

```bash
rosconsole set /zivid_camera/zivid_camera ros.zivid_camera debug
```

### How to compile the project with warnings enabled

```bash
catkin build -DCOMPILER_WARNINGS=ON
```

## License

This project is licensed under BSD 3-clause license, see the [LICENSE](LICENSE) file for details.

## Support

Please report any issues or feature requests related to the ROS driver in the issue tracker.
Visit [Zivid Knowledge Base](http://help.zivid.com) for general help on using Zivid 3D
cameras. If you cannot find a solution to your issue, please contact support@zivid.com.

## Acknowledgements

<img src="https://www.zivid.com/software/zivid-ros/rosin_logo.png">

This FTP (Focused Technical Project) has received funding from the European Union's
Horizon 2020 research and innovation programme under the project ROSIN with the
grant agreement No 732287. For more information, visit [rosin-project.eu](http://rosin-project.eu/).


[ci-badge]: https://img.shields.io/azure-devops/build/zivid-devops/376f5fda-ba80-4d6c-aaaa-cbcd5e0ad6c0/2/master.svg
[ci-url]: https://dev.azure.com/zivid-devops/zivid-ros/_build/latest?definitionId=1&branchName=master