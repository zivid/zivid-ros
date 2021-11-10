# Zivid ROS driver

This is the official ROS driver for [Zivid 3D cameras](https://www.zivid.com/).

[![Build Status][ci-badge]][ci-url]
![Zivid Image][header-image]

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

## Installation

### ROS

This driver supports Ubuntu 16.04 with ROS Kinetic, Ubuntu 18.04 with ROS Melodic and Ubuntu 20.04
with ROS Noetic. Follow the official [ROS installation instructions](http://wiki.ros.org/ROS/Installation) for
your OS.

Also install catkin and git:

Ubuntu 16.04/18.04:
```bash
sudo apt-get update
sudo apt-get install -y python-catkin-tools git
```

Ubuntu 20.04:
```bash
sudo apt-get update
sudo apt-get install -y python3-catkin-tools python3-osrf-pycommon git
```

### Zivid SDK

To use the ROS driver you need to download and install the "Zivid Core" package. Zivid SDK version 2.5
is supported. See [releases](https://github.com/zivid/zivid-ros/releases) for older ROS driver releases that
supports older SDK versions.

Follow [this guide][zivid-software-installation-url]
to install "Zivid Core" for your version of Ubuntu. The "Zivid Studio" and "Zivid Tools" packages can be useful
to test your system setup and camera, but are not required by the driver.

An OpenCL 1.2 compatible GPU with driver installed is required by the SDK. Follow
[this guide][install-opencl-drivers-ubuntu-url] to install OpenCL drivers for your system.

### C++ compiler

A C++17 compiler is required.

Ubuntu 16.04:
```bash
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install -y g++-8
```

Ubuntu 18.04/20.04:
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

Ubuntu 20.04:
```bash
source /opt/ros/noetic/setup.bash
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

Ubuntu 18.04/20.04:
```bash
catkin build
```

## Getting started

Connect the Zivid camera to your USB3 port on your PC. You can use the ZividListCameras command-line
tool available in the "Zivid Tools" package to confirm that your system has been configured correctly, and
that the camera is discovered by your PC. You can also open Zivid Studio and connect to the camera.
Close Zivid Studio before continuing with the rest of this guide.

Launch `sample_capture_assistant.py` to test that everything is working:

```bash
cd ~/catkin_ws && source devel/setup.bash
roslaunch zivid_samples sample.launch type:=sample_capture_assistant.py
```

This will start the `zivid_camera` driver node, the
[sample_capture_assistant.py](./zivid_samples/scripts/sample_capture_assistant.py) node, as well as
[rviz](https://wiki.ros.org/rviz) to visualize the point cloud and the 2D color and depth images
and [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) to view or change capture settings.

The `sample_capture_assistant.py` node will first call the
[capture_assistant/suggest_settings](#capture_assistantsuggest_settings) service to find suggested
capture settings for your particular scene, then call the [capture](#capture) service to
capture using those settings. If everything is working, the point cloud, color image and depth image
should be visible in rviz.

You can adjust the maximum capture time by changing variable `max_capture_time` in
[sample_capture_assistant.py](./zivid_samples/scripts/sample_capture_assistant.py) and
re-launching the sample.

<p align="center">
    <img src="https://www.zivid.com/software/zivid-ros/ros_rviz_miscobjects_sdk2.png?" width="750" height="436">
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

For example, to run the zivid_camera driver with `frame_id` specified, append `_frame_id:=camera1` to the
`rosrun` command:

```bash
ROS_NAMESPACE=zivid_camera rosrun zivid_camera zivid_camera_node _frame_id:=camera1
```

Or, if using `roslaunch` specify the parameter using `<param>`:

```xml
<launch>
    <node name="zivid_camera" pkg="zivid_camera" type="zivid_camera_node" ns="zivid_camera" output="screen">
        <param type="string" name="frame_id" value="camera_1" />
    </node>
</launch>
```

`file_camera_path` (string, default: "")
> Specify the path to a file camera to use instead of a real Zivid camera. This can be used to
> develop without access to hardware. The file camera returns the same point cloud for every capture.
> [Click here to download a file camera.](https://www.zivid.com/software/FileCameraZividOne.zip)

`frame_id` (string, default: "zivid_optical_frame")
> Specify the frame_id used for all published images and point clouds.

`max_capture_acquisitions` (int, default: 10)
> Specify the number of dynamic_reconfigure `settings/acquisition_<n>` nodes that are created. This number
> defines the maximum number of acquisitions that can be a part of a 3D capture. All `settings/acquisition_<n>`
> nodes are by default enabled=false (see section [Configuration](#configuration)). If you need to
> perform 3D HDR capture with more than 10 enabled acquisitions then increase this number. Otherwise it can
> be left as default. We do not recommend lowering this setting, especially if you are using the
> [capture_assistant/suggest_settings](#capture_assistantsuggest_settings) service.

`serial_number` (string, default: "")
> Specify the serial number of the Zivid camera to use. Important: When passing this value via
> the command line or rosparam the serial number must be prefixed with a colon (`:12345`).
> This parameter is optional. By default the driver will connect to the first available camera.

`update_firmware_automatically` (bool, default: true)
> Specify if the firmware of the connected camera should be automatically updated to the correct
> version when the Zivid ROS driver starts. If set to false, if the firmware version is out of date
> then camera must be manually updated, for example using Zivid Studio or `ZividFirmwareUpdater`.
> Read more about [firmware update in our knowledgebase][firmware-update-kb-url].
> This parameter is optional, and by default it is true.

## Services

### capture_assistant/suggest_settings
[zivid_camera/CaptureAssistantSuggestSettings.srv](./zivid_camera/srv/CaptureAssistantSuggestSettings.srv)

Invoke this service to analyze your scene and find suggested settings for your particular scene,
camera distance, ambient lighting conditions, etc. The suggested settings are configured on this
node and accessible via dynamic_reconfigure, see section [Configuration](#configuration). When this
service has returned you can invoke the [capture](#capture) service to trigger a 3D capture using
these suggested settings.

This service has two parameters:

`max_capture_time` (duration):
> Specify the maximum capture time for the settings suggested by the Capture Assistant. A longer
> capture time may be required to get good data for more challenging scenes. Minimum value is
> 0.2 sec and maximum value is 10.0 sec.

`ambient_light_frequency` (uint8):
> Possible values are: `AMBIENT_LIGHT_FREQUENCY_NONE`, `AMBIENT_LIGHT_FREQUENCY_50HZ`,
> `AMBIENT_LIGHT_FREQUENCY_60HZ`. Can be used to ensure that the suggested settings are compatible
> with the frequency of the ambient light in the scene. If ambient light is unproblematic, use
> `AMBIENT_LIGHT_FREQUENCY_NONE` for optimal performance. Default is `AMBIENT_LIGHT_FREQUENCY_NONE`.

See [Sample Capture Assistant](#sample-capture-assistant) for code example.

### capture
[zivid_camera/Capture.srv](./zivid_camera/srv/Capture.srv)

Invoke this service to trigger a 3D capture. See section [Configuration](#configuration) for how to
configure the 3D capture settings. The resulting point cloud is published on topics [points/xyz](#points/xyz) and
[points/xyzrgba](#points/xyzrgba), color image is published on topic [color/image_color](#colorimage_color),
and depth image is published on topic [depth/image](depthimage). Camera calibration is published on
topics [color/camera_info](#colorcamera_info) and [depth/camera_info](#depthcamera_info).

See [Sample Capture](#sample-capture) for code example.

### capture_2d
[zivid_camera/Capture2D.srv](./zivid_camera/srv/Capture2D.srv)

Invoke this service to trigger a 2D capture. See section [Configuration](#configuration) for how to
configure the 2D capture settings. The resulting 2D image is published to topic
[color/image_color](#colorimage_color). Note: 2D RGB image is also published as a part of 3D
capture, see [capture](#capture).

See [Sample Capture 2D](#sample-capture-2d) for code example.

### load_settings_from_file
[zivid_camera/LoadSettingsFromFile.srv](./zivid_camera/srv/LoadSettingsFromFile.srv)

Loads 3D settings from a `.yml` file saved from Zivid Studio or the Zivid SDK. The settings are
visible via dynamic_reconfigure, see section [Configuration](#configuration).
When this service has returned you can invoke the [capture](#capture) service to trigger a 3D
capture using these settings.

### load_settings_2d_from_file
[zivid_camera/LoadSettings2DFromFile.srv](./zivid_camera/srv/LoadSettings2DFromFile.srv)

Loads 2D settings from a `.yml` file saved from Zivid Studio or the Zivid SDK. The settings are
visible via dynamic_reconfigure, see section [Configuration](#configuration). When this
service has returned you can invoke the [capture_2d](#capture) service to trigger a 2D capture
using these settings.

### camera_info/model_name
[zivid_camera/CameraInfoModelName.srv](./zivid_camera/srv/CameraInfoModelName.srv)

Returns the camera's model name.

### camera_info/serial_number
[zivid_camera/CameraInfoSerialNumber.srv](./zivid_camera/srv/CameraInfoSerialNumber.srv)

Returns the camera's serial number.

### is_connected
[zivid_camera/IsConnected.srv](./zivid_camera/srv/IsConnected.srv)

Returns if the camera is currently in `Connected` state (from the perspective of the ROS driver).
The connection status is updated by the driver every 10 seconds and before each [capture](#capture)
service call. If the camera is not in `Connected` state the driver will attempt to re-connect to
the camera when it detects that the camera is available. This can happen if the camera is
power-cycled or the USB cable is unplugged and then replugged.

## Topics

The Zivid ROS driver provides several topics providing 3D, color, SNR and camera calibration
data as a result of calling [capture](#capture) and/or [capture_2d](#capture_2d).
The different output topics provides flexibility for different use cases. Note that for
performance reasons no messages are generated or sent on topics with zero active subscribers.

### color/camera_info
[sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

Camera calibration and metadata.

### color/image_color
[sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

Color/RGBA image. RGBA image is published as a result of invoking the [capture](#capture) or
[capture_2d](#capture_2d) service. Images are encoded as "rgba8", where the alpha channel
is always 255.

### depth/camera_info
[sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

Camera calibration and metadata.

### depth/image
[sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

Depth image. Each pixel contains the z-value (along the camera Z axis) in meters.
The image is encoded as 32-bit float. Pixels where z-value is missing are NaN.

### points/xyzrgba
[sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)

Point cloud data. Sent as a result of the [capture](#capture) service. The output is
in the camera's optical frame, where x is right, y is down and z is forward.
The included point fields are x, y, z (in meters) and rgba (color).

### points/xyz
[sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)

Point cloud data. This topic is similar to topic [points/xyzrgba](#points/xyzrgba), except
that only the XYZ 3D coordinates are included. This topic is recommended if you don't need
the RGBA values.

### snr/camera_info
[sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

Camera calibration and metadata.

### snr/image
[sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

Each pixel contains the SNR (signal-to-noise ratio) value. The image is encoded as 32-bit
float. Published as a part of the [capture](#capture) service.

### normals/xyz
[sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)

Normals for the point cloud. The included fields are normal x, y and z coordinates.
Each coordinate is a float value. There are no additional padding floats, so point-step is
12 bytes (3*4 bytes). The normals are unit vectors. Note that subscribing to this topic
will cause some additional processing time for calculating the normals.

## Configuration

The `zivid_camera` node supports both single-acquisition (2D and 3D) and multi-acquisition
HDR captures (3D only). 3D HDR-capture works by taking several acquisitions with different
settings (for example different exposure time) and combining them into one high-quality point
cloud. For more information about HDR capture, visit our
[knowledge base][hdr-getting-good-point-clouds-url].

The capture settings available in the `zivid_camera` node matches the settings in the Zivid API.
To become more familiar with the different settings and what they do, see the API reference for the
[Settings](http://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings.html)
and [Settings2D](http://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings2D.html)
classes, or use Zivid Studio.

The settings can be viewed and configured using [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure).
Use [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure) to view/change the settings using a GUI:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

If you want to experiment with the capture settings, launch the [Sample Capture](#sample-capture) sample,
which will capture in a loop forever, and use [rqt_reconfigure](https://wiki.ros.org/rqt_reconfigure)
to adjust the settings.

The available capture settings are organized into a hierarchy of configuration nodes. 3D settings are available
under the `/settings` namespace, while 2D settings are available under `/settings_2d`.

```
/settings
    ...
    /acquisition_0
        ...
    /acquisition_1
        ...
    ...
    /acquisition_9
        ...
/settings_2d
    ...
    /acquisition_0
        ...
```

**Important notice for C++ users:** The default value and min/max values of the settings are dependent on
what Zivid camera model you are using. Therefore you need to query the server for the correct values after
you have connected to the camera. To initialize a settings Config object you should **not** use the static
`__getDefault()__` methods of the auto-generated C++ config classes (`zivid_camera::SettingsConfig`,
`zivid_camera::SettingsAcquisitionConfig`, `zivid_camera::Settings2DConfig`, and
`zivid_camera::Settings2DAcquisitionConfig`). Instead, you should query the server for the default
values using `dynamic_reconfigure::Client<T>::getDefaultConfiguration()`. See the [C++ samples](#samples)
for how to do this. For Python users this is already handled by dynamic_reconfigure.client.Client.

**Tip:** Use the [load_settings_from_file](#load_settings_from_file)
or [load_settings_2d_from_file](#load_settings_2d_from_file) service to load 3D/2D settings from a
.yml file saved from Zivid Studio or the Zivid SDK.

**Tip:** The Capture Assistant feature can be used to find optimized 3D capture settings for your
scene. Refer to service [capture_assistant/suggest_settings](#capture_assistantsuggest_settings).

### 3D Settings

#### Acquisition settings

`settings/acquisition_<n>/` contains settings for an individual acquisition. By default `<n>` can be 0 to 9 for a
total of 10 acquisitions. The total number of acquisitions can be configured using the launch parameter
`max_capture_acquisitions` (see section [Launch Parameters](#launch-parameters-advanced) above).

`settings/acquisition_<n>/enabled` controls if acquisition `<n>` will be included when the [capture](#capture) service is
invoked. If only one acquisition is enabled the [capture](#capture) service performs a 3D single-capture. If more than
one acquisition is enabled the [capture](#capture) service will perform a 3D HDR-capture. By default `enabled` is false.
In order to capture a point cloud at least one acquisition needs to be enabled.

| Name                                     | Type   |  Zivid API Setting             |  Note  |
|------------------------------------------|--------|--------------------------------|--------|
| `settings/acquisition_<n>/enabled`       | bool   |
| `settings/acquisition_<n>/aperture`      | double | [Settings::Acquisition::Aperture](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Acquisition_1_1Aperture.html)
| `settings/acquisition_<n>/brightness`    | double | [Settings::Acquisition::Brightness](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Acquisition_1_1Brightness.html)
| `settings/acquisition_<n>/exposure_time` | int    | [Settings::Acquisition::ExposureTime](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Acquisition_1_1ExposureTime.html) | Microseconds
| `settings/acquisition_<n>/gain`          | double | [Settings::Acquisition::Gain](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Acquisition_1_1Gain.html)

#### Processing settings

`settings/` contains settings related to processing, like color balance and filter settings.

| Name                                                     | Type   |  Zivid API Setting                     |
|----------------------------------------------------------|--------|----------------------------------------|
| `settings/processing_color_balance_blue`                 | double | [Settings::Processing::Color::Balance::Blue](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Color_1_1Balance_1_1Blue.html)
| `settings/processing_color_balance_green`                | double | [Settings::Processing::Color::Balance::Green](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Color_1_1Balance_1_1Green.html)
| `settings/processing_color_balance_red`                  | double | [Settings::Processing::Color::Balance::Red](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Color_1_1Balance_1_1Red.html)
| `settings/processing_color_gamma`                        | double | [Settings::Processing::Color::Gamma](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Color_1_1Gamma.html)
| `settings/processing_filters_noise_removal_enabled`      | bool   | [Settings::Processing::Filters::Noise::Removal::Enabled](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Noise_1_1Removal_1_1Enabled.html)
| `settings/processing_filters_noise_removal_threshold`    | double | [Settings::Processing::Filters::Noise::Removal::Threshold](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Noise_1_1Removal_1_1Threshold.html)
| `settings/processing_filters_outlier_removal_enabled`    | bool   | [Settings::Processing::Filters::Outlier::Removal::Enabled](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Outlier_1_1Removal_1_1Enabled.html)
| `settings/processing_filters_outlier_removal_threshold`  | double | [Settings::Processing::Filters::Outlier::Removal::Threshold](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Outlier_1_1Removal_1_1Threshold.html)
| `settings/processing_filters_reflection_removal_enabled` | bool   | [Settings::Processing::Filters::Reflection::Removal::Enabled](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Reflection_1_1Removal_1_1Enabled.html)
| `settings/processing_filters_smoothing_gaussian_enabled` | bool   | [Settings::Processing::Filters::Smoothing::Gaussian::Enabled](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Smoothing_1_1Gaussian_1_1Enabled.html)
| `settings/processing_filters_smoothing_gaussian_sigma`   | double | [Settings::Processing::Filters::Smoothing::Gaussian::Sigma](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Smoothing_1_1Gaussian_1_1Sigma.html)

#### Experimental settings

Note that these settings may be changed, renamed or removed in future SDK releases.

| Name                                                                               | Type   |  Zivid API Setting             |
|------------------------------------------------------------------------------------|--------|--------------------------------|
| `settings/experimental_engine`                                                     | enum   | [Settings::Experimental::Engine](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Experimental_1_1Engine.html)
| `settings/processing_color_experimental_tone_mapping_enabled` | enum | [Settings::Processing::Color::Experimental::ToneMapping::Enabled](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Color_1_1Experimental_1_1ToneMapping_1_1Enabled.html)
| `settings/processing_filters_experimental_contrast_distortion_correction_enabled`  | bool   | [Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Experimental_1_1ContrastDistortion_1_1Correction_1_1Enabled.html)
| `settings/processing_filters_experimental_contrast_distortion_correction_strength` | double | [Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Experimental_1_1ContrastDistortion_1_1Correction_1_1Strength.html)
| `settings/processing_filters_experimental_contrast_distortion_removal_enabled`     | bool   | [Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Experimental_1_1ContrastDistortion_1_1Removal_1_1Enabled.html)
| `settings/processing_filters_experimental_contrast_distortion_removal_threshold`   | double | [Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings_1_1Processing_1_1Filters_1_1Experimental_1_1ContrastDistortion_1_1Removal_1_1Threshold.html)


### 2D settings

#### Acquisition settings

To trigger a 2D capture, invoke the [capture_2d](#capture_2d) service. Note that
`settings_2d/acquisition_0/enabled` is default false, and must be set to true before
calling the [capture_2d](#capture_2d) service, otherwise the service will return an error.

| Name                                      | Type   |  Zivid API Setting             |  Note  |
|-------------------------------------------|--------|--------------------------------|--------|
| `settings_2d/acquisition_0/enabled`       | bool   |
| `settings_2d/acquisition_0/aperture`      | double | [Settings2D::Acquisition::Aperture](https://www.zivid.com/software/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings2D_1_1Acquisition_1_1Aperture.html)
| `settings_2d/acquisition_0/brightness`    | double | [Settings2D::Acquisition::Brightness](https://www.zivid.com/software/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings2D_1_1Acquisition_1_1Brightness.html)
| `settings_2d/acquisition_0/exposure_time` | int    | [Settings2D::Acquisition::ExposureTime](https://www.zivid.com/software/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings2D_1_1Acquisition_1_1ExposureTime.html) | Microseconds
| `settings_2d/acquisition_0/gain`          | double | [Settings2D::Acquisition::Gain](https://www.zivid.com/software/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings2D_1_1Acquisition_1_1Gain.html)

#### Processing settings

`settings_2d/` contains settings related to processing of the captured image.

| Name                                         | Type   |  Zivid API Setting                     |
|----------------------------------------------|--------|----------------------------------------|
| `settings_2d/processing_color_balance_blue`  | double | [Settings2D::Processing::Color::Balance::Blue](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings2D_1_1Processing_1_1Color_1_1Balance_1_1Blue.html)
| `settings_2d/processing_color_balance_green` | double | [Settings2D::Processing::Color::Balance::Green](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings2D_1_1Processing_1_1Color_1_1Balance_1_1Green.html)
| `settings_2d/processing_color_balance_red`   | double | [Settings2D::Processing::Color::Balance::Red](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings2D_1_1Processing_1_1Color_1_1Balance_1_1Red.html)
| `settings_2d/processing_color_gamma`         | double | [Settings2D::Processing::Color::Gamma](https://www.zivid.com/hubfs/softwarefiles/releases/2.5.0+19fa6891-1/doc/cpp/classZivid_1_1Settings2D_1_1Processing_1_1Color_1_1Gamma.html)

## Samples

In the `zivid_samples` package we have added samples in C++ and Python that demonstrate how to use
the Zivid ROS driver. These samples can be used as a starting point for your project.

### Sample Capture Assistant

This sample shows how to use the Capture Assistant to capture with suggested settings for your
particular scene. This sample first calls the
[capture_assistant/suggest_settings](#capture_assistantsuggest_settings) service to get the suggested
settings. It then calls the [capture](#capture) service to invoke the 3D capture using those settings.

Source code: [C++](./zivid_samples/src/sample_capture_assistant.cpp), [Python](./zivid_samples/scripts/sample_capture_assistant.py)

Using roslaunch (also launches `roscore`, `zivid_camera`, `rviz` and `rqt_reconfigure`):
```bash
roslaunch zivid_samples sample.launch type:=sample_capture_assistant_cpp
roslaunch zivid_samples sample.launch type:=sample_capture_assistant.py
```
Using rosrun (when `roscore` and `zivid_camera` are running):
```bash
rosrun zivid_samples sample_capture_assistant_cpp
rosrun zivid_samples sample_capture_assistant.py
```

### Sample Capture

This sample performs single-acquisition 3D captures repeatedly. This sample shows how to [configure](#configuration)
the capture settings, how to subscribe to the [points/xyzrgba](#points/xyzrgba) topic, and how to invoke the
[capture](#capture) service.

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
the 2D capture settings, how to subscribe to the [color/image_color](#colorimage_color) topic, and
how to invoke the [capture_2d](#capture_2d) service.

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
you first start the first node, wait for it to be ready (for example, by waiting for the [capture](#capture)
service to be available), then start the second node. This avoids any race conditions where both nodes
may try to connect to the same camera at the same time.

### How to run the unit and module tests

This project comes with a set of unit and module tests to verify the provided functionality. To run
the tests locally, first download and install the file camera used for testing:
```bash
wget -q https://www.zivid.com/software/FileCameraZividOne.zip
unzip ./FileCameraZividOne.zip
rm ./FileCameraZividOne.zip
sudo mkdir -p /usr/share/Zivid/data/
sudo mv ./FileCameraZividOne.zfc /usr/share/Zivid/data/
```

Then run the tests:
```bash
cd ~/catkin_ws && source devel/setup.bash
catkin run_tests && catkin_test_results ~/catkin_ws
```

The tests can also be run via [docker](https://www.docker.com/). See the
[GitHub Actions configuration file](./.github/workflows/ROS-commit.yml) for details.

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
Visit [Zivid Knowledge Base][zivid-knowledge-base-url] for general help on using Zivid 3D
cameras. If you cannot find a solution to your issue, please contact customersuccess@zivid.com.

## Acknowledgements

<img src="https://www.zivid.com/software/zivid-ros/rosin_logo.png">

This FTP (Focused Technical Project) has received funding from the European Union's
Horizon 2020 research and innovation programme under the project ROSIN with the
grant agreement No 732287. For more information, visit [rosin-project.eu](http://rosin-project.eu/).

[ci-badge]: https://img.shields.io/github/workflow/status/zivid/zivid-ros/ROS%20Commit/master
[ci-url]: https://github.com/zivid/zivid-ros/actions?query=workflow%3A%22ROS+Commit%22+branch%3Amaster+
[header-image]: https://www.zivid.com/hubfs/softwarefiles/images/zivid-generic-github-header.png

[zivid-knowledge-base-url]: https://support.zivid.com
[zivid-software-installation-url]: https://support.zivid.com/latest/getting-started/software-installation.html
[install-opencl-drivers-ubuntu-url]: https://support.zivid.com/latest/getting-started/software-installation/gpu/install-opencl-drivers-ubuntu.html
[hdr-getting-good-point-clouds-url]: https://support.zivid.com/latest/academy/camera/capturing-high-quality-point-clouds/getting-the-right-exposure-for-good-point-clouds.html
[firmware-update-kb-url]: https://support.zivid.com/latest/academy/camera/firmware-update.html
