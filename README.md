# Zivid ROS driver

**This is a preview of the ROS 2 driver for Zivid cameras.**

_Thank you for testing this preview of our ROS2 driver! Be aware that there may be adjustments to behavior and API_
_before the official production release, which is scheduled for July._

_Please read the rest of this README.md for a description of the driver, and see [CHANGELOG.md](./CHANGELOG.md) for noteworthy changes._

_Contact us via issue https://github.com/zivid/zivid-ros/issues/50, or via customersuccess@zivid.com, to provide feedback or report issues._

[![Build Status][ci-badge]][ci-url]
![Zivid Image][header-image]

---

*Contents:*
[**Installation**](#installation) |
[**Getting Started**](#getting-started) |
[**Launching**](#launching-the-driver) |
[**Configuration**](#configuration) |
[**Services**](#services) |
[**Topics**](#topics) |
[**Samples**](#samples) |
[**FAQ**](#frequently-asked-questions)

---

## Installation

### Support

This driver supports Ubuntu 20.04/22.04 with ROS2. Follow the official [ROS installation instructions](https://docs.ros.org/) for
your OS. If you are looking for the ROS1 driver, see [releases](https://github.com/zivid/zivid-ros/releases).

### Zivid SDK

To use the ROS driver you need to download and install the "Zivid Core" package. Zivid SDK version 2.9.0 to 2.12.0 is
supported. See [releases](https://github.com/zivid/zivid-ros/releases) for older ROS driver releases
that supports older SDK versions.

Follow [this guide][zivid-software-installation-url]
to install "Zivid Core" for your version of Ubuntu. The "Zivid Studio" and "Zivid Tools" packages can be useful
to test your system setup and camera, but are not required by the driver.

An OpenCL 1.2 compatible GPU with driver installed is required by the SDK. Follow
[this guide][install-opencl-drivers-ubuntu-url] to install OpenCL drivers for your system.

### C++ compiler

A C++17 compiler is required.

```bash
sudo apt-get install -y g++
```

### Downloading and building Zivid ROS driver

First, source the `setup.bash` script for your ROS distribution in your terminal:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

Then create the workspace and src directory:
```bash
mkdir -p ~/ros2_ws/src
```

Clone the Zivid ROS project into the src directory:
```bash
cd ~/ros2_ws/src
git clone https://github.com/zivid/zivid-ros.git -b ros2-preview
```

Initialize rosdep:
```bash
cd ~/ros2_ws/src
sudo rosdep init
rosdep update
```

Install dependencies:
```bash
cd ~/ros2_ws/src
rosdep install -i --from-path ./ -y
```

Finally, build the driver:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

## Getting started

Connect the Zivid camera to your PC. You can use the `ZividListCameras` command-line
tool available in the "Zivid Tools" package to confirm that your system has been configured correctly, and
that the camera is discovered by your PC. You can also open Zivid Studio and connect to the camera.
Close Zivid Studio before continuing with the rest of this guide.

Run the sample_capture_cpp via the launch script to check that everything is working.

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch zivid_samples sample_with_rviz.launch sample:=sample_capture_cpp
```

This will start the `zivid_camera` driver node, the `sample_capture_cpp` node, and `rviz`.
The `zivid_camera` driver will connect to the first available Zivid camera, and 
then `sample_capture_cpp` will trigger captures repeatedly. The resulting point cloud and
color image should be visible in `rviz`.

A more detailed description of the `zivid_camera` driver follows below.

For sample code, see the [Samples](#samples) section.

## Launching the driver

To launch the driver, use `ros2 run`:

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run zivid_camera zivid_camera
```

The driver will by default connect to the first available Zivid camera.
This behavior can be overridden by setting the `serial_number` launch parameter, see below.

### Launch Parameters (advanced)

The following parameters can be specified when starting the driver. Note that all the parameters are
optional.

For example, to run the zivid_camera driver with a specific `serial_number` specified:

```bash
ros2 run zivid_camera zivid_camera --ros-args -p serial_number:=ABCD1234
```

Or you can use a [launch file](./zivid_samples/launch/zivid_camera_with_serial_number.launch)
and invoke it as:

```bash
ros2 launch zivid_samples zivid_camera_with_serial_number.launch serial_number:=ABCD1234
```

`file_camera_path` (string, default: "")
> Specify the path to a file camera to use instead of a real Zivid camera. This can be used to
> develop without access to hardware. The file camera returns the same point cloud for every capture.
> [Visit our knowledgebase to download file camera.](https://support.zivid.com/en/latest/academy/camera/file-camera.html)

`frame_id` (string, default: "zivid_optical_frame")
> Specify the frame_id used for all published images and point clouds.

`serial_number` (string, default: "")
> Specify the serial number of the Zivid camera to use.  This parameter is optional. By default, the
> driver will connect to the first available camera.

`update_firmware_automatically` (bool, default: true)
> Specify if the firmware of the connected camera should be automatically updated to the correct
> version when the Zivid ROS driver starts. If set to false, if the firmware version is out of date
> then camera must be manually updated, for example using Zivid Studio or `ZividFirmwareUpdater`.
> Read more about [firmware update in our knowledgebase][firmware-update-kb-url].
> This parameter is optional, and by default it is true.

## Configuration

The capture settings used by the `zivid_camera` ROS driver must be configured using YAML,
which can be exported from Zivid Studio or the API, or downloaded as .yml files from our [knowledge
base][presets-kb-url].

For convenience, the Zivid ROS driver supports configuring capture settings in two ways: Using file path
to a .yml file, or as a YAML string.

The following ROS parameters control which settings are used when capturing with the driver. Note
that you must set _either_ the `_file_path` or the `_yaml` parameter. If both `_file_path` and `_yaml`
parameters are set to a non-empty string at the same time, then the driver will throw an exception
when capturing. By default, all settings parameters are empty.

### 3D capture

`settings_file_path` (string, default: "")
> Specify the path to a .yml file that contains the settings you want to use.

`settings_yaml` (string, default: "")
> Specify a YAML string that contains the settings you want to use. For example, you can copy the contents of a .yml
> file saved from Zivid Studio.

The service `capture_assistant/suggest_settings` will modify the settings parameters automatically.

### 2D capture

`settings_2d_file_path` (string, default: "")
> Specify the path to a .yml file that contains the settings you want to use.

`settings_2d_yaml` (string, default: "")
> Specify a YAML string that contains the 2D settings you want to use. For example, you can copy the contents of a 
> .yml file saved from Zivid Studio.

## Services

### capture
[std_srvs/srv/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html)

Invoke this service to trigger a 3D capture. See section [Configuration](#configuration) for how to
configure the 3D capture settings. The resulting point cloud is published on topics [points/xyz](#pointsxyz) and
[points/xyzrgba](#pointsxyzrgba), color image is published on topic [color/image_color](#colorimage_color),
and depth image is published on topic [depth/image](#depthimage). Camera calibration is published on
topics [color/camera_info](#colorcamera_info) and [depth/camera_info](#depthcamera_info).

See [Sample Capture](#sample-capture) for code example.

### capture_2d
[std_srvs/srv/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html)

Invoke this service to trigger a 2D capture. See section [Configuration](#configuration) for how to
configure the 2D capture settings. The resulting 2D image is published to topic
[color/image_color](#colorimage_color). Note: 2D RGB image is also published as a part of 3D
capture, see [capture](#capture).

See [Sample Capture 2D](#sample-capture-2d) for code example.

### capture_and_save
[zivid_interfaces/srv/CaptureAndSave.srv](./zivid_interfaces/srv/CaptureAndSave.srv)

It does exactly the same as the [capture](#capture) service, in addition it will save the frame to
a file. This service takes a path as an argument. The chosen format is detected via the file extension.
See [knowledge base](https://support.zivid.com/en/latest/reference-articles/point-cloud-structure-and-output-formats.html#zivid-output-formats)
for a list of available output formats.

See [Sample Capture and Save](#sample-capture-and-save) for code example.

### capture_assistant/suggest_settings
[zivid_interfaces/srv/CaptureAssistantSuggestSettings.srv](./zivid_interfaces/srv/CaptureAssistantSuggestSettings.srv)

Invoke this service to analyze your scene and find suggested settings for your particular scene,
camera distance, ambient lighting conditions, etc. This service will automatically update the node parameter
`settings_yaml` with the suggested settings, see section [Configuration](#configuration).
When this service has returned, you can invoke the [capture](#capture) service to trigger a 3D capture using
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

### camera_info/model_name
[zivid_interfaces/srv/CameraInfoModelName.srv](./zivid_interfaces/srv/CameraInfoModelName.srv)

Returns the camera's model name.

### camera_info/serial_number
[zivid_interfaces/srv/CameraInfoSerialNumber.srv](./zivid_interfaces/srv/CameraInfoSerialNumber.srv)

Returns the camera's serial number.

### is_connected
[zivid_interfaces/srv/IsConnected.srv](./zivid_interfaces/srv/IsConnected.srv)

Returns if the camera is currently in `Connected` state (from the perspective of the ROS driver).
The connection status is updated by the driver every 10 seconds and before each [capture](#capture)
service call. If the camera is not in `Connected` state the driver will attempt to re-connect to
the camera when it detects that the camera is available. This can happen if the camera is
power-cycled or the USB/Ethernet cable is unplugged and then replugged.

## Topics

The Zivid ROS driver provides several topics providing 3D, color, SNR and camera calibration
data as a result of calling capture/capture_2d services. The different output topics provides 
flexibility for different use cases. Note that for performance reasons no messages are generated
or sent on topics with zero active subscribers.

### color/camera_info
[sensor_msgs/msg/CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html)

Camera calibration and metadata.

### color/image_color
[sensor_msgs/msg/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)

Color/RGBA image. RGBA image is published as a result of invoking the [capture](#capture) or
[capture_2d](#capture_2d) service. Images are encoded as "rgba8", where the alpha channel
is always 255.

### depth/camera_info
[sensor_msgs/msg/CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html)

Camera calibration and metadata.

### depth/image
[sensor_msgs/msg/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)

Depth image. Each pixel contains the z-value (along the camera Z axis) in meters.
The image is encoded as 32-bit float. Pixels where z-value is missing are NaN.

### points/xyzrgba
[sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)

Point cloud data. Sent as a result of the [capture](#capture) service. The output is
in the camera's optical frame, where x is right, y is down and z is forward.
The included point fields are x, y, z (in meters) and rgba (color).

### points/xyz
[sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)

Point cloud data. This topic is similar to topic [points/xyzrgba](#pointsxyzrgba), except
that only the XYZ 3D coordinates are included. This topic is recommended if you don't need
the RGBA values.

### snr/camera_info
[sensor_msgs/msg/CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html)

Camera calibration and metadata.

### snr/image
[sensor_msgs/msg/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)

Each pixel contains the SNR (signal-to-noise ratio) value. The image is encoded as 32-bit
float. Published as a part of the [capture](#capture) service.

### normals/xyz
[sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)

Normals for the point cloud. The included fields are normal x, y and z coordinates.
Each coordinate is a float value. There are no additional padding floats, so point-step is
12 bytes (3*4 bytes). The normals are unit vectors. Note that subscribing to this topic
will cause some additional processing time for calculating the normals.

## Samples

In the `zivid_samples` package we have added samples that demonstrate how to use
the Zivid ROS driver. These samples can be used as a starting point for your project.

_More samples will soon be provided_.

### Sample Capture

This sample performs single-acquisition 3D captures in a loop. This sample shows how to [configure](#configuration)
the capture settings, how to subscribe to the [points/xyzrgba](#pointsxyzrgba) topic, and how to invoke the
[capture](#capture) service.

Source code: [C++](./zivid_samples/src/sample_capture.cpp) [Python](./zivid_samples/scripts/sample_capture.py)

```bash
ros2 launch zivid_samples sample.launch sample:=sample_capture_cpp
ros2 launch zivid_samples sample.launch sample:=sample_capture.py
```
Using ros2 run (when `zivid_camera` node is already running):
```bash
ros2 run zivid_samples sample_capture_cpp
ros2 run zivid_samples sample_capture.py
```

### Sample Capture 2D

This sample performs single-acquisition 2D captures in a loop. This sample shows how to [configure](#configuration) the
capture 2d settings with a YAML string, how to subscribe to the [color/image_color](#colorimage_color) topic, and how to
invoke the [capture_2d](#capture_2d) service.

Source code: [C++](./zivid_samples/src/sample_capture_2d.cpp) [Python](./zivid_samples/scripts/sample_capture_2d.py)

```bash
ros2 launch zivid_samples sample.launch sample:=sample_capture_2d_cpp
ros2 launch zivid_samples sample.launch sample:=sample_capture_2d.py
```
Using ros2 run (when `zivid_camera` node is already running):
```bash
ros2 run zivid_samples sample_capture_2d_cpp
ros2 run zivid_samples sample_capture_2d.py
```

### Sample Capture and Save

This sample performs a capture, and stores the resulting frame to file. This sample shows how to
[configure](#configuration) the capture settings with a YAML string, how to invoke the
[capture_and_save](#capture_and_save) service, and how to read the response from the service call.

Source code: [C++](./zivid_samples/src/sample_capture_and_save.cpp) [Python](./zivid_samples/scripts/sample_capture_and_save.py)

```bash
ros2 launch zivid_samples sample.launch sample:=sample_capture_and_save_cpp
ros2 launch zivid_samples sample.launch sample:=sample_capture_and_save.py
```

Using ros2 run (when `zivid_camera` node is already running):

```bash
ros2 run zivid_samples sample_capture_and_save_cpp
ros2 run zivid_samples sample_capture_and_save.py
```

### Sample Capture Assistant

This sample shows how to invoke the [capture_assistant/suggest_settings](#capture_assistantsuggest_settings) service to
suggest and set capture settings. Then, it shows how to subscribe to the [points/xyzrgba](#pointsxyzrgba) and
[color/image_color](#colorimage_color) topics, and finally invoke the [capture](#capture) service.

Source code: [C++](./zivid_samples/src/sample_capture_assistant.cpp) [Python](./zivid_samples/scripts/sample_capture_assistant.py)

```bash
ros2 launch zivid_samples sample.launch sample:=sample_capture_assistant_cpp
ros2 launch zivid_samples sample.launch sample:=sample_capture_assistant.py
```
Using ros2 run (when `zivid_camera` node is already running):
```bash
ros2 run zivid_samples sample_capture_assistant_cpp
ros2 run zivid_samples sample_capture_assistant.py
```

### Sample Capture with Settings from File

This sample performs single-acquisition 3D captures in a loop. This sample shows how to [configure](#configuration) the
capture settings from a yaml file, how to subscribe to the [points/xyzrgba](#pointsxyzrgba) topic, and how to invoke the
[capture](#capture) service.

Source code: [C++](./zivid_samples/src/sample_capture_with_settings_from_file.cpp) [Python](./zivid_samples/scripts/sample_capture_with_settings_from_file.py)

```bash
ros2 launch zivid_samples sample.launch sample:=sample_capture_with_settings_from_file_cpp
ros2 launch zivid_samples sample.launch sample:=sample_capture_with_settings_from_file.py
```

Using ros2 run (when `zivid_camera` node is already running):

```bash
ros2 run zivid_samples sample_capture_with_settings_from_file_cpp
ros2 run zivid_samples sample_capture_with_settings_from_file.py
```

## Frequently Asked Questions

### How to visualize the output from the camera in rviz

```bash
ros2 launch zivid_camera visualize.launch
```

### How to connect to specific Zivid serial number

```bash
ros2 run zivid_camera zivid_camera --ros-args -p serial_number:=ABCD1234
```

### How to start the driver using settings.yml files

See section [Configuration](#configuration) for more details.
```bash
ros2 run zivid_camera zivid_camera --ros-args -p settings_file_path:=/path/to/settings.yml -p settings_2d_file_path:=/path/to/settings2D.yml
```

### How to trigger 3D/2D capture via terminal

```bash
ros2 service call /capture std_srvs/srv/Trigger
ros2 service call /capture_2d std_srvs/srv/Trigger
```

### How to use a file camera

```bash
ros2 run zivid_camera zivid_camera --ros-args -p file_camera_path:=/usr/share/Zivid/data/FileCameraZivid2L100.zfc
```

Visit our  [knowledgebase](https://support.zivid.com/en/latest/academy/camera/file-camera.html) to download file camera.

### How to use multiple Zivid cameras

You can use multiple Zivid cameras simultaneously by starting one node per camera and specifying
unique namespaces per node.

```bash
ros2 run zivid_camera zivid_camera --ros-args --remap __ns:=/zivid_camera1
```

```bash
ros2 run zivid_camera zivid_camera --ros-args --remap __ns:=/zivid_camera2
```

You can combine this with a serial_number parameter (see above) to  control which node uses which camera.
By default, the zivid_camera node will connect to the first available (unused) camera. We recommend that
you first start the first node, then wait for it to be ready (for example, by waiting for the [capture](#capture)
service to be available), and then start the second node. This avoids any race conditions where both nodes
may try to connect to the same camera at the same time.

### How to run the unit and module tests

This project comes with a set of unit and module tests to verify the provided functionality. To run
the tests locally, first download and install the file camera used for testing:
```bash
wget -q https://www.zivid.com/software/FileCameraZivid2M70.zip
unzip ./FileCameraZivid2M70.zip
rm ./FileCameraZivid2M70.zip
sudo mkdir -p /usr/share/Zivid/data/
sudo mv ./FileCameraZivid2M70.zfc /usr/share/Zivid/data/
```

Then run the tests:
```bash
cd ~/ros2_ws/ && source install/setup.bash
colcon test --event-handlers console_direct+ && colcon test-result --all
```

The tests can also be run via [docker](https://www.docker.com/). See the
[GitHub Actions configuration file](./.github/workflows/ROS-commit.yml) for details.

### How to enable debug logging

The node logs extra information at log level debug, including the settings used when capturing.
Enable debug logging to troubleshoot issues:

```bash
ros2 run zivid_camera zivid_camera  --ros-args --log-level debug
```

Above will enable debug logging for all components, you can also specify just the zivid_camera
logger like so:
```
ros2 run zivid_camera zivid_camera  --ros-args --log-level zivid_camera:=debug
```

### How to compile the project with warnings enabled

```bash
colcon build --cmake-args -DCOMPILER_WARNINGS=ON
```

### How to format the source code

The CI test for this package enforces the linting defined by `ament_lint_auto`.

```bash
cd ~/ros2_ws/src/zivid-ros
ament_uncrustify --reformat ./
```

## License

This project is licensed under BSD 3-clause license, see the [LICENSE](LICENSE) file for details.

## Support

Please report any issues or feature requests related to the ROS driver in the issue tracker.
Visit [Zivid Knowledge Base][zivid-knowledge-base-url] for general help on using Zivid 3D
cameras. If you cannot find a solution to your issue, please contact customersuccess@zivid.com.

## Acknowledgements

<img src="https://www.zivid.com/software/zivid-ros/rosin_logo.png">
<img src="https://www.zivid.com/software/zivid-ros/rosin_logo.png">

This FTP (Focused Technical Project) has received funding from the European Union's
Horizon 2020 research and innovation programme under the project ROSIN with the
grant agreement No 732287. For more information, visit [rosin-project.eu](http://rosin-project.eu/).

[ci-badge]: https://img.shields.io/github/actions/workflow/status/zivid/zivid-ros/ROS-commit.yml?branch=ros2-preview
[ci-url]: https://github.com/zivid/zivid-ros/actions?query=workflow%3A%22ROS+Commit%22+branch%3Aros2-preview+
[header-image]: https://www.zivid.com/hubfs/softwarefiles/images/zivid-generic-github-header.png

[zivid-knowledge-base-url]: https://support.zivid.com
[zivid-software-installation-url]: https://support.zivid.com/latest/getting-started/software-installation.html
[install-opencl-drivers-ubuntu-url]: https://support.zivid.com/latest/getting-started/software-installation/gpu/install-opencl-drivers-ubuntu.html
[hdr-getting-good-point-clouds-url]: https://support.zivid.com/latest/academy/camera/capturing-high-quality-point-clouds/getting-the-right-exposure-for-good-point-clouds.html
[firmware-update-kb-url]: https://support.zivid.com/latest/academy/camera/firmware-update.html
[presets-kb-url]: https://support.zivid.com/en/latest/reference-articles/presets-settings.html
