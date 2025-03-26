# Zivid ROS driver

This is the official ROS driver for [Zivid 3D cameras](https://www.zivid.com/).

This driver provides support for ROS2.

If you are looking for the Zivid ROS1 driver, please use the [`ros1-master` branch](https://github.com/zivid/zivid-ros/tree/ros1-master) if
you are using Zivid SDK 2.13 or older, or use the [`ros1-sdk-2.14.0` branch](https://github.com/zivid/zivid-ros/tree/ros1-sdk-2.14.0) if you
are using Zivid SDK 2.14 or newer.

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
[**RViz Plugin**](#rviz-plugin) |
[**FAQ**](#frequently-asked-questions)

---

## Installation

### Support

This driver supports Ubuntu 20.04 / 22.04 / 24.04 with ROS2. Follow the official [ROS installation instructions](https://docs.ros.org/) for
your OS.

If you are looking for the Zivid ROS1 driver, please use the [`ros1-master` branch](https://github.com/zivid/zivid-ros/tree/ros1-master) for
Zivid SDK 2.13 or older, or use the [`ros1-sdk-2.14.0` branch](https://github.com/zivid/zivid-ros/tree/ros1-sdk-2.14.0) for Zivid SDK 2.14 or newer.

### Zivid SDK

To use the ROS driver you need to download and install the "Zivid Core" package. Zivid SDK version 2.15.0 is
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
git clone https://github.com/zivid/zivid-ros.git
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

`color_space` (string, default: "linear_rgb")
> Specify the color space to use when publishing point clouds and images. Valid values:
>
>  - `srgb`: Publish in the sRGB color space. The sRGB color space is suitable for showing an image
>    on a display for human viewing. It is easier to see details in darker areas of an image in sRGB
>    than in linear RGB, as more of the dynamic range is dedicated to darker colors. This format is
>    assumed by default by most monitors and should be used when displaying an image. This option
>    should be used to match colors to the visualization in Zivid Studio.
>  - `linear_rgb`: Publish in linear RGB color space. Linear RGB is suitable as input to computer
>    vision algorithms.
>
> In particular, this parameter affects the data published over the [color/image_color](#colorimage_color) and
> [points/xyzrgba](#pointsxyzrgba) topics. Please see the Zivid knowledge base on
> [2D Color Spaces and Output Formats](https://support.zivid.com/en/latest/reference-articles/color-spaces-and-output-formats.html)
> for more details.

## Configuration

The capture settings used by the `zivid_camera` ROS driver must be configured using YAML,
which can be exported from Zivid Studio or the API, or downloaded as .yml files from our [knowledge
base][presets-kb-url].

For convenience, the Zivid ROS driver supports configuring capture settings in two ways: Using file path
to a .yml file, or as a YAML string.

The following ROS parameters control which settings are used when capturing with the driver. Note
that you must set _either_ the `_file_path` or the `_yaml` parameter. If both `_file_path` and `_yaml`
parameters are set to a non-empty string at the same time, then the driver will return an error when
capturing. By default, all settings parameters are empty.

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

### capture_and_detect_calibration_board
[zivid_interfaces/srv/CaptureAndDetectCalibrationBoard.srv](./zivid_interfaces/srv/CaptureAndDetectCalibrationBoard.srv)

Performs a capture to detect a calibration board and estimate its pose. This service call will perform a relatively slow
but high-quality point cloud capture with the connected camera. Any settings applied to the camera in the ROS driver
will be ignored for calls to this service, appropriate settings are automatically used. The resulting point cloud and
color image will be published just like during a normal call to the [capture](#capture) service.

The returned data from the service includes a member of the type
[DetectionResultCalibrationBoard.msg](./zivid_interfaces/msg/DetectionResultCalibrationBoard.msg), with details on any
detected calibration board.

The functionality is to be exclusively used in combination with Zivid verified calibration boards. For further
information please visit [Zivid help page](https://support.zivid.com).

### capture_and_detect_markers
[zivid_interfaces/srv/CaptureAndDetectMarkers.srv](./zivid_interfaces/srv/CaptureAndDetectMarkers.srv)

Performs a capture to detect fiducial markers, such as ArUco markers, and estimate their poses. As opposed to the
[capture_and_detect_calibration_board](#capture_and_detect_calibration_board) service, this service uses the current
settings applied to the camera. See section [Configuration](#configuration) for how to configure the 3D capture
settings. The resulting point cloud and color image will be published just like during a normal call to the
[capture](#capture) service.

The name of the fiducial dictionary must be provided, along with a list of marker IDs. The scene may not need not
contain all listed markers for a successful detection. For further information on fiducial markers see [this wikipedia
page](https://en.wikipedia.org/wiki/Fiducial_marker). For more information on ArUco markers specifically, refer to the
[OpenCV documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).

The returned data from the service includes a member of the type
[DetectionResultFiducialMarkers.msg](./zivid_interfaces/msg/DetectionResultFiducialMarkers.msg), with details on any
detected fiducial markers.

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

### hand_eye_calibration/start
[zivid_interfaces/srv/HandEyeCalibrationStart.srv](./zivid_interfaces/srv/HandEyeCalibrationStart.srv)

Prepares the node for hand-eye calibration and clears any previously collected hand-eye captures. The type of the
[calibration object](https://support.zivid.com/en/latest/academy/applications/hand-eye/calibration-object.html) to be
used during capture must be provided. A *working directory* can optionally be provided so that all captures are saved to
this directory. If provided, the directory must be given as an absolute path and the directory must be empty.

This service must be called first before capturing data for hand-eye calibration. It can also be used to restart an
active hand-eye calibration session. After calling this service, proceed with captures by calling the
[hand_eye_calibration/capture](#hand_eye_calibrationcapture) service.

Please refer to the Zivid knowledge base for more information on [hand-eye
calibration](https://support.zivid.com/en/latest/academy/applications/hand-eye.html).

### hand_eye_calibration/capture
[zivid_interfaces/srv/HandEyeCalibrationCapture.srv](./zivid_interfaces/srv/HandEyeCalibrationCapture.srv)

Performs a hand-eye calibration capture. This service takes the robot pose as an input. Then it performs a capture, and
detects any calibration objects. The resulting point cloud and color image will be published just like during a normal call to the
[capture](#capture) service.

If the detections is successful, the result is stored locally in the driver. Additionally, if a working directory was
specified during start, the captured frame and robot pose is saved to that directory.

This service uses the currently stored 3D capture settings to perform the capture. Ensure that the camera is [properly
configured](#configuration) first. Please see the knowledge base for [how to get good quality data](https://support.zivid.com/en/latest/academy/applications/hand-eye/how-to-get-good-quality-data-on-zivid-calibration-board.html).

The camera and robot should be appropriately positioned so that the calibration object is visible in the frame. Multiple
captures in different poses are necessary. After performing several captures, one can proceed by calling the
[hand_eye_calibration/calibrate](#hand_eye_calibrationcalibrate) service.

### hand_eye_calibration/calibrate
[zivid_interfaces/srv/HandEyeCalibrationCalibrate.srv](./zivid_interfaces/srv/HandEyeCalibrationCalibrate.srv)

Computes the hand-eye calibration transform based on the captures gathered during the current hand-eye calibration
session. Both eye-to-hand and eye-in-hand configurations are supported. If successful, the computed hand-eye transform
is returned, see the knowledge base for more information on the [hand-eye calibration
solution](https://support.zivid.com/en/latest/academy/applications/hand-eye/hand-eye-calibration-solution.html).

The calibration procedure requires all robot poses to be different. At least 2 poses are required when using a
calibration board, or 6 poses when using fiducial markers. For fiducial markers, each marker must be detected across 2
poses at minimum.

Low degrees-of-freedom (DOF) calibration is also supported (experimental) by supplying the fixed placement of
calibration objects. This is not needed for regular (6-DOF) calibration.

### hand_eye_calibration/load
[zivid_interfaces/srv/HandEyeCalibrationLoad.srv](./zivid_interfaces/srv/HandEyeCalibrationLoad.srv)

Loads a working directory from a previous hand-eye calibration session. See the
[hand_eye_calibration/start](#hand_eye_calibrationstart) service for how to start a session with a working directory.
The captures and robot poses are loaded from the provided directory. The directory is opened as read-only, and no new
captures can be made during this session. However, calls to the
[hand_eye_calibration/calibrate](#hand_eye_calibrationcalibrate) service can be made to compute the hand-eye transform
from the loaded data.

### infield_correction/read
[zivid_interfaces/srv/InfieldCorrectionRead.srv](./zivid_interfaces/srv/InfieldCorrectionRead.srv)

Returns the state of the [infield
correction](https://support.zivid.com/en/latest/academy/camera/infield-correction.html) of the camera.

### infield_correction/reset
[std_srvs/srv/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html)

Resets the infield correction on the camera to factory settings.

### infield_correction/start
[std_srvs/srv/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html)

Prepares for infield correction, and clears any infield correction captures gathered so far in the `zivid_camera` node.

This service must be called before gathering captures using the [infield_correction/capture](#infield_correctioncapture)
service. However, other infield correction services can be used without calling the start service first. This service
can also be used to restart an active infield correction session.

Please refer to the Zivid knowledge base for further information and guidelines for [infield
correction](https://support.zivid.com/en/latest/academy/camera/infield-correction.html). 

### infield_correction/capture
[zivid_interfaces/srv/InfieldCorrectionCapture.srv](./zivid_interfaces/srv/InfieldCorrectionCapture.srv)

Takes a capture to be used for infield correction. Please point the camera at a Zivid infield calibration object. It is
recommended to cover several distances, with one or more captures at each distance. Successful captures are stored in
the `zivid_camera` node.

Before calling this service, a call to [infield_correction/start](#infield_correctionstart) must have been made first.

This service call will perform a relatively slow but high-quality point cloud capture with the connected camera. Any
settings applied to the camera in the ROS driver will be ignored for calls to this service, appropriate settings are
automatically used. The resulting point cloud and color image will be published just like during a normal call to the
[capture](#capture) service.

After sufficient number of captures, proceed by calling the [infield_correction/compute](#infield_correctioncompute)
service to compute the verification metrics and the correction based on the captures gathered so far. To additionally
write the correction to camera, proceed by calling the
[infield_correction/compute_and_write](#infield_correctioncompute_and_write) service.

The captured data is cleared if the node is stopped, or after a successful call to either of the services
[infield_correction/start](#infield_correctionstart) or
[infield_correction/compute_and_write](#infield_correctioncompute_and_write).

### infield_correction/compute
[zivid_interfaces/srv/InfieldCorrectionCompute.srv](./zivid_interfaces/srv/InfieldCorrectionCompute.srv)

Calculates the new infield correction based the captured data gathered so far through the service
[infield_correction/capture](#infield_correctioncapture).

The quantity and range of data is up to the user, but generally a larger dataset will yield a more accurate and reliable
correction. If all measurements are taken at approximately the same distance, the resulting correction will mainly be
valid at those distances. If several measurements are taken at significantly different distances, the resulting
correction will likely be more suitable for extrapolation to distances beyond where the dataset is collected.

This service also acts as verification of the quality of the infield correction on a camera, or the need for one if none
exists already. It returns an indication of the dimension trueness at the location where the input data was captured.

If the returned assessment indicates a trueness error that is above the threshold for your application, consider using
[infield_correction/compute_and_write](#infield_correctioncompute_and_write). The service also returns information
regarding the proposed working range and the accuracy that can be expected within the working range, if the correction
is later written to the camera.

### infield_correction/compute_and_write
[zivid_interfaces/srv/InfieldCorrectionCompute.srv](./zivid_interfaces/srv/InfieldCorrectionCompute.srv)

Calculates the new infield correction based the captured data gathered so far through the service
[infield_correction/capture](#infield_correctioncapture), and writes the result to the camera.

If the write operation is successful, the infield correction capture data is cleared. To perform infield correction
again, a new session must be started with a call to the [infield_correction/start](#infield_correctionstart) service.

Please see the [infield_correction/compute](#infield_correctioncompute) service for more information on the computed
correction.

### infield_correction/remove_last_capture
[std_srvs/srv/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html)

Removes the last infield correction capture gathered in the `zivid_camera` node.

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

To launch the Python samples using `ros2 launch`, you need `python` to be available as a command.
For example, the `python-is-python3` package can be installed to achieve this, by running the
following command:

```bash
sudo apt install python-is-python3
```

On Windows, the Python samples cannot be launched using `ros2 launch`. Instead, please launch the
samples using `ros2 run zivid_samples <sample_name>.py` together with
`ros2 run zivid_camera zivid_camera` in another terminal window.

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

### Sample Capture and Detect Calibration Board

This sample performs a capture and detects any calibration board in the current scene. It shows how to invoke the
[capture_and_detect_calibration_board](#capture_and_detect_calibration_board) service and log the detection results.

Source code: [C++](./zivid_samples/src/sample_capture_and_detect_calibration_board.cpp)

```bash
ros2 launch zivid_samples sample.launch sample:=sample_capture_and_detect_calibration_board_cpp
```

Using ros2 run (when `zivid_camera` node is already running):

```bash
ros2 run zivid_samples sample_capture_and_detect_calibration_board_cpp
```

### Sample Capture and Detect Markers

This sample performs a capture and detects any fiducial markers in the current scene. It shows how to invoke the
[capture_and_detect_markers](#capture_and_detect_markers) service and log the detection result.

Source code: [C++](./zivid_samples/src/sample_capture_and_detect_markers.cpp)

```bash
ros2 launch zivid_samples sample.launch sample:=sample_capture_and_detect_markers_cpp
```

Using ros2 run (when `zivid_camera` node is already running):

```bash
ros2 run zivid_samples sample_capture_and_detect_markers_cpp
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

### Sample Infield Correction

This sample shows how to invoke the various [infield_correction/[...]](#infield_correctionread) services to perform infield correction on Zivid cameras.

Source code: [C++](./zivid_samples/src/sample_infield_correction.cpp)

```bash
ros2 launch zivid_samples sample.launch sample:=sample_infield_correction_cpp operation:=<operation>
```
Using ros2 run (when `zivid_camera` node is already running):
```bash
ros2 run zivid_samples sample_infield_correction_cpp --ros-args -p operation:=<operation>
```

With the following argument:

`operation` (string, required)
> Specify the infield correction operation to perform, one of: 
>   - `verify`: Verify camera correction quality based on a single capture using the [`infield_correction/start`](#infield_correctionstart), [`infield_correction/capture`](#infield_correctioncapture), and [`infield_correction/compute`](#infield_correctioncompute) services.
>   - `correct`: Calculate in-field correction based on a series of captures at different distances. Demonstrates the use of [`infield_correction/start`](#infield_correctionstart), [`infield_correction/capture`](#infield_correctioncapture), and [`infield_correction/compute`](#infield_correctioncompute) services. Begins by preparing the camera node for infield correction captures, then the sample gathers a fixed number of captures at a fixed duration between captures. The correction result is computed after every capture.
>   - `correct_and_write`: Same as `correct`, but additionally writes the correction results to the camera. Demonstrates the [`infield_correction/compute_and_write`](#infield_correctioncompute_and_write) service.
>   - `read`: Get information about the correction currently on the connected camera using the [`infield_correction/read`](#infield_correctionread) service.
>   - `reset`: Reset correction on connected camera to factory settings using the [`infield_correction/reset`](#infield_correctionreset) service.

Please see the [infield correction documentation](https://support.zivid.com/en/latest/academy/camera/infield-correction.html) for prerequisites and guidelines on how to perform the correction.

For a more interactive experience, we recommend using the [infield correction panel](#infield-correction-panel) from the Zivid RViz plugin.

The typical procedure for performing a new infield correction is:

1. `start`: Prepare for infield correction, clears any existing infield correction captures.
2. `capture`: Take multiple captures from different angles and distances in accordance with the typical operating conditions of the camera.
3. `compute`: Check the computed correction results and the estimated errors, verify that they give satisfactory results.
4. `compute_and_write`: Compute the correction and write the results to camera.

The `zivid_camera` node persists the infield correction dataset as long as it is running. To start the infield correction captures over again, use the `start` operation which clears all infield correction captures previously gathered. The `remove_last_capture` can be used to remove just the last capture.

### Sample Hand-Eye Calibration

This sample shows how to invoke various [hand_eye_calibration/[...]](#hand_eye_calibrationstart) services to perform
hand-eye calibration on Zivid cameras. The sample is for exposition only, to demonstrate how the services can be called.

The sample begins by preparing the camera node for hand-eye calibration. Then a fixed number of captures is gathered at
a fixed duration between captures, using a simulated robot pose. Finally, hand-eye calibration is run using the gathered
data.

Source code: [C++](./zivid_samples/src/sample_hand_eye_calibration.cpp)

```bash
ros2 launch zivid_samples sample.launch sample:=sample_hand_eye_calibration_cpp configuration:=<configuration> marker_ids:=<marker_ids> working_directory:=<working_directory>
```

Using ros2 run (when `zivid_camera` node is already running):
```bash
ros2 run zivid_samples sample_hand_eye_calibration_cpp --ros-args -p configuration:=<configuration> -p marker_ids:=<marker_ids> -p working_directory:=<working_directory>
```

With the following arguments:

`configuration` (string, required)
> Specify the configuration for the hand-eye calibration, one of:
>   - `eye_to_hand`: Performs calibration in the [eye-to-hand configuration](https://support.zivid.com/en/latest/academy/applications/hand-eye/hand-eye-calibration-problem.html#eye-to-hand).
>   - `eye_in_hand`: Performs calibration in the [eye-in-hand configuration](https://support.zivid.com/en/latest/academy/applications/hand-eye/hand-eye-calibration-problem.html#eye-in-hand).

`marker_ids` (dynamic array of integers, default: *empty*):
> Specifies a list of [fiducial marker IDs](https://support.zivid.com/en/latest/academy/applications/hand-eye/calibration-object.html) used for detection (e.g. `[1,2,3]`),
> or empty if a [Zivid calibration board](https://support.zivid.com/en/latest/academy/applications/hand-eye/calibration-object.html) is used instead.

`working_directory` (string, default: *empty*)
> argument specifies the [working directory](#hand_eye_calibrationstart) used to store the
> gathered data, or empty to indicate that the data should not be stored on disk.
> A non-empty value must specify an absolute path to an empty directory.

For more information on performing the calibration, please see the [Zivid hand-eye calibration documentation](https://support.zivid.com/en/latest/academy/applications/hand-eye.html).

## RViz Plugin

### Infield Correction Panel

The Zivid RViz plugin provides a panel to interactively perform infield correction with a Zivid camera.

To see the panel in RViz, go to `Panels -> Add New Panel`, then select `Zivid Infield Correction` and click `OK`.
Infield correction can now be performed by interacting with the newly added panel.

The panel is also visible when launching a Zivid sample with RViz, e.g.:
```
ros2 launch zivid_samples sample_with_rviz.launch sample:=sample_capture_with_settings_from_file_cpp
```

When the `zivid_camera` node is already running, RViz can also be started directly with the panel visible by using the
Zivid RViz display configuration:
```
rviz2 -d $(ros2 pkg prefix zivid_camera)/share/zivid_camera/config/rviz/zivid.rviz
```

Please see the [infield correction documentation](https://support.zivid.com/en/latest/academy/camera/infield-correction.html)
for prerequisites and guidelines on how to perform the correction.


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

### How to start the driver using the sRGB color space set

```bash
ros2 run zivid_camera zivid_camera --ros-args -p color_space:=srgb
```
### How to trigger 3D/2D capture via terminal

```bash
ros2 service call /capture std_srvs/srv/Trigger
ros2 service call /capture_2d std_srvs/srv/Trigger
```

### How to change the color space via terminal

```bash
ros2 param set zivid_camera color_space srgb
ros2 param set zivid_camera color_space linear_rgb
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
the tests locally, first download and install the required data used for testing:
```bash
for sample in "FileCameraZivid2M70.zip" "BinWithCalibrationBoard.zip"; do
    echo "Downloading ${sample}"
    wget -q "https://www.zivid.com/software/${sample}" || exit $?
    mkdir -p /usr/share/Zivid/data/ || exit $?
    unzip "./${sample}" -d /usr/share/Zivid/data/ || exit $?
    rm "./${sample}" || exit $?
done
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

The CI test for this package enforces the linting defined by `clang-format`. From the code analysis
docker image, run:

```bash
find /host -name '*.cpp' -or -name '*.hpp' | xargs clang-format -i
```

The style follows the one from
[`ament_clang_format`](https://github.com/ament/ament_lint/blob/master/ament_clang_format/doc/index.rst).

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

[ci-badge]: https://img.shields.io/github/actions/workflow/status/zivid/zivid-ros/ROS-commit.yml?branch=master
[ci-url]: https://github.com/zivid/zivid-ros/actions?query=workflow%3A%22ROS+Commit%22+branch%3Amaster+
[header-image]: https://www.zivid.com/hubfs/softwarefiles/images/zivid-generic-github-header.png

[zivid-knowledge-base-url]: https://support.zivid.com
[zivid-software-installation-url]: https://support.zivid.com/latest/getting-started/software-installation.html
[install-opencl-drivers-ubuntu-url]: https://support.zivid.com/latest/getting-started/software-installation/gpu/install-opencl-drivers-ubuntu.html
[hdr-getting-good-point-clouds-url]: https://support.zivid.com/latest/academy/camera/capturing-high-quality-point-clouds/getting-the-right-exposure-for-good-point-clouds.html
[firmware-update-kb-url]: https://support.zivid.com/latest/academy/camera/firmware-update.html
[presets-kb-url]: https://support.zivid.com/en/latest/reference-articles/presets-settings.html
