# Zivid ROS driver changelog

This project adheres to [Semantic Versioning](https://semver.org).

## 1.1.0

* Updated the Intel OpenCL CPU runtime driver used in CI tests. This makes the CI tests
  stable again.
* Fixed a unit test that failed due to a missing sleep.
* Enabled unit tests for the Capture Assistant.
* Other minor improvements to README.md.
* Bumped minimum Zivid SDK version to 1.8.0.

## 1.0.0

* Added support for Capture Assistant. Added new service "capture_assistant/suggest_settings"
  that can be used to find suggested settings for your scene. Added sample_capture_assistant
  for C++ and Python. For more information, see README.md.
* Added support for 2D capture. Added new service "capture_2d" and new dynamic_reconfigure
  server "capture_2d/frame_0". Added sample_capture_2d for both C++ and Python. For more
  information, see README.md.
* Made common sample.launch script for all samples.
* Adjusted rviz/camera_view.rviz to make the 2D RGB image a bit bigger.
* Other minor improvements to README.md.
* Bumped minimum Zivid SDK version to 1.7.0.

## 0.9.0

* Initial release of ROS driver for Zivid cameras. For more information, see README.md.
