# Zivid ROS driver changelog

## 0.9.1

* Added support for 2D capture. Added new service "capture_2d" and new dynamic_reconfigure
  server "capture_2d/frame_0". Added sample_capture_2d for both C++ and Python. For more
  information about this new service, see README.md.
* Made common sample.launch script for all samples.
* Adjusted rviz/camera_view.rviz to make the 2D RGB image a bit bigger.
* Other minor improvements to README.md.
* Bumped minimum Zivid SDK version to 1.6.0.

## 0.9.0

* Initial release of ROS driver for Zivid cameras. For more information, see README.md.
