# The 2D image coordinates of the corners of the detected marker.
geometry_msgs/Point[4] corners_in_pixel_coordinates

# The 3D spatial coordinates of the corners of the detected marker.
#
# Note: Lengths are given in units of meter. In the Zivid driver, the units are automatically converted from the default
# units of the Zivid point clouds in millimeter, to the ROS convention of meter.
geometry_msgs/Point[4] corners_in_camera_coordinates

# The id of the detected marker.
int32 id

# The 3D pose of the marker. The returned pose will be positioned at the center of the marker, and have an orientation
# such that its z-axis points perpendicularly into the face of the marker.
#
# Note: Lengths are given in units of meter. In the Zivid driver, the units are automatically converted from the default
# units of the Zivid point clouds in millimeter, to the ROS convention of meter.
geometry_msgs/Pose pose
