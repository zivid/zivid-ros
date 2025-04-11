uint8 STATUS_NOT_SET = 0
uint8 STATUS_OK = 1
uint8 STATUS_NO_VALID_FIDUCIAL_MARKER_DETECTED = 2
uint8 STATUS_MULTIPLE_VALID_FIDUCIAL_MARKERS_DETECTED = 3
uint8 STATUS_BOARD_DETECTION_FAILED = 4
uint8 STATUS_INSUFFICIENT_3D_QUALITY = 5

# Get the detection status/validity as one of the above numeric status values.
uint8 status

# A human-readable description of the status.
string status_description

# The centroid of the detected feature points in camera-space.
#
# Note: Lengths are given in units of meter. In the Zivid driver, the units are automatically converted from the default
# units of the Zivid point clouds in millimeter, to the ROS convention of meter.
geometry_msgs/Point centroid

# The position and orientation of the top left detected corner in the camera coordinate system.
#
# Note: Lengths are given in units of meter. In the Zivid driver, the units are automatically converted from the default
# units of the Zivid point clouds in millimeter, to the ROS convention of meter.
geometry_msgs/Pose pose

# The detected feature points in camera-space.
#
# Represents a list of 3D coordinates representing the centers of the calibration board squares. The points are arranged
# in a 2D grid corresponding to the calibration board, with a size given by `feature_points_width` and
# `feature_points_height`. The first element corresponds to the physical top-left corner of the board. Incrementing
# indices first move to the right on the physical board and then wraps down.
#
# Note: Lengths are given in units of meter. In the Zivid driver, the units are automatically converted from the default
# units of the Zivid point clouds in millimeter, to the ROS convention of meter.
geometry_msgs/Point[] feature_points

# Get the detected feature points in pixel-space.
#
# Same as `feature_points`, but with 2D coordinates instead of 3D coordinates. The points are reported at subpixel
# accuracy. The third dimension (`z`) of each point is set to zero.
geometry_msgs/Point[] feature_points_2d

# The width (number of columns) of the feature points.
uint64 feature_points_width

# The height (number of rows) of the feature points.
uint64 feature_points_height
