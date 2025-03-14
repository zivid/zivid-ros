# The ID of the fiducial marker to specify a position for.
int32 marker_id

# The position of the fiducial marker, specified at the center of the marker.
#
# For eye-in-hand calibration, positions should be given in the robot's base frame. For eye-to-hand calibration,
# positions should be given in the robot's end-effector frame.
#
# Note: Lengths should be given in units of meter. In the Zivid driver, this is converted internally to match the
# default units of Zivid point clouds which are given in millimeter.
geometry_msgs/Point position
