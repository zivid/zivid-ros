uint8 POSITION = 1
uint8 POSE = 2

# The representation to use for the placement of the fixed calibration board: POSITION or POSE
uint8 representation


### For POSITION representation

# Position of the calibration board's origin.
#
# The origin is the top left inner corner of the calibration board.
#
# For eye-in-hand calibration, the position should be given in the robot's base frame. For eye-to-hand calibration, the
# position should be given in the robot's end-effector frame.
#
# Note: Lengths should be given in units of meter. In the Zivid driver, this is converted internally to match the
# default units of Zivid point clouds which are given in millimeter.
geometry_msgs/Point position


### For POSE representation

# Pose of the calibration board.
#
# Using a pose instead of a position can improve accuracy of the hand-eye calibration in some situations. For
# eye-in-hand calibration, the pose should be given in the robot's base frame. For eye-to-hand calibration, the pose
# should be given in the robot's end-effector frame.
#
# Note: Lengths should be given in units of meter. In the Zivid driver, this is converted internally to match the
# default units of Zivid point clouds which are given in millimeter.
geometry_msgs/Pose pose
