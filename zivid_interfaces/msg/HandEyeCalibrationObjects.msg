uint8 CALIBRATION_BOARD = 1
uint8 FIDUCIAL_MARKERS = 2

# The type of calibration object to use: CALIBRATION_BOARD or FIDUCIAL_MARKERS
uint8 type


### For FIDUCIAL_MARKERS type

# A list of the IDs of markers to be detected.
int32[] marker_ids

# The marker dictionary that describes the appearance of each marker. Example: "aruco4x4_50".
string marker_dictionary
