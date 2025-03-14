uint8 NONE = 0
uint8 CALIBRATION_BOARD = 1
uint8 FIDUCIAL_MARKERS = 2

# The type of calibration objects to use for low degrees-of-freedom (DOF) hand-eye calibration, if any:
# NONE (default) or CALIBRATION_BOARD or FIDUCIAL_MARKERS
# If set to NONE, regular (6-DOF) hand-eye calibration is performed.
uint8 type


### For CALIBRATION_BOARD type

# Specifies the fixed placement of a Zivid calibration board for low degrees-of-freedom hand-eye calibration.
# This field is not needed for regular (6-DOF) hand-eye calibration.
FixedPlacementOfCalibrationBoard calibration_board


### For FIDUCIAL_MARKERS type

# The dictionary that describes the appearance of the markers specified below. Example: "aruco4x4_50". Only applies to
# low degrees-of-freedom hand-eye calibration. This field is not needed for regular (6-DOF) hand-eye calibration.
string marker_dictionary

# A list describing the fixed placement of fiducial markers for low degrees-of-freedom hand-eye calibration.
# This field is not needed for regular (6-DOF) hand-eye calibration.
FixedPlacementOfFiducialMarker[] markers
