# Fan check of the camera health. See CameraHealth.msg.

# Fan operating as expected.
uint8 FAN_OK = 0
# Fan stopped unexpectedly.
uint8 FAN_UNEXPECTED_STOP = 1

# Status of the check. One of the CameraHealth.STATUS_* constants.
uint8 status

# True if `value` is set.
bool has_value

# Whether the fan is operating as expected, or stopped unexpectedly. One of the FAN_* constants. Only valid if
# `has_value` is true.
uint8 value
