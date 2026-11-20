# Temperature check of the camera health. See CameraHealth.msg.

# Status of the check. One of the CameraHealth.STATUS_* constants.
uint8 status

# True if `value_celsius` is set.
bool has_value

# Temperature in degrees Celsius. Only valid if `has_value` is true.
float64 value_celsius
