# Infield verification check of the camera health. See CameraHealth.msg.

# Status of the check. One of the CameraHealth.STATUS_* constants.
uint8 status

# True if `last_verification` is set.
bool has_value

# Time of the most recent infield verification. Only valid if `has_value` is true.
builtin_interfaces/Time last_verification
