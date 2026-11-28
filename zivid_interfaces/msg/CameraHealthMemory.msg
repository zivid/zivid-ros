# Memory check of the camera health. See CameraHealth.msg.

# Status of the check. One of the CameraHealth.STATUS_* constants.
uint8 status

# True if `error_count` is set.
bool has_value

# Number of memory errors detected by the memory test. Only valid if `has_value` is true.
uint32 error_count
