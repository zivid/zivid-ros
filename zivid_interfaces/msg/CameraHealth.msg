# Result of running the camera's health checks.
#
# Each check has a status (severity) and, when available, a measured value. The `overall` field aggregates the
# highest-precedence status among all checks. Precedence from highest to lowest is STATUS_ERROR, STATUS_UNKNOWN,
# STATUS_SUBOPTIMAL, STATUS_OK.
#
# A check that was not performed, or whose outcome could not be determined, has status STATUS_UNKNOWN. For example, all
# checks are STATUS_UNKNOWN for a file camera.
#
# The thresholds that determine each status are defined by the Zivid SDK and can change between SDK versions. See the
# Zivid SDK documentation of CameraHealth for the thresholds of the SDK in use.

# Optimal.
uint8 STATUS_OK = 0
# Degraded but functional.
uint8 STATUS_SUBOPTIMAL = 1
# Faulty or out of spec.
uint8 STATUS_ERROR = 2
# Check was not performed, or its outcome could not be determined.
uint8 STATUS_UNKNOWN = 3

# Aggregated status across all checks. One of the STATUS_* constants.
uint8 overall

# End-to-end link speed between the camera and the host computer, the minimum of the camera's and the PC's link speed.
#   - STATUS_OK: The link is fast enough for optimal performance.
#   - STATUS_SUBOPTIMAL: The link works, but limits performance.
#   - STATUS_ERROR: The link is too slow.
CameraHealthMaxTransferSpeed max_transfer_speed

# DMD (Digital Micromirror Device) temperature.
#   - STATUS_OK: Within the normal operating range.
#   - STATUS_SUBOPTIMAL: Elevated, but within the operating range.
#   - STATUS_ERROR: Outside the operating range.
CameraHealthTemperature temperature_dmd

# LED temperature.
#   - STATUS_OK: Within the normal operating range.
#   - STATUS_SUBOPTIMAL: Elevated, but within the operating range.
#   - STATUS_ERROR: Outside the operating range.
CameraHealthTemperature temperature_led

# Lens temperature.
#   - STATUS_OK: Within the normal operating range.
#   - STATUS_SUBOPTIMAL: Elevated, but within the operating range.
#   - STATUS_ERROR: Outside the operating range.
CameraHealthTemperature temperature_lens

# The camera's fan.
#   - STATUS_OK: Fan operating as expected.
#   - STATUS_ERROR: Unexpected fan stop.
CameraHealthFan fan

# The camera's memory test.
#   - STATUS_OK: Memory was tested and reported zero errors.
#   - STATUS_ERROR: Memory was tested and an error was located.
#   - STATUS_UNKNOWN: Memory test was not performed.
CameraHealthMemory memory

# The camera's most recent infield verification.
#   - STATUS_OK: The camera was verified in the field recently.
#   - STATUS_SUBOPTIMAL: The last verification is old.
#   - STATUS_UNKNOWN: The camera has never been verified in the field.
CameraHealthInfieldVerification infield_verification
