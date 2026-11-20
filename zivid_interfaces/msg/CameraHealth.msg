# Result of running the camera's health checks.
#
# Each check has a status (severity) and, when available, a measured value. The `overall` field aggregates the
# highest-precedence status among all checks. Precedence from highest to lowest is STATUS_ERROR, STATUS_UNKNOWN,
# STATUS_SUBOPTIMAL, STATUS_OK.
#
# A check that was not performed, or whose outcome could not be determined, has status STATUS_UNKNOWN. For example, all
# checks are STATUS_UNKNOWN for a file camera.

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
#   - STATUS_OK: Max transfer speed > 1 Gbps.
#   - STATUS_SUBOPTIMAL: Max transfer speed = 1 Gbps.
#   - STATUS_ERROR: Max transfer speed < 1 Gbps.
CameraHealthMaxTransferSpeed max_transfer_speed

# DMD (Digital Micromirror Device) temperature.
#   - STATUS_OK: Temperature in (0, 50) degrees Celsius.
#   - STATUS_SUBOPTIMAL: Temperature in [50, 60] degrees Celsius.
#   - STATUS_ERROR: Temperature < 0 or > 60 degrees Celsius.
CameraHealthTemperature temperature_dmd

# LED temperature.
#   - STATUS_OK: Temperature in (0, 60) degrees Celsius.
#   - STATUS_SUBOPTIMAL: Temperature in [60, 70] degrees Celsius.
#   - STATUS_ERROR: Temperature < 0 or > 70 degrees Celsius.
CameraHealthTemperature temperature_led

# Lens temperature.
#   - STATUS_OK: Temperature in (0, 50) degrees Celsius.
#   - STATUS_SUBOPTIMAL: Temperature in [50, 60] degrees Celsius.
#   - STATUS_ERROR: Temperature < 0 or > 60 degrees Celsius.
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
#   - STATUS_OK: Last verification performed within the past year.
#   - STATUS_SUBOPTIMAL: Last verification older than one year.
#   - STATUS_UNKNOWN: Camera has never been verified in the field.
CameraHealthInfieldVerification infield_verification
