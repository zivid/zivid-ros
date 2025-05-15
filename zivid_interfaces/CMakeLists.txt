cmake_minimum_required(VERSION 3.5)

project(zivid_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(zivid_interfaces
                           "msg/DetectionResultCalibrationBoard.msg"
                           "msg/DetectionResultFiducialMarkers.msg"
                           "msg/FixedPlacementOfCalibrationBoard.msg"
                           "msg/FixedPlacementOfCalibrationObjects.msg"
                           "msg/FixedPlacementOfFiducialMarker.msg"
                           "msg/HandEyeCalibrationObjects.msg"
                           "msg/HandEyeCalibrationResidual.msg"
                           "msg/MarkerShape.msg"
                           "srv/CaptureAndDetectCalibrationBoard.srv"
                           "srv/CaptureAndDetectMarkers.srv"
                           "srv/CaptureAndSave.srv"
                           "srv/CaptureAssistantSuggestSettings.srv"
                           "srv/CameraInfoModelName.srv"
                           "srv/CameraInfoSerialNumber.srv"
                           "srv/HandEyeCalibrationCalibrate.srv"
                           "srv/HandEyeCalibrationCapture.srv"
                           "srv/HandEyeCalibrationLoad.srv"
                           "srv/HandEyeCalibrationStart.srv"
                           "srv/InfieldCorrectionCapture.srv"
                           "srv/InfieldCorrectionCompute.srv"
                           "srv/InfieldCorrectionRead.srv"
                           "srv/IsConnected.srv"
                           "srv/ProjectionResolution.srv"
                           "srv/ProjectionStart.srv"
                           "srv/ProjectionStatus.srv"
                           DEPENDENCIES
                           builtin_interfaces
                           geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
