// Copyright 2025 Zivid AS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Zivid AS nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <Zivid/Calibration/DetectionResult.h>
#include <Zivid/Calibration/Pose.h>
#include <Zivid/Matrix.h>
#include <Zivid/Point.h>

#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <zivid_camera/utility.hpp>

namespace zivid_camera
{
namespace
{
Zivid::Matrix4x4 transposeMatrix(const Zivid::Matrix4x4 & value)
{
  Zivid::Matrix4x4 transposed;
  for (size_t i = 0; i < Zivid::Matrix4x4::rows; ++i) {
    for (size_t j = 0; j < Zivid::Matrix4x4::cols; ++j) {
      transposed(i, j) = value(j, i);
    }
  }
  return transposed;
}
}  // namespace

Zivid::Calibration::Pose toZividPose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform tf2Transform;
  tf2::fromMsg(pose, tf2Transform);
  std::array<tf2Scalar, 16> tf2_transform = {};
  tf2Transform.getOpenGLMatrix(tf2_transform.data());

  auto transform = transposeMatrix(Zivid::Matrix4x4{tf2_transform.begin(), tf2_transform.end()});
  transform.at(0, 3) = rosLengthToZivid(tf2Transform.getOrigin().x());
  transform.at(1, 3) = rosLengthToZivid(tf2Transform.getOrigin().y());
  transform.at(2, 3) = rosLengthToZivid(tf2Transform.getOrigin().z());
  return Zivid::Calibration::Pose{transform};
}

Zivid::PointXYZ toZividPoint(const geometry_msgs::msg::Point & point)
{
  return Zivid::PointXYZ{
    rosLengthToZivid(point.x),
    rosLengthToZivid(point.y),
    rosLengthToZivid(point.z),
  };
}

geometry_msgs::msg::Transform toGeometryMsgTransform(const Zivid::Matrix4x4 & transform)
{
  const tf2::Matrix3x3 rotation{
    static_cast<double>(transform(0, 0)), static_cast<double>(transform(0, 1)),
    static_cast<double>(transform(0, 2)), static_cast<double>(transform(1, 0)),
    static_cast<double>(transform(1, 1)), static_cast<double>(transform(1, 2)),
    static_cast<double>(transform(2, 0)), static_cast<double>(transform(2, 1)),
    static_cast<double>(transform(2, 2))};

  tf2::Quaternion q;
  rotation.getRotation(q);

  geometry_msgs::msg::Transform result;
  result.translation.x = zividLengthToRos(transform(0, 3));
  result.translation.y = zividLengthToRos(transform(1, 3));
  result.translation.z = zividLengthToRos(transform(2, 3));
  result.rotation.x = q.x();
  result.rotation.y = q.y();
  result.rotation.z = q.z();
  result.rotation.w = q.w();
  return result;
}

geometry_msgs::msg::Pose toGeometryMsgPose(const Zivid::Calibration::Pose & pose)
{
  const auto transform = toGeometryMsgTransform(pose.toMatrix());
  geometry_msgs::msg::Pose result;
  result.orientation = transform.rotation;
  result.position.x = transform.translation.x;
  result.position.y = transform.translation.y;
  result.position.z = transform.translation.z;
  return result;
}

geometry_msgs::msg::Point toGeometryMsgPoint(const Zivid::PointXYZ & point)
{
  geometry_msgs::msg::Point result;
  result.x = zividLengthToRos(point.x);
  result.y = zividLengthToRos(point.y);
  result.z = zividLengthToRos(point.z);
  return result;
}

geometry_msgs::msg::Point pixelCoordinatesToGeometryMsgPoint(
  const Zivid::PointXY & pixel_coordinates)
{
  geometry_msgs::msg::Point result;
  result.x = static_cast<double>(pixel_coordinates.x);
  result.y = static_cast<double>(pixel_coordinates.y);
  result.z = 0;
  return result;
}

zivid_interfaces::msg::DetectionResultCalibrationBoard toZividMsgDetectionResult(
  const Zivid::Calibration::DetectionResult & detection)
{
  using zivid_interfaces::msg::DetectionResultCalibrationBoard;
  DetectionResultCalibrationBoard result;

  result.status = [status = detection.status()] {
    using Zivid::Calibration::CalibrationBoardDetectionStatus;
    switch (status) {
      case CalibrationBoardDetectionStatus::ok:
        return DetectionResultCalibrationBoard::STATUS_OK;
      case CalibrationBoardDetectionStatus::noValidFiducialMarkerDetected:
        return DetectionResultCalibrationBoard::STATUS_NO_VALID_FIDUCIAL_MARKER_DETECTED;
      case CalibrationBoardDetectionStatus::multipleValidFiducialMarkersDetected:
        return DetectionResultCalibrationBoard::STATUS_MULTIPLE_VALID_FIDUCIAL_MARKERS_DETECTED;
      case CalibrationBoardDetectionStatus::boardDetectionFailed:
        return DetectionResultCalibrationBoard::STATUS_BOARD_DETECTION_FAILED;
      case CalibrationBoardDetectionStatus::insufficient3DQuality:
        return DetectionResultCalibrationBoard::STATUS_INSUFFICIENT_3D_QUALITY;
      default:
        throw std::runtime_error(
          "Unhandled detection status: " + std::to_string(static_cast<int>(status)));
    }
  }();

  result.status_description = detection.statusDescription();

  if (result.status != DetectionResultCalibrationBoard::STATUS_OK) {
    return result;
  }

  result.pose = toGeometryMsgPose(detection.pose());
  result.centroid = toGeometryMsgPoint(detection.centroid());

  const auto detection_feature_points = detection.featurePoints();
  const auto detection_feature_points_2d = detection.featurePoints2D();
  if (
    detection_feature_points.width() != detection_feature_points_2d.width() ||
    detection_feature_points.height() != detection_feature_points_2d.height()) {
    throw std::runtime_error(
      "Expected the feature points and feature points 2D to have the same dimensions");
  }

  result.feature_points.reserve(detection_feature_points.size());
  std::transform(
    detection_feature_points.begin(), detection_feature_points.end(),
    std::back_inserter(result.feature_points),
    [](const Zivid::PointXYZ & point) { return toGeometryMsgPoint(point); });

  result.feature_points_2d.reserve(detection_feature_points_2d.size());
  std::transform(
    detection_feature_points_2d.begin(), detection_feature_points_2d.end(),
    std::back_inserter(result.feature_points_2d),
    [](const Zivid::PointXY & point) { return pixelCoordinatesToGeometryMsgPoint(point); });

  result.feature_points_width = detection_feature_points.width();
  result.feature_points_height = detection_feature_points.height();

  return result;
}

zivid_interfaces::msg::DetectionResultFiducialMarkers toZividMsgDetectionResult(
  const Zivid::Calibration::DetectionResultFiducialMarkers & detection)
{
  std::vector<zivid_interfaces::msg::MarkerShape> markers;

  for (const auto & detected_marker : detection.detectedMarkers()) {
    zivid_interfaces::msg::MarkerShape & marker = markers.emplace_back();

    const auto corners_in_camera_coordinates = detected_marker.cornersInCameraCoordinates();
    static_assert(
      std::tuple_size_v<decltype(corners_in_camera_coordinates)> ==
      std::tuple_size_v<decltype(marker.corners_in_camera_coordinates)>);
    std::transform(
      corners_in_camera_coordinates.begin(), corners_in_camera_coordinates.end(),
      marker.corners_in_camera_coordinates.begin(),
      [](const Zivid::PointXYZ & point) { return toGeometryMsgPoint(point); });

    const auto corners_in_pixel_coordinates = detected_marker.cornersInPixelCoordinates();
    static_assert(
      std::tuple_size_v<decltype(corners_in_pixel_coordinates)> ==
      std::tuple_size_v<decltype(marker.corners_in_pixel_coordinates)>);
    std::transform(
      corners_in_pixel_coordinates.begin(), corners_in_pixel_coordinates.end(),
      marker.corners_in_pixel_coordinates.begin(),
      [](const Zivid::PointXY & point) { return pixelCoordinatesToGeometryMsgPoint(point); });

    marker.id = detected_marker.id();
    marker.pose = toGeometryMsgPose(detected_marker.pose());
  }

  zivid_interfaces::msg::DetectionResultFiducialMarkers result;
  result.detected_markers = std::move(markers);
  result.allowed_marker_ids = detection.allowedMarkerIds();
  return result;
}

}  // namespace zivid_camera
