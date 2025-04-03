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

}  // namespace zivid_camera
