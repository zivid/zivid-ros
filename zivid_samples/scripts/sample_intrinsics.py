#!/usr/bin/env python

import rospy
import rospkg
from zivid_camera.srv import *
from sensor_msgs.msg import CameraInfo


class Sample:
    def __init__(self):
        rospy.init_node("sample_capture_with_settings_from_yml_py", anonymous=True)

        rospy.loginfo("Starting sample_capture_with_settings_from_yml.py")

        rospy.wait_for_service("/zivid_camera/capture", 30.0)

        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)

        rospy.Subscriber(
            "/zivid_camera/color/camera_info", CameraInfo, self.on_color_camera_info
        )
        rospy.Subscriber(
            "/zivid_camera/depth/camera_info", CameraInfo, self.on_depth_camera_info
        )

        samples_path = rospkg.RosPack().get_path("zivid_samples")
        settings_path = samples_path + "/settings/camera_settings.yml"
        rospy.loginfo("Loading settings from %s", settings_path)
        self.load_settings_from_file_service = rospy.ServiceProxy(
            "/zivid_camera/load_settings_from_file", LoadSettingsFromFile
        )
        self.load_settings_from_file_service(settings_path)

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.capture_service()

    def on_color_camera_info(self, msg: CameraInfo):
        rospy.loginfo(
            f"""\
Received color camera info:
  width: {msg.width}
  height: {msg.height}
  distortion_model: {msg.distortion_model}
  D: [{msg.D}]
  K: [{msg.K}]
  R: [{msg.R}]
  P: [{msg.P}]
"""
        )

    def on_depth_camera_info(self, msg: CameraInfo):
        rospy.loginfo(
            f"""\
Received depth camera info:
  width: {msg.width}
  height: {msg.height}
  distortion_model: {msg.distortion_model}
  D: [{msg.D}]
  K: [{msg.K}]
  R: [{msg.R}]
  P: [{msg.P}]
"""
        )

    def set_intrinsics_source(self, value: str):
        param_name = (
            "/zivid_camera/zivid_camera/intrinsics_source"  # Node-private parameter
        )
        rospy.loginfo(f"Setting intrinsics source to: {value}")

        rospy.set_param(param_name, value)

        read_value = rospy.get_param(param_name, None)
        if read_value is not None:
            if read_value != value:
                rospy.logwarn(f"Expected '{value}' but got '{read_value}'")
            else:
                rospy.loginfo(f"Param set successfully, value: {read_value}")
        else:
            rospy.logwarn("Failed to read back param")


if __name__ == "__main__":
    s = Sample()
    s.set_intrinsics_source("camera")
    s.capture()
    s.set_intrinsics_source("frame")
    s.capture()

    rospy.spin()
