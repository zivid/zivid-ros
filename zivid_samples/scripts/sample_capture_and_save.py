#!/usr/bin/env python

import rospy
import rosnode
import dynamic_reconfigure.client
from zivid_camera.srv import *
from std_msgs.msg import String


class Sample:
    def __init__(self):
        rospy.init_node("sample_capture_and_save_py", anonymous=True)

        rospy.loginfo("Starting sample_capture_and_save.py")

        rospy.wait_for_service("/zivid_camera/capture_and_save", 30.0)

        self.capture_and_save_service = rospy.ServiceProxy(
            "/zivid_camera/capture_and_save", CaptureAndSave
        )

        self._enable_first_acquistion()
        self._enable_diagnostics()

    def capture(self):
        rospy.loginfo("Calling capture_and_save service")
        file_path = "/tmp/capture_py.zdf"
        self.capture_and_save_service(file_path)
        rospy.loginfo(f"Your .zdf file is now available here: {file_path}")

    def _enable_first_acquistion(self):
        rospy.loginfo("Enabling the first acquisition")
        acquisition_0_client = dynamic_reconfigure.client.Client(
            "/zivid_camera/settings/acquisition_0"
        )
        acquisition_0_config = {"enabled": True}
        acquisition_0_client.update_configuration(acquisition_0_config)

    def _enable_diagnostics(self):
        rospy.loginfo("Enabling diagnostics mode")
        settings_client = dynamic_reconfigure.client.Client("/zivid_camera/settings/")
        settings_config = {"diagnostics_enabled": True}
        settings_client.update_configuration(settings_config)


if __name__ == "__main__":
    s = Sample()
    s.capture()
    rospy.spin()
