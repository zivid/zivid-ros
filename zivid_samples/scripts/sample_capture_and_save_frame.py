#!/usr/bin/env python

import rospy
import rosnode
import dynamic_reconfigure.client
from zivid_camera.srv import *
from std_msgs.msg import String


class Sample:
    def __init__(self):
        rospy.init_node("sample_capture_and_save_frame_py", anonymous=True)

        rospy.loginfo("Starting sample_capture_and_save_frame.py")

        rospy.wait_for_service("/zivid_camera/capture_and_save_frame", 30.0)

        self.capture_and_save_frame_service = rospy.ServiceProxy(
            "/zivid_camera/capture_and_save_frame", CaptureAndSaveFrame
        )

        self.enable_diagnostic()
        self.enable_first_acquistion()

    def capture(self):
        rospy.loginfo("Calling capture_and_save_frame service")
        file_path = "/tmp/capture_py.zdf"
        self.capture_and_save_frame_service(file_path)

    def enable_diagnostic(self):
        rospy.loginfo("Enabling the diagnostics mode")
        settings_client = dynamic_reconfigure.client.Client("/zivid_camera/settings/")
        settings_config = {"diagnostics_enabled": True}
        settings_client.update_configuration(settings_config)

    def enable_first_acquistion(self):
        rospy.loginfo("Enabling the first acquisition")
        acquisition_0_client = dynamic_reconfigure.client.Client(
            "/zivid_camera/settings/acquisition_0"
        )
        acquisition_0_config = {"enabled": True}
        acquisition_0_client.update_configuration(acquisition_0_config)


if __name__ == "__main__":
    s = Sample()
    s.capture()
    rospy.spin()
