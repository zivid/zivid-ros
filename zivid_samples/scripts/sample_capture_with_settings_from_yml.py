#!/usr/bin/env python

import rospy
import rospkg
import rosnode
import dynamic_reconfigure.client
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2


class Sample:
    def __init__(self):
        rospy.init_node("sample_capture_with_settings_from_yml_py", anonymous=True)

        rospy.loginfo("Starting sample_capture_with_settings_from_yml.py")

        rospy.wait_for_service("/zivid_camera/capture", 30.0)

        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)

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


if __name__ == "__main__":
    s = Sample()
    s.capture()
    rospy.spin()
