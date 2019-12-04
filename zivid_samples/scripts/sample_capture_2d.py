#!/usr/bin/env python

import rospy
import rosnode
import dynamic_reconfigure.client
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image


class Sample:
    def __init__(self):
        rospy.init_node("sample_capture_2d_py", anonymous=True)

        rospy.loginfo("Starting sample_capture_2d.py")

        rospy.wait_for_service("/zivid_camera/capture_2d", 30.0)

        rospy.Subscriber("/zivid_camera/color/image_color", Image, self.on_image_color)

        self.capture_2d_service = rospy.ServiceProxy(
            "/zivid_camera/capture_2d", Capture2D
        )

        rospy.loginfo("Configuring 2D settings")
        frame0_config_client = dynamic_reconfigure.client.Client(
            "/zivid_camera/capture_2d/frame_0"
        )
        frame0_config = {
            "enabled": True,
            "iris": 35,
            "exposure_time": 10000,
            "gain": 1.0,
            "brightness": 1.0,
        }
        frame0_config_client.update_configuration(frame0_config)

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.capture_2d_service()

    def on_image_color(self, data):
        rospy.loginfo("Color image received")
        self.capture()


if __name__ == "__main__":
    s = Sample()
    s.capture()
    rospy.spin()
