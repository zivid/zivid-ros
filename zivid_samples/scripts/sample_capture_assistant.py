#!/usr/bin/env python

import sys

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from zivid_interfaces.srv import CaptureAssistantSuggestSettings


class Sample(Node):

    def __init__(self):
        super().__init__('sample_capture_assistant_py')

        self.capture_assistant_suggest_settings_service = self.create_client(
            CaptureAssistantSuggestSettings, 'capture_assistant/suggest_settings'
        )
        while not self.capture_assistant_suggest_settings_service.wait_for_service(
            timeout_sec=3.0
        ):
            self.get_logger().info(
                'Capture assistant service not available, waiting again...'
            )

        self.capture_service = self.create_client(Trigger, 'capture')
        while not self.capture_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Capture service not available, waiting again...')

        self.points_subscription = self.create_subscription(
            PointCloud2, 'points/xyzrgba', self.on_points, 10
        )
        self.image_subscription = self.create_subscription(
            Image, 'color/image_color', self.on_image_color, 10
        )

    def capture_assistant_suggest_settings(self):
        self.get_logger().info('Calling capture assistant service')
        request = CaptureAssistantSuggestSettings.Request(
            max_capture_time=Duration(sec=2),
            ambient_light_frequency=(
                CaptureAssistantSuggestSettings.Request.AMBIENT_LIGHT_FREQUENCY_NONE
            ),
        )
        return self.capture_assistant_suggest_settings_service.call_async(request)

    def capture(self):
        self.get_logger().info('Calling capture service')
        return self.capture_service.call_async(Trigger.Request())

    def on_points(self, msg):
        self.get_logger().info(
            f'Received point cloud of size {msg.width} x {msg.height}'
        )

    def on_image_color(self, msg):
        self.get_logger().info(f'Received image of size {msg.width} x {msg.height}')


def main(args=None):
    rclpy.init(args=args)

    try:
        sample = Sample()

        future = sample.capture_assistant_suggest_settings()
        rclpy.spin_until_future_complete(sample, future, timeout_sec=30)
        if not future.result():
            raise RuntimeError('Failed to suggest settings')
        sample.get_logger().info('Capture assistant complete')

        future = sample.capture()
        rclpy.spin_until_future_complete(sample, future)
        sample.get_logger().info('Capture complete')

        sample.get_logger().info('Spinning node.. Press Ctrl+C to abort.')
        rclpy.spin(sample)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
