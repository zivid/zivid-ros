#!/usr/bin/env python

import sys

from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class Sample(Node):

    def __init__(self):
        super().__init__('sample_capture_2d_py')

        self.capture_2d_service = self.create_client(Trigger, 'capture_2d')
        while not self.capture_2d_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('capture_2d service not available, waiting again...')

        self._set_settings_2d()

        self.subscription = self.create_subscription(
            Image, 'color/image_color', self.on_image_color, 10
        )

    def _set_settings_2d(self):
        self.get_logger().info('Setting parameter `settings_2d_yaml`')
        settings_parameter = Parameter(
            'settings_2d_yaml',
            Parameter.Type.STRING,
            """
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
    - Acquisition:
        Aperture: 2.83
        Brightness: 1.0
        ExposureTime: 10000
        Gain: 2.5
""",
        ).to_parameter_msg()
        param_client = self.create_client(SetParameters, 'zivid_camera/set_parameters')
        while not param_client.wait_for_service(timeout_sec=3):
            self.get_logger().info('Parameter service not available, waiting again...')

        future = param_client.call_async(
            SetParameters.Request(parameters=[settings_parameter])
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=30)
        if not future.result():
            raise RuntimeError('Failed to set parameters')

    def capture(self):
        self.get_logger().info('Calling capture_2d service')
        return self.capture_2d_service.call_async(Trigger.Request())

    def on_image_color(self, msg):
        self.get_logger().info(f'Received image of size {msg.width} x {msg.height}')


def main(args=None):
    rclpy.init(args=args)

    try:
        sample = Sample()

        sample.get_logger().info('Spinning node.. Press Ctrl+C to abort.')

        while rclpy.ok():
            future = sample.capture()
            rclpy.spin_until_future_complete(sample, future)
            sample.get_logger().info('Capture complete')

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
