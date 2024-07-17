#!/usr/bin/env python

import sys

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


class Sample(Node):

    def __init__(self):
        super().__init__('sample_capture_with_settings_from_file_py')

        self.capture_service = self.create_client(Trigger, 'capture')
        while not self.capture_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Capture service not available, waiting again...')

        self._set_capture_settings()

        self.subscription = self.create_subscription(
            PointCloud2, 'points/xyzrgba', self.on_points, 10
        )

    def _set_capture_settings(self):
        path_to_settings_yml = (
            get_package_share_directory('zivid_samples')
            + '/settings/camera_settings.yml'
        )
        self.get_logger().info(
            'Setting parameter `settings_file_path` to: ' + path_to_settings_yml
        )

        settings_parameter = Parameter(
            'settings_file_path',
            Parameter.Type.STRING,
            path_to_settings_yml,
        ).to_parameter_msg()

        param_client = AsyncParameterClient(self, 'zivid_camera')
        param_client.wait_for_services()
        future = param_client.set_parameters([settings_parameter])
        rclpy.spin_until_future_complete(self, future)

    def capture(self):
        self.get_logger().info('Calling capture service')
        return self.capture_service.call_async(Trigger.Request())

    def on_points(self, msg):
        self.get_logger().info(
            f'Received point cloud of size {msg.width} x {msg.height}'
        )


def main(args=None):
    rclpy.init(args=args)

    try:
        sample = Sample()

        future = sample.capture()
        rclpy.spin_until_future_complete(sample, future)
        sample.get_logger().info('Capture complete')

        rclpy.spin(sample)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
