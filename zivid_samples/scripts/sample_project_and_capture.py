#!/usr/bin/env python

import sys

from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from zivid_interfaces.srv import (
    CameraInfoModelName,
    ProjectionResolution,
    ProjectionStart,
)


def generate_marker(width, height):
    data = width * height * 4 * [0]

    center_x = int(width / 2)
    center_y = int(height / 2)

    # Center
    for i, j in [[-1, 0], [1, 0], [0, 0], [0, -1], [0, 1]]:
        pos = 4 * ((center_y + i) * width + (center_x + j))
        data[pos + 1] = 255

    # Horizontal lines
    for j in range(center_x - 20, center_x - 5):
        pos = 4 * (center_y * width + j)
        data[pos + 1] = 255

    for j in range(center_x + 5, center_x + 20):
        pos = 4 * (center_y * width + j)
        data[pos + 1] = 255

    # Vertical lines
    for i in range(center_y - 20, center_y - 5):
        pos = 4 * (i * width + center_x)
        data[pos + 1] = 255

    for i in range(center_y + 5, center_y + 20):
        pos = 4 * (i * width + center_x)
        data[pos + 1] = 255

    return data


class Sample(Node):

    def __init__(self):
        super().__init__('sample_project')

        self.camera_info_service = self.create_client(
            CameraInfoModelName, 'camera_info/model_name'
        )
        while not self.camera_info_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info(
                'camera info service not available, waiting again...'
            )

        self.project_service = self.create_client(ProjectionStart, 'projection/start')
        while not self.project_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('projection service not available, waiting again...')

        self.capture_service = self.create_client(Trigger, 'projection/capture_2d')
        while not self.capture_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('projection service not available, waiting again...')

        response = self._get_model_name()

        self._set_settings_2d(response.model_name)

        self.stop_service = self.create_client(Trigger, 'projection/stop')
        while not self.stop_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('projection service not available, waiting again...')

        self.resolution_service = self.create_client(
            ProjectionResolution, 'projection/resolution'
        )
        while not self.project_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('projection service not available, waiting again...')

        self.subscription = self.create_subscription(
            Image, 'color/image_color', self.on_image_color, 10
        )

    def _get_model_name(self):
        self.get_logger().info('Calling camera_info service')
        future = self.camera_info_service.call_async(CameraInfoModelName.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _set_settings_2d(self, model_name):
        self.get_logger().info('Setting parameter `settings_2d_yaml`')

        settings = """
__version__: 7
Settings2D:
  Acquisitions:
    - Acquisition:
        Aperture: 3.36
        Brightness: 0
        ExposureTime: 20000
        Gain: 4
  Processing:
    Color:
      Balance:
        Blue: 1
        Green: 1
        Red: 1
      Experimental:
        Mode: automatic
      Gamma: 1
  Sampling:
    Color: rgb
    Pixel: all
"""
        if model_name in ['Zivid 2+ MR60', 'Zivid 2+ LR110', 'Zivid 2+ MR130']:
            settings = settings.replace('Color: rgb', 'Color: grayscale')

        settings_parameter = Parameter(
            'settings_2d_yaml', Parameter.Type.STRING, settings
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

    def get_resolution(self):
        self.get_logger().info('Calling resolution service')
        future = self.resolution_service.call_async(ProjectionResolution.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def project(self):
        r = self.get_resolution()
        request = ProjectionStart.Request(data=generate_marker(r.width, r.height))
        self.get_logger().info('Calling project service')
        return self.project_service.call_async(request)

    def stop(self):
        self.get_logger().info('Calling stop service')
        return self.stop_service.call_async(Trigger.Request())

    def capture(self):
        self.get_logger().info('Calling capture_2d service')
        return self.capture_service.call_async(Trigger.Request())

    def on_image_color(self, msg):
        self.get_logger().info(f'Received image of size {msg.width} x {msg.height}')


def main(args=None):
    rclpy.init(args=args)

    try:
        sample = Sample()

        future = sample.project()
        rclpy.spin_until_future_complete(sample, future)

        future = sample.capture()
        future = sample.stop()
        rclpy.spin_until_future_complete(sample, future)

        sample.get_logger().info('Spinning node.. Press Ctrl+C to abort.')
        rclpy.spin(sample)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
