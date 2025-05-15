#!/usr/bin/env python

import sys

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from zivid_interfaces.srv import ProjectionResolution, ProjectionStart


def declare_and_get_parameter(node, name):
    value = (
        node.declare_parameter(name, '', ParameterDescriptor(read_only=True))
        .get_parameter_value()
        .string_value
    )
    return value


def generate_image(width, height):
    data = []
    for i in range(0, height):
        for j in range(0, width):
            blue = min(int((256 * j) / width), 255)
            red = min(int((256 * (width - j)) / width), 255)
            green = min(int((256 * i) / height), 255)
            data.extend([blue, green, red, 255])

    return data


class Sample(Node):

    def __init__(self):
        super().__init__('sample_projection')

        self.project_service = self.create_client(ProjectionStart, 'projection/start')
        while not self.project_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('projection service not available, waiting again...')

        self.stop_service = self.create_client(Trigger, 'projection/stop')
        while not self.stop_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('projection service not available, waiting again...')

        self.resolution_service = self.create_client(
            ProjectionResolution, 'projection/resolution'
        )
        while not self.project_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('projection service not available, waiting again...')

    def get_resolution(self):
        self.get_logger().info('Calling resolution service')
        future = self.resolution_service.call_async(ProjectionResolution.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def project(self, image_path):
        if not image_path:
            r = self.get_resolution()
            request = ProjectionStart.Request(data=generate_image(r.width, r.height))
        else:
            request = ProjectionStart.Request(image_path=image_path)

        self.get_logger().info('Calling project service')
        return self.project_service.call_async(request)

    def stop(self):
        # self.get_logger().info('Calling stop service')
        # Cannot log; known issue: https://github.com/ros2/rclpy/issues/1287
        return self.stop_service.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)

    try:
        sample = Sample()
        image_path = declare_and_get_parameter(sample, 'image_path')

        future = sample.project(image_path)
        rclpy.spin_until_future_complete(sample, future)

        sample.get_logger().info('Spinning node for 5 seconds')
        rclpy.spin_until_future_complete(sample, rclpy.task.Future(), timeout_sec=5)

        sample.get_logger().info('Stopping projection')
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
