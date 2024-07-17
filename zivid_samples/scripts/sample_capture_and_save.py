#!/usr/bin/env python

import sys
import tempfile

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from zivid_interfaces.srv import CaptureAndSave


class Sample(Node):

    def __init__(self):
        super().__init__('sample_capture_and_save_py')

        self.capture_and_save_service = self.create_client(
            CaptureAndSave, 'capture_and_save'
        )
        while not self.capture_and_save_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Capture service not available, waiting again...')

        self._set_capture_settings()

    def _set_capture_settings(self):
        self.get_logger().info('Setting parameter `settings_yaml`')
        settings_parameter = Parameter(
            'settings_yaml',
            Parameter.Type.STRING,
            """
__version__:
  serializer: 1
  data: 22
Settings:
  Acquisitions:
    - Acquisition:
        Aperture: 5.66
        ExposureTime: 8333
  Processing:
    Filters:
      Outlier:
        Removal:
          Enabled: yes
          Threshold: 5
""",
        ).to_parameter_msg()
        param_client = AsyncParameterClient(self, 'zivid_camera')
        param_client.wait_for_services()
        future = param_client.set_parameters([settings_parameter])
        rclpy.spin_until_future_complete(self, future)

    def capture(self):
        file_path = tempfile.gettempdir() + '/zivid_sample_capture_and_save.zdf'
        self.get_logger().info(f'Calling capture service with file path: {file_path}')
        request = CaptureAndSave.Request(file_path=file_path)
        return self.capture_and_save_service.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    try:
        sample = Sample()
        future = sample.capture()

        rclpy.spin_until_future_complete(sample, future)

        response: CaptureAndSave.Response = future.result()
        if not response.success:
            sample.get_logger().error(f'Failed capture and save: {response.message}')

        sample.get_logger().info('Capture and save complete')

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
