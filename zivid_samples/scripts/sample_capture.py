import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


class Sample(Node):

    def __init__(self):
        super().__init__('sample_capture_py')

        self.capture_service = self.create_client(Trigger, 'capture')
        while not self.capture_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Capture service not available, waiting again...')

        self._set_capture_settings()

        self.subscription = self.create_subscription(
            PointCloud2, 'points/xyzrgba', self.on_points, 10
        )

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
