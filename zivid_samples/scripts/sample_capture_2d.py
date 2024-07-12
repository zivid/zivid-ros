import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class Sample(Node):

    def __init__(self):
        super().__init__("sample_capture_2d_py")

        self.capture_2d_service = self.create_client(Trigger, "capture_2d")
        while not self.capture_2d_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Capture service not available, waiting again...")

        self._set_capture_settings()

        self.subscription = self.create_subscription(
            Image, "color/image_color", self.on_image_color, 10
        )

    def _set_capture_settings(self):
        self.get_logger().info("Setting parameter 'settings_2d_yaml'")
        settings_parameter = Parameter(
            "settings_2d_yaml",
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
        param_client = AsyncParameterClient(self, "zivid_camera")
        param_client.wait_for_services()
        future = param_client.set_parameters([settings_parameter])
        rclpy.spin_until_future_complete(self, future)

    def capture(self):
        self.get_logger().info("Calling capture service")
        return self.capture_2d_service.call_async(Trigger.Request())

    def on_image_color(self, msg):
        self.get_logger().info(f"Received image of size {msg.width} x {msg.height}")


def main(args=None):
    rclpy.init(args=args)

    try:
        sample = Sample()

        future = sample.capture()
        rclpy.spin_until_future_complete(sample, future)
        sample.get_logger().info("Capture complete")

        rclpy.spin(sample)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
