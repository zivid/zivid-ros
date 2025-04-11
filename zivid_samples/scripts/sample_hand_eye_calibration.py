#!/usr/bin/env python
import enum
import math
import sys
from typing import List

from geometry_msgs.msg import Pose
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.client import Client
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from zivid_interfaces.msg import HandEyeCalibrationObjects
from zivid_interfaces.srv import (
    HandEyeCalibrationCalibrate,
    HandEyeCalibrationCapture,
    HandEyeCalibrationStart,
)


class HandEyeConfiguration(enum.Enum):
    EyeToHand = 'eye_to_hand'
    EyeInHand = 'eye_in_hand'


def declare_and_get_parameter_enum(node, name, enum_type):
    name_value_map = {op.value: op for op in enum_type}
    value = (
        node.declare_parameter(name, '', ParameterDescriptor(read_only=True))
        .get_parameter_value()
        .string_value
    )
    if value not in name_value_map:
        valid_values = ', '.join(list(name_value_map.keys()))
        raise RuntimeError(
            f"Invalid value for parameter '{name}': '{value}'. Expected one of: {valid_values}."
        )
    node.get_logger().info(f'Got parameter {name}: {value}')
    return name_value_map[value]


def sanitize_marker_ids(marker_ids):
    # We use an integer array with a single value [-1] as a placeholder for an empty array, since
    # an empty integer array cannot be specified directly.
    # See: https://github.com/ros2/rclcpp/issues/1955
    if list(marker_ids) == [-1]:
        return []
    for marker_id in marker_ids:
        if marker_id < 0:
            raise RuntimeError(
                f'Invalid marker ID: {marker_id}. Negative marker IDs are not allowed'
            )
    return marker_ids


def get_simulated_robot_pose(simulated_time):
    # This simulated response is for exposition only. On a real system, this method should be
    # replaced with the actual robot pose.
    pose = Pose()
    yaw = simulated_time * 0.1
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    pose.position.x = math.sin(simulated_time)
    return pose


class Sample(Node):

    def __init__(self):
        super().__init__('sample_hand_eye_calibration_py')

    def _request_start(
        self, client: Client, marker_ids: List[int], working_directory: str
    ) -> bool:
        request = HandEyeCalibrationStart.Request()

        if working_directory:
            self.get_logger().info(f'Setting working directory to: {working_directory}')
            request.working_directory = working_directory

        if not marker_ids:
            request.calibration_objects.type = (
                HandEyeCalibrationObjects.CALIBRATION_BOARD
            )
        else:
            request.calibration_objects.type = (
                HandEyeCalibrationObjects.FIDUCIAL_MARKERS
            )
            request.calibration_objects.marker_dictionary = 'aruco4x4_50'
            request.calibration_objects.marker_ids = marker_ids

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        self.get_logger().info(f'Hand-eye calibration start ({client.srv_name}):')
        self.get_logger().info(f'  Success: {result.success}')
        if result.message:
            self.get_logger().info(f'  Message: """{result.message}"""')
        else:
            self.get_logger().info('  Message: ')

        return result.success

    def request_capture_and_print_response(self, client: Client, pose: Pose) -> bool:
        request = HandEyeCalibrationCapture.Request()
        request.robot_pose = pose

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        self.get_logger().info(f'Hand-eye calibration capture ({client.srv_name}):')
        self.get_logger().info(f'  Success: {result.success}')
        if result.message:
            self.get_logger().info(f'  Message: """{result.message}"""')
        else:
            self.get_logger().info('  Message: ')
        self.get_logger().info(f'  Capture handle: {result.capture_handle}')

        return result.success

    def request_calibration_and_print_response(
        self, client: Client, configuration: HandEyeConfiguration
    ) -> bool:
        self.get_logger().info('--- Starting hand-eye calibration ---')
        request = HandEyeCalibrationCalibrate.Request()
        request.configuration = (
            HandEyeCalibrationCalibrate.Request.EYE_TO_HAND
            if configuration == HandEyeConfiguration.EyeToHand
            else HandEyeCalibrationCalibrate.Request.EYE_IN_HAND
        )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        self.get_logger().info(f'Hand-eye calibration capture ({client.srv_name}):')
        self.get_logger().info(f'  Success: {result.success}')
        self.get_logger().info(
            '  Message: ' + (f'"""{result.message}"""' if result.message else '')
        )
        self.get_logger().info(
            f'  Transform: {{ translation in m: {{{result.transform.translation}}}, '
            f'rotation: {{{result.transform.rotation}}}'
        )

        residuals_str = ', '.join(
            f'{{ rotation in deg: {residual.rotation}, translation in m: {residual.translation} }}'
            for residual in result.residuals
        )
        self.get_logger().info(f'  Residuals: {residuals_str}')

        return result.success

    def _set_settings(self):
        # The following settings are for exposition only. Please refer to the Zivid knowledge base
        # on how to get good quality data for hand-eye calibration:
        # https://support.zivid.com/en/latest/academy/applications/hand-eye/how-to-get-good-quality-data-on-zivid-calibration-board.html
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

        param_client = self.create_client(SetParameters, 'zivid_camera/set_parameters')
        while not param_client.wait_for_service(timeout_sec=3):
            self.get_logger().info('Parameter service not available, waiting again...')

        future = param_client.call_async(
            SetParameters.Request(parameters=[settings_parameter])
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=30)
        if not future.result():
            raise RuntimeError('Failed to set parameters')

    def _start_hand_eye_calibration(
        self, marker_ids: List[int], working_directory: str
    ):
        start_client = self.create_client(
            HandEyeCalibrationStart, 'hand_eye_calibration/start'
        )
        while not start_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info(
                f'Service {start_client.srv_name} not available, waiting...'
            )

        self.get_logger().info('--- Starting hand-eye calibration ---')
        self._request_start(start_client, marker_ids, working_directory)
        self._set_settings()

    def _perform_captures_for_hand_eye_calibration(self):
        captures_client = self.create_client(
            HandEyeCalibrationCapture, 'hand_eye_calibration/capture'
        )
        while not captures_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info(
                f'Service {captures_client.srv_name} not available, waiting...'
            )

        wait_seconds = 5
        num_successful_captures_target = 6
        self.get_logger().info(
            f'--- Starting captures, will proceed until {num_successful_captures_target}'
            ' captures have completed with a detected calibration object ---'
        )

        sleep_before_capture = False
        num_successful_captures = 0
        while num_successful_captures < num_successful_captures_target:
            if sleep_before_capture:
                self.get_logger().info(
                    f'--- Waiting for {wait_seconds} seconds '
                    'before taking the next capture ---'
                )
                rclpy.spin_until_future_complete(
                    self, rclpy.task.Future(), None, wait_seconds
                )
            else:
                sleep_before_capture = True
            simulated_time = 0.5 * num_successful_captures
            robot_pose = get_simulated_robot_pose(simulated_time)
            if self.request_capture_and_print_response(captures_client, robot_pose):
                num_successful_captures += 1

    def _perform_calibration(self, configuration: HandEyeConfiguration):
        calibrate_client = self.create_client(
            HandEyeCalibrationCalibrate, 'hand_eye_calibration/calibrate'
        )
        while not calibrate_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info(
                f'Service {calibrate_client.srv_name} not available, waiting...'
            )
        self.get_logger().info(
            '--- Performing hand-eye calibration with captured data ---'
        )
        self.request_calibration_and_print_response(calibrate_client, configuration)
        self.get_logger().info('--- Calibration complete ---')

    def run_hand_eye_calibration(
        self,
        configuration: HandEyeConfiguration,
        marker_ids: List[int],
        working_directory: str,
    ):
        self._start_hand_eye_calibration(marker_ids, working_directory)

        self._perform_captures_for_hand_eye_calibration()

        self._perform_calibration(configuration)


def main(args=None):
    rclpy.init(args=args)

    try:
        sample = Sample()
        configuration = declare_and_get_parameter_enum(
            sample, 'configuration', HandEyeConfiguration
        )
        marker_ids = sanitize_marker_ids(
            sample.declare_parameter(
                'marker_ids', [-1], ParameterDescriptor(read_only=True)
            )
            .get_parameter_value()
            .integer_array_value
        )
        working_directory = (
            sample.declare_parameter(
                'working_directory', '', ParameterDescriptor(read_only=True)
            )
            .get_parameter_value()
            .string_value
        )

        sample.run_hand_eye_calibration(configuration, marker_ids, working_directory)

        sample.get_logger().info('Spinning node.. Press Ctrl+C to abort.')
        rclpy.spin(sample)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
