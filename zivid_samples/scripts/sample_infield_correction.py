#!/usr/bin/env python
from datetime import datetime
import enum
import sys

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.client import Client
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from zivid_interfaces.srv import (
    InfieldCorrectionCapture,
    InfieldCorrectionCompute,
    InfieldCorrectionRead,
)


class InfieldCorrectionOperation(enum.Enum):
    Verify = 'verify'
    Correct = 'correct'
    CorrectAndWrite = 'correct_and_write'
    Read = 'read'
    Reset = 'reset'


def capture_status_to_string(response):
    status_map = {
        InfieldCorrectionCapture.Response.STATUS_NOT_SET: 'STATUS_NOT_SET',
        InfieldCorrectionCapture.Response.STATUS_OK: 'STATUS_OK',
        InfieldCorrectionCapture.Response.STATUS_DETECTION_FAILED: 'STATUS_DETECTION_FAILED',
        InfieldCorrectionCapture.Response.STATUS_INVALID_CAPTURE_METHOD: 'STATUS_INVALID_CAPTURE_METHOD',  # noqa: E501
        InfieldCorrectionCapture.Response.STATUS_INVALID_ALIGNMENT: 'STATUS_INVALID_ALIGNMENT',
    }
    if response.status in status_map:
        return status_map[response.status]
    raise RuntimeError(f'Invalid status: {response.status}')


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


class Sample(Node):

    def __init__(self):
        super().__init__('sample_infield_correction_py')

    def _request_trigger_and_print_response(self, client):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if not future.done():
            raise RuntimeError(f'Failed to call the service {client.srv_name}')
        result = future.result()
        self.get_logger().info(f'Trigger results ({client.srv_name}):')
        self.get_logger().info(f'  Success: {result.success}')
        message_str = f'"""{result.message}"""' if result.message else ''
        self.get_logger().info(f'  Message: {message_str}')
        return result.success

    def _request_capture_and_print_response(self, client):
        future = client.call_async(InfieldCorrectionCapture.Request())
        rclpy.spin_until_future_complete(self, future)
        if not future.done():
            raise RuntimeError('Failed to call the infield correction capture service')
        result = future.result()
        self.get_logger().info(f'Trigger results ({client.srv_name}):')
        self.get_logger().info(f'  Success: {result.success}')
        message_str = f'"""{result.message}"""' if result.message else ''
        self.get_logger().info(f'  Message: {message_str}')
        self.get_logger().info(f'  Status: {capture_status_to_string(result)}')
        self.get_logger().info(f'  Number of captures: {result.number_of_captures}')
        return result.success

    def _request_infield_correction_compute_and_print_response(
        self, client: Client, name: str
    ):
        future = client.call_async(InfieldCorrectionCompute.Request())
        rclpy.spin_until_future_complete(self, future)
        if not future.done():
            raise RuntimeError(f'Failed to call the infield correction {name} service')
        result = future.result()
        self.get_logger().info(
            f'Infield correction compute results ({client.srv_name}):'
        )
        self.get_logger().info(f'  Success: {result.success}')
        self.get_logger().info(f'  Number of captures: {result.number_of_captures}')
        self.get_logger().info('  Trueness errors:')
        for i, error in enumerate(result.trueness_errors):
            self.get_logger().info(f'  - Capture {i + 1}: {error * 100.0} %')
        self.get_logger().info(
            f'  Average trueness error: {result.average_trueness_error * 100.0} %'
        )
        self.get_logger().info(
            f'  Maximum trueness error: {result.maximum_trueness_error * 100.0} %'
        )
        self.get_logger().info(
            f'  Dimension accuracy: {result.dimension_accuracy * 100.0} %'
        )
        self.get_logger().info(f'  Z min: {result.z_min} m')
        self.get_logger().info(f'  Z max: {result.z_max} m')
        message_str = f'"""{result.message}"""' if result.message else ''
        self.get_logger().info(f'  Message: {message_str}')
        return result.success

    def _request_read_and_print_response(self, client: Client):
        future = client.call_async(InfieldCorrectionRead.Request())
        rclpy.spin_until_future_complete(self, future)
        if not future.done():
            raise RuntimeError('Failed to call the infield correction read service')
        result = future.result()
        time_str = datetime.fromtimestamp(
            result.camera_correction_timestamp.sec
        ).strftime('%c')

        self.get_logger().info(f'Trigger results ({client.srv_name}):')
        self.get_logger().info(f'  Success: {result.success}')
        message_str = f'"""{result.message}"""' if result.message else ''
        self.get_logger().info(f'  Message: {message_str}')
        self.get_logger().info(
            f'  Has camera correction: {result.has_camera_correction}'
        )
        self.get_logger().info(
            f'  Camera correction timestamp: {result.camera_correction_timestamp.sec} ({time_str})'
        )
        return result.success

    def run_operation(self, operation: InfieldCorrectionOperation):
        if operation == InfieldCorrectionOperation.Verify:
            self.get_logger().info('--- Starting infield correction: Verify ---')
            start_client = self.create_client(Trigger, 'infield_correction/start')
            while not start_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info(
                    f'Service {start_client.srv_name} not available, waiting...'
                )

            capture_client = self.create_client(
                InfieldCorrectionCapture, 'infield_correction/capture'
            )
            compute_client = self.create_client(
                InfieldCorrectionCompute, 'infield_correction/compute'
            )

            if not self._request_trigger_and_print_response(start_client):
                raise RuntimeError('Could not start infield correction, aborting.')
            if not self._request_capture_and_print_response(capture_client):
                raise RuntimeError(
                    'Could not perform an infield correction capture, aborting.'
                )

            self._request_infield_correction_compute_and_print_response(
                compute_client, 'compute'
            )

        elif operation in [
            InfieldCorrectionOperation.Correct,
            InfieldCorrectionOperation.CorrectAndWrite,
        ]:
            self.get_logger().info('--- Starting infield correction ---')
            start_client = self.create_client(Trigger, 'infield_correction/start')
            while not start_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info(
                    f'Service {start_client.srv_name} not available, waiting...'
                )

            if not self._request_trigger_and_print_response(start_client):
                raise RuntimeError('Could not start infield correction, aborting.')

            capture_client = self.create_client(
                InfieldCorrectionCapture, 'infield_correction/capture'
            )
            compute_client = self.create_client(
                InfieldCorrectionCompute, 'infield_correction/compute'
            )

            wait_seconds = 5
            num_successful_captures_target = 3
            self.get_logger().info(
                f'--- Starting captures, will proceed until {num_successful_captures_target} '
                'captures have completed with a detected calibration board ---'
            )

            sleep_before_capture = False
            num_successful_captures = 0
            while num_successful_captures < num_successful_captures_target:
                if sleep_before_capture:
                    self.get_logger().info(
                        f'--- Waiting for {wait_seconds} seconds'
                        'before taking the next capture ---'
                    )
                    rclpy.spin_until_future_complete(
                        self, rclpy.task.Future(), None, wait_seconds
                    )
                else:
                    sleep_before_capture = True

                success = self._request_capture_and_print_response(capture_client)
                if success:
                    num_successful_captures += 1
                    self._request_infield_correction_compute_and_print_response(
                        compute_client, 'compute'
                    )

            self.get_logger().info('--- Captures complete ---')

            if operation == InfieldCorrectionOperation.CorrectAndWrite:
                self.get_logger().info('--- Writing correction results to camera ---')
                compute_and_write_client = self.create_client(
                    InfieldCorrectionCompute, 'infield_correction/compute_and_write'
                )
                self._request_infield_correction_compute_and_print_response(
                    compute_and_write_client, 'compute and write'
                )

        elif operation == InfieldCorrectionOperation.Read:
            self.get_logger().info('--- Starting infield correction: Read ---')
            read_client = self.create_client(
                InfieldCorrectionRead, 'infield_correction/read'
            )
            while not read_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info(
                    f'Service {read_client.srv_name} not available, waiting...'
                )
            self._request_read_and_print_response(read_client)

        elif operation == InfieldCorrectionOperation.Reset:
            self.get_logger().info('--- Starting infield correction: Reset ---')
            reset_client = self.create_client(Trigger, 'infield_correction/reset')
            while not reset_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info(
                    f'Service {reset_client.srv_name} not available, waiting...'
                )
            self._request_trigger_and_print_response(reset_client)

        else:
            raise RuntimeError(f'Unknown operation: {operation}')


def main(args=None):
    rclpy.init(args=args)

    try:
        sample = Sample()
        operation = declare_and_get_parameter_enum(
            sample, 'operation', InfieldCorrectionOperation
        )
        sample.run_operation(operation)
        sample.get_logger().info('Spinning node.. Press Ctrl+C to abort.')
        rclpy.spin(sample)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
