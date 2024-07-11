from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    launch_description.add_action(
        Node(package="zivid_camera", executable="zivid_camera", name="zivid_camera")
    )

    launch_description.add_action(
        Node(
            package="zivid_samples",
            executable="sample_capture_py",
            name="zivid_sample",
            output="screen",
            emulate_tty=True,
        ),
    )

    return launch_description
