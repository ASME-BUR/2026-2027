import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("bur_teleop"), "config", "ps4.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy",
                parameters=[config],
            ),
            Node(
                package="bur_teleop",
                executable="joy_to_swim",
                name="joy_to_swim",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
        ]
    )
