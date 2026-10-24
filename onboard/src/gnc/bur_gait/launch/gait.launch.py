import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("bur_gait"), "config", "gait.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="bur_gait",
                executable="bur_gait",
                name="bur_gait",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            )
        ]
    )
