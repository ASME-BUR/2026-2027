import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # bur-onboard used an XML launch file with all 36 gains inline. Moved to a
    # YAML config so the gains can be edited without touching launch, and so
    # `ros2 param dump` round-trips.
    config = os.path.join(
        get_package_share_directory("bur_controller"), "config", "controller.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="bur_controller",
                executable="bur_controller",
                name="bur_controller",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            )
        ]
    )
