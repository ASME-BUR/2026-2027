from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # No more pi_address. This node must run ON the Pi.
    return LaunchDescription(
        [
            Node(
                package="bur_sensors",
                executable="bar30_node",
                name="bar30",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"i2c_bus": 1},
                    {"rate": 30},
                    # TODO(hw): set this for the pool you are actually testing in.
                    {"fluid_density": 1000.0},
                    {"depth_variance": 0.005},
                ],
            )
        ]
    )
