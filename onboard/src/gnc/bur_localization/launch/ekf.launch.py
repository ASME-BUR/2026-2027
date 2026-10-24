import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # bur-onboard built its entire TF tree out of five chained
    # static_transform_publisher nodes, because there was no URDF. That works
    # for a rigid body with bolted-on sensors; it cannot describe a chain of
    # moving joints.
    #
    # The articulated part of the tree now comes from bur_description via
    # robot_state_publisher (see bur_description/launch/description.launch.py).
    # Only genuinely fixed sensor offsets stay here.
    #
    # TODO(hw): every offset below is inherited from LAST YEAR'S HULL and is
    # certainly wrong. Measure them on the new vehicle. The IMU one matters
    # most: it should sit in the rigid nose section, and whatever yaw offset it
    # ends up with has to be right or the heading loop chases its own tail.

    params = os.path.join(
        get_package_share_directory("bur_localization"), "params", "ekf.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[params],
            ),
            # TODO(hw): placeholder, carried over from bur-onboard.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu",
                arguments=[
                    "--x", "-0.1", "--y", "0.0", "--z", "0.0",
                    "--yaw", "3.14159", "--pitch", "0", "--roll", "0",
                    "--frame-id", "base_link", "--child-frame-id", "imu",
                ],
            ),
            # TODO(hw): placeholder, carried over from bur-onboard.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_depth",
                arguments=[
                    "--x", "-0.2", "--y", "0.0", "--z", "0.05",
                    "--yaw", "0", "--pitch", "0", "--roll", "0",
                    "--frame-id", "base_link", "--child-frame-id", "depth_sensor",
                ],
            ),
            # map -> odom. The EKF publishes odom -> base_link itself, so
            # bur-onboard's static odom -> base_link publisher is GONE: having
            # both meant two nodes fighting over the same transform.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--yaw", "0", "--pitch", "0", "--roll", "0",
                    "--frame-id", "map", "--child-frame-id", "odom",
                ],
            ),
        ]
    )
