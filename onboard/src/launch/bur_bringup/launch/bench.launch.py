from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Dry bench bring-up: no sensors, no MCU, no EKF. Just the description,
    # the gait generator and RViz, so a gait can be watched in simulation
    # before anything is wired.
    #
    # This is the launch file to use first. Phase 1 of the plan is exactly
    # this: get a clean traveling wave visible before worrying about water.
    def inc(pkg, rel):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(pkg), "launch", rel)
            )
        )

    return LaunchDescription(
        [
            inc("bur_description", "description.launch.py"),
            inc("bur_gait", "gait.launch.py"),
            inc("bur_controller", "controller.launch.py"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                # TODO(port): ship an .rviz config with the robot model and TF
                # display already enabled, so this opens on something useful.
            ),
        ]
    )
