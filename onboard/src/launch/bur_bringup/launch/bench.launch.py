from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    def inc(pkg, rel, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(pkg), "launch", rel)
            ),
            **kwargs,
        )

    return LaunchDescription(
        [
            # With no EKF the controller can only publish MODE_COAST, and it
            # does so at 10 Hz on /gait_command. Turn it off to drive the gait
            # by hand with `ros2 topic pub`, or the two fight over the topic.
            DeclareLaunchArgument(
                "use_controller",
                default_value="true",
                description="Run bur_controller. Set false to publish "
                            "/gait_command by hand.",
            ),
            inc("bur_description", "description.launch.py"),
            inc("bur_gait", "gait.launch.py"),
            inc(
                "bur_controller",
                "controller.launch.py",
                condition=IfCondition(LaunchConfiguration("use_controller")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("bur_bringup"),
                        "rviz",
                        "bench.rviz",
                    ),
                ],
            ),
        ]
    )
