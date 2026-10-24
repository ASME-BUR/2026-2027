import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory("bur_description"), "urdf", "bur.urdf.xacro"
    )

    use_gui = LaunchConfiguration("use_gui")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="false",
                description="Run joint_state_publisher_gui instead of taking "
                            "joint states from bur_gait. Useful for checking "
                            "the URDF before any gait code runs.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            # bur_gait publishes /joint_states in normal operation. Only run
            # the GUI publisher when explicitly asked, or the two will fight.
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(use_gui),
            ),
        ]
    )
