import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _manager(context):
    share = get_package_share_directory("bur_autonomy")
    tree = LaunchConfiguration("tree").perform(context)
    if not os.path.isabs(tree):
        tree = os.path.join(share, "behavior_trees", tree)
    if not os.path.isfile(tree):
        raise RuntimeError(f"bur_autonomy: behaviour tree not found: {tree}")

    # D-7: plain launch_ros parameter overrides, not nav2's RewrittenYaml.
    overrides = {
        "behavior_tree": tree,
        "autostart": LaunchConfiguration("autostart").perform(context).lower() == "true",
    }
    mode = LaunchConfiguration("swim_forward_mode").perform(context)
    if mode:
        overrides["swim_forward_mode"] = mode

    return [
        Node(
            package="bur_autonomy",
            executable="manager",
            name="bur_autonomy",
            output="screen",
            parameters=[LaunchConfiguration("config").perform(context), overrides],
        )
    ]


def generate_launch_description():
    share = get_package_share_directory("bur_autonomy")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "tree", default_value="fish_tree_horizontal.xml",
                description="File in share/bur_autonomy/behavior_trees, or an absolute path."),
            DeclareLaunchArgument(
                "config", default_value=os.path.join(share, "config", "autonomy.yaml")),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument(
                "swim_forward_mode", default_value="",
                description="timed | odometry. Empty = the value in the config file."),
            OpaqueFunction(function=_manager),
        ]
    )
