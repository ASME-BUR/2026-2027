import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(pkg, rel, condition=None):
    src = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(pkg), "launch", rel)
    )
    return IncludeLaunchDescription(src, condition=condition)


def generate_launch_description():
    # The full onboard stack. Everything defaults ON except the pieces that
    # need hardware this workspace cannot assume is present.
    #
    # Note what is NOT here: there is no separate aggregator node. bur-onboard
    # routed everything through joy_command, which meant the vehicle could not
    # run without a gamepad. bur_controller now takes odometry directly.

    use_sensors = LaunchConfiguration("use_sensors")
    use_micro_ros = LaunchConfiguration("use_micro_ros")
    use_teleop = LaunchConfiguration("use_teleop")
    use_gait_node = LaunchConfiguration("use_gait_node")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sensors", default_value="true",
                description="Bar30 over local I2C. Requires running on the Pi."),
            DeclareLaunchArgument(
                "use_micro_ros", default_value="true",
                description="micro-ROS agents for the MCU link."),
            DeclareLaunchArgument(
                "use_teleop", default_value="false",
                description="Gamepad. Off by default so autonomy can run alone."),
            DeclareLaunchArgument(
                "use_gait_node", default_value="true",
                description="Run the Pi-side gait generator. Set false once "
                            "the MCU owns the phase and subscribes to "
                            "/gait_command directly - otherwise both will "
                            "drive the servos."),

            # always: description + state estimation + outer loop
            _include("bur_description", "description.launch.py"),
            _include("bur_localization", "ekf.launch.py"),
            _include("bur_controller", "controller.launch.py"),

            _include("bur_gait", "gait.launch.py",
                     condition=IfCondition(use_gait_node)),
            _include("bur_sensors", "bar30.launch.py",
                     condition=IfCondition(use_sensors)),
            _include("bur_bringup", "micro_ros.launch.py",
                     condition=IfCondition(use_micro_ros)),
            _include("bur_teleop", "teleop.launch.py",
                     condition=IfCondition(use_teleop)),

            # TODO(port): camera. bur-onboard used klintan/ros2_usb_camera
            # pinned to foxy-devel, which will not build on Jazzy, and it was
            # commented out of both launch files anyway. On the new vehicle the
            # camera is the ZED on the Jetson, launched over there, not here.
            #
            # TODO(port): bur-onboard had a topic_viewer TUI in bur_bringup that
            # printed command/control_effort/thruster values live. It was
            # genuinely useful during pool tests. Worth rewriting against
            # /swim_command, /gait_command and /servo_chain_state.
        ]
    )
