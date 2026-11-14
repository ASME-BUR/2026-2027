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
    #
    # use_autonomy:=true adds the autonomy manager (A-1) and bench_sim, a
    # kinematic stand-in for the vehicle, the EKF and the camera. bench_sim
    # turns /gait_command into /odometry/filtered and /vision, so the whole
    # chain runs: tree -> /swim_command -> bur_controller -> /gait_command ->
    # bench_sim -> odometry -> tree. The body visibly swims in RViz, which
    # uses a second config fixed to `odom`. Its numbers are invented.
    def inc(pkg, rel, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(pkg), "launch", rel)
            ),
            **kwargs,
        )

    rviz_dir = os.path.join(get_package_share_directory("bur_bringup"), "rviz")
    use_autonomy = LaunchConfiguration("use_autonomy")
    use_rviz = LaunchConfiguration("use_rviz")

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
            DeclareLaunchArgument(
                "use_autonomy",
                default_value="false",
                description="Run the autonomy manager against bench_sim "
                            "(kinematic stand-in, invented numbers).",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true",
                description="false for a headless check."),
            inc("bur_description", "description.launch.py"),
            inc("bur_gait", "gait.launch.py"),
            inc(
                "bur_controller",
                "controller.launch.py",
                condition=IfCondition(LaunchConfiguration("use_controller")),
            ),
            # ---- autonomy on the bench (A-1 done-when) ----
            Node(
                package="bur_autonomy",
                executable="bench_sim",
                name="bench_sim",
                output="screen",
                condition=IfCondition(use_autonomy),
            ),
            inc(
                "bur_autonomy",
                "autonomy.launch.py",
                launch_arguments={
                    "autostart": "true",
                    # bench_sim's odometry is exact, so distance can be measured
                    "swim_forward_mode": "odometry",
                }.items(),
                condition=IfCondition(use_autonomy),
            ),
            # ---- RViz: base_link-fixed without autonomy (no odom frame exists),
            # odom-fixed with it ----
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(rviz_dir, "bench.rviz")],
                condition=IfCondition(
                    _and(use_rviz, _not(use_autonomy))),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(rviz_dir, "bench_autonomy.rviz")],
                condition=IfCondition(_and(use_rviz, use_autonomy)),
            ),
        ]
    )


def _not(sub):
    from launch.substitutions import NotSubstitution
    return NotSubstitution(sub)


def _and(a, b):
    from launch.substitutions import AndSubstitution
    return AndSubstitution(a, b)
