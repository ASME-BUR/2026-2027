from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # bur-onboard ran TWO agents, one per Arduino: ttyACM0 for thrusters and
    # ttyACM1 for the IMU, both at 250000 baud.
    #
    # TODO(decide): is the new vehicle still two MCUs, or one? One board that
    # owns both the servo chain and the IMU is simpler and removes a whole
    # agent, but couples the gait loop to IMU read timing. Decide before
    # writing firmware.
    #
    # TODO(hw): /dev/ttyACM* ordering is not stable across reboots. Add a udev
    # rule keyed on the USB serial number and use /dev/bur_servo and
    # /dev/bur_imu instead. This WILL bite you on competition day otherwise.

    servo_dev = LaunchConfiguration("servo_dev")
    imu_dev = LaunchConfiguration("imu_dev")
    baud = LaunchConfiguration("baud")

    return LaunchDescription(
        [
            DeclareLaunchArgument("servo_dev", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("imu_dev", default_value="/dev/ttyACM1"),
            DeclareLaunchArgument("baud", default_value="250000"),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent_servo",
                output="screen",
                arguments=["serial", "-D", servo_dev, "-b", baud, "-v", "4"],
            ),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent_imu",
                output="screen",
                arguments=["serial", "-D", imu_dev, "-b", baud, "-v", "4"],
            ),
        ]
    )
