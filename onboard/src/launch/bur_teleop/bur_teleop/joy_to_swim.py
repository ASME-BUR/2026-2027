#!/usr/bin/env python3
"""Map a gamepad to bur_msgs/SwimCommand.

Replaces bur-onboard's bur_rov/joy_command.py, which did two unrelated jobs in
one node: it read the joystick AND it aggregated odometry, depth and IMU into
the Command message the controller consumed. That meant the teleop node sat in
the autonomous path too, and the controller could not run at all without a
gamepad connected (it gated on button 9).

Here the two jobs are separate. bur_controller subscribes to /odometry/filtered
itself, so this node only translates sticks into intent and nothing downstream
depends on it existing.

TODO(hw): the axis map below is for a PS4 pad, carried over from
bur-onboard/config/ps4.yaml. Verify against whatever controller actually gets
used, with `ros2 topic echo /joy`.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from bur_msgs.msg import SwimCommand


class JoyToSwim(Node):
    def __init__(self):
        super().__init__("joy_to_swim")

        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("swim_topic", "/swim_command")
        self.declare_parameter("publish_rate", 20)

        # Axis / button map. Carried over from ps4.yaml where it applies.
        self.declare_parameter("axis.surge", 1)
        self.declare_parameter("axis.yaw_rate", 3)
        self.declare_parameter("axis.depth", 4)
        self.declare_parameter("button.deadman", 9)

        # Scaling. These are what the stick full-deflection means.
        # TODO(hw): max_surge is a guess until the vehicle has been timed over
        # a known distance. Keep it low for the first wet test.
        self.declare_parameter("max_surge_mps", 0.4)
        self.declare_parameter("max_yaw_rate_rps", 0.6)
        self.declare_parameter("depth_rate_mps", 0.2)
        self.declare_parameter("depth_min_m", -3.0)
        self.declare_parameter("depth_max_m", 0.0)

        self.swim_pub = self.create_publisher(
            SwimCommand, self.get_parameter("swim_topic").value, 10
        )
        self.joy_sub = self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self.joy_callback, 10
        )

        rate = self.get_parameter("publish_rate").value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.out = SwimCommand()
        self.depth_setpoint = 0.0
        self.have_joy = False
        self.last_tick = self.get_clock().now()

    def joy_callback(self, msg: Joy):
        self.latest = msg
        self.have_joy = True

    def _axis(self, msg: Joy, idx: int) -> float:
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return float(msg.axes[idx])

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now

        if not self.have_joy:
            return

        msg = self.latest
        deadman_idx = self.get_parameter("button.deadman").value
        held = (
            deadman_idx < len(msg.buttons) and bool(msg.buttons[deadman_idx])
        )

        # Depth is integrated from a stick rather than commanded directly, so
        # releasing the stick holds depth instead of surfacing.
        depth_axis = self._axis(msg, self.get_parameter("axis.depth").value)
        self.depth_setpoint += (
            depth_axis * self.get_parameter("depth_rate_mps").value * dt
        )
        self.depth_setpoint = max(
            self.get_parameter("depth_min_m").value,
            min(self.get_parameter("depth_max_m").value, self.depth_setpoint),
        )

        out = SwimCommand()
        out.header.stamp = now.to_msg()
        out.header.frame_id = "base_link"
        out.surge_mps = (
            -self._axis(msg, self.get_parameter("axis.surge").value)
            * self.get_parameter("max_surge_mps").value
        )
        out.yaw_rate_rps = (
            -self._axis(msg, self.get_parameter("axis.yaw_rate").value)
            * self.get_parameter("max_yaw_rate_rps").value
        )
        out.depth_m = float(self.depth_setpoint)
        out.pitch_rad = 0.0
        out.enable = held

        if not held:
            # Reset the depth setpoint to wherever we are so re-arming does
            # not snap to a stale target. bur-onboard's controller had exactly
            # this bug via its never-reset integrator.
            # TODO(port): read the real depth from /odometry/filtered to do
            # this properly. Zero is only right at the surface.
            self.depth_setpoint = 0.0

        self.swim_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToSwim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
