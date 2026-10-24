# Carried over from ASME-BUR/bur-interfaces@main.
#
# The aggregate state+setpoint message the controller consumes. Built by the
# teleop/aggregator node, not by the behavior tree.
#
# TODO(fish): target_vel.twist.linear.y (sway) is meaningless on an undulating
# body, which cannot translate sideways. Decide whether to keep the full 6-DOF
# Twist here for compatibility and simply ignore sway in the controller, or
# narrow this message. Prefer narrowing: a field that exists but is ignored
# WILL get set by someone eventually. See SwimCommand.msg.
#
# TODO(port): `auxilary` is misspelled (should be `auxiliary`) and nothing in
# bur-onboard documented what the slots mean. Either define them here or drop
# the field.
std_msgs/Header header
geometry_msgs/TwistStamped target_vel
geometry_msgs/TwistStamped current_vel
geometry_msgs/PoseStamped current_pos
geometry_msgs/PoseStamped target_pos
float32[] auxilary
bool[] buttons
