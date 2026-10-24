# High-level motion intent. What the behavior tree and the teleop node emit.
#
# Deliberately has NO sway term. A fish cannot strafe, and leaving the field out
# is the cheapest way to stop the thruster-era assumption leaking back in
# through a planner or a behavior node.
std_msgs/Header header

float32 surge_mps        # desired forward speed, m/s. Negative = reverse.
                         # TODO(hw): achievable range is unknown until the body
                         # is in water. Clamp in the controller, not here.
float32 yaw_rate_rps     # desired turn rate, rad/s. +ve = to port (NWU).
float32 depth_m          # desired depth, metres. Negative-down in NWU, to
                         # match the Bar30 sign convention carried over from
                         # bur-onboard.
float32 pitch_rad        # desired body pitch.

bool enable              # false => coast. The gait generator must fall to
                         # neutral, NOT freeze the last joint command.
