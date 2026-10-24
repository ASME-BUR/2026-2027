# Replaces ThrusterCommand.msg.
#
# Per-joint angle targets for the servo chain. Used only when the gait phase is
# generated on the Pi, for bring-up and debugging. In normal operation the MCU
# owns the phase accumulator and subscribes to GaitCommand instead, because
# serial jitter on a 50 Hz topic shows up as visible wobble at the tail.
#
# TODO(decide): confirm the MCU-owns-phase split before writing firmware. If it
# changes, this message becomes the primary actuation path and needs a hard
# real-time story.
std_msgs/Header header

float32[] position_rad   # one entry per joint, head to tail.
uint8 sequence           # wraps at 255. Lets the MCU notice a dropped command
                         # and coast, rather than holding a stale angle.
