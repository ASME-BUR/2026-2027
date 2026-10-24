# Telemetry back from the servo chain. Nothing like this existed last year: the
# thrusters were open-loop and the Arduino never reported anything.
#
# Real joint feedback is the single biggest debuggability win available on the
# new vehicle. It tells you whether a bad gait is a command problem or a servo
# that is stalling, browning out, or cooking.
std_msgs/Header header

float32[] position_rad   # measured, head to tail. Empty if the servos do not
                         # report position.
                         # TODO(hw): do the chosen servos have position feedback
                         # at all? Smart serial servos do; hobby PWM servos do
                         # not. This drives the whole telemetry story.
float32[] current_a      # per-joint current draw. Watch this for stall.
float32[] temperature_c
uint16 fault_flags       # TODO(decide): define the bits once the servo and
                         # driver are chosen. Reserve 0 for "no fault".
uint8 sequence           # echoes the last GaitCommand/ServoChainCommand seen
