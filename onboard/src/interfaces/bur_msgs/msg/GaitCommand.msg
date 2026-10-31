# Gait parameters: the output of the outer control loop and the input to the
# gait generator. This message replaces the old wrench -> thruster allocation
# path entirely.
#
# The traveling-wave form these parameterise, for joint i of n:
#
#   phase   += 2*pi*frequency_hz*dt          (integrated by the receiver)
#   theta_i  = amplitude * envelope(i/n)
#                        * sin(phase - 2*pi*i/wavelength)
#              + turn_bias
#
# The phase is integrated rather than computed from absolute time, so a change
# of frequency_hz bends the wave smoothly instead of jumping it mid-cycle.
#
# TODO(fish): `envelope` is absent from this message because it is a property of
# the body, not of the command, so it lives in bur_gait's config. Confirm that
# split once there is a real tail to tune.

std_msgs/Header header

uint8 MODE_ESTOP=0       # all joints to neutral, ignore every other field
uint8 MODE_COAST=1       # stop driving, let the body go limp
uint8 MODE_HOLD=2        # hold station: zero net thrust, keep attitude authority
uint8 MODE_CRUISE=3      # steady swimming
uint8 MODE_TURN=4        # swimming with a sustained turn bias
uint8 mode

float32 frequency_hz     # tail-beat rate.
                         # TODO(hw): the upper bound is set by servo bandwidth
                         # under hydrodynamic load, not by the datasheet.
float32 amplitude        # 0..1, scales the envelope. Unitless on purpose.
float32 wavelength       # body lengths per full wave. Carangiform swimmers sit
                         # near 1.0.
                         # TODO(hw): tune in a tank, not on the bench.
float32 turn_bias        # rad. DC offset added to every joint, biasing the
                         # whole body to one side to turn. +ve = to port (NWU).
float32 fin_angle_rad    # rad. Dive-plane deflection. NOT a body bend.
                         #
                         # The chain is Z-only (4 joints, D-1) and cannot pitch,
                         # so depth and pitch authority come from fins. This is
                         # the commanded fin deflection, which the MCU routes to
                         # the fin servo(s) - it is deliberately NOT applied to
                         # any chain joint.
                         #
                         # TODO(hw): fin lift scales as v^2, so the usable range
                         # of this field depends on forward speed. At low speed
                         # a large deflection buys almost nothing. See D-3.
