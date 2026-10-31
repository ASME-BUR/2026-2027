#ifndef BUR_GAIT_HPP
#define BUR_GAIT_HPP

// Traveling-wave gait generator.
//
// This package REPLACES bur_thruster_manager. Nothing carried over except the
// clamp and rate-limit ideas: a static pseudo-inverse force allocation matrix
// has no meaning for a serial link chain, because the chain does not produce a
// wrench at a point, it produces thrust by shedding vortices over a beat
// period.
//
// Kept deliberately free of ROS so it can be unit tested on the host and, more
// importantly, so the SAME function can be compiled into the MCU firmware. The
// Pi-side node is for bring-up and RViz; on the vehicle the MCU owns the phase.

#include <cmath>
#include <cstddef>
#include <vector>

namespace bur_gait
{

// M_PI is a POSIX extension, not standard C++, so a strict toolchain (possibly
// the Due's) does not define it. Keep this header self-sufficient.
constexpr double kPi = 3.14159265358979323846;

struct GaitParams
{
    double frequency_hz = 0.0;
    double amplitude = 0.0;      // 0..1
    double wavelength = 1.0;     // body lengths per wave
    double turn_bias = 0.0;      // rad
    //
    // NOTE: GaitCommand.fin_angle_rad is deliberately absent here. Fins are a
    // separate actuator from the chain, not part of the gait, so the wave
    // function has no business knowing about them. The MCU reads that field
    // straight off the message and drives the fin servo with it.
};

struct ChainConfig
{
    std::size_t n_joints = 4;   // matches gait.yaml and the URDF

    // Amplitude envelope along the body, head (0) to tail (1). Real swimmers
    // are close to zero at the head and maximal at the tail.
    //
    // TODO(hw): this cubic is a placeholder taken from the carangiform
    // literature, not from your hull. Measure the real thing: drive a single
    // joint at a time and see what actually moves the vehicle.
    double envelope_exponent = 2.0;

    double joint_limit_rad = 0.6;   // TODO(hw): from the servo and the hull
    double max_rate_rad_s = 4.4;    // TODO(hw): measured servo slew under load
};

// Amplitude scaling at normalised body position s in [0, 1].
inline double envelope(double s, const ChainConfig & cfg)
{
    if (s < 0.0) { s = 0.0; }
    if (s > 1.0) { s = 1.0; }
    return std::pow(s, cfg.envelope_exponent);
}

// Advance the wave phase by one tick and wrap it to [0, 2*pi).
//
// The phase is integrated, never computed as 2*pi*f*t. With an absolute time,
// any change of f jumps the wave by delta_f * t cycles (at t = 105 s,
// 1.0 -> 1.1 Hz is 10.5 cycles: the tail flips to the other side in one tick),
// and the controller changes f every tick. Integrating keeps the wave
// continuous through a frequency change. Wrapping keeps full precision however
// long the vehicle runs.
//
// The caller owns the phase: on the vehicle that is the MCU, here it is the
// bench node.
inline double advancePhase(double phase, double frequency_hz, double dt)
{
    const double two_pi = 2.0 * kPi;
    phase = std::fmod(phase + two_pi * frequency_hz * dt, two_pi);
    if (phase < 0.0) { phase += two_pi; }
    return phase;
}

// Joint angles at wave phase `phase` (from advancePhase) for the given
// parameters.
//
//   theta_i = A * env(i/n) * sin(phase - 2*pi*i/lambda) + bias
//
// Only the wave and the turn bias appear here. There is no pitch term: the
// chain articulates about Z only (D-1), so it cannot produce a pitching moment,
// and adding a uniform pitch offset to every joint - as an earlier draft did -
// achieved nothing except stealing joint range from the wave and saturating the
// tail. Pitch and depth authority come from fins instead (D-3), commanded
// separately via GaitCommand.fin_angle_rad.
inline std::vector<double> jointAngles(double phase, const GaitParams & p,
                                       const ChainConfig & cfg)
{
    std::vector<double> out(cfg.n_joints, 0.0);
    if (cfg.n_joints == 0) { return out; }

    const double two_pi = 2.0 * kPi;
    // Guard against a zero or negative wavelength in a malformed command.
    const double lambda = (p.wavelength > 1e-3) ? p.wavelength : 1.0;
    const double denom =
        (cfg.n_joints > 1) ? static_cast<double>(cfg.n_joints - 1) : 1.0;

    for (std::size_t i = 0; i < cfg.n_joints; ++i) {
        const double s = static_cast<double>(i) / denom;
        const double joint_phase = phase - two_pi * s / lambda;
        double theta = p.amplitude * envelope(s, cfg) * std::sin(joint_phase);
        theta += p.turn_bias;

        if (theta > cfg.joint_limit_rad) { theta = cfg.joint_limit_rad; }
        if (theta < -cfg.joint_limit_rad) { theta = -cfg.joint_limit_rad; }
        out[i] = theta;
    }
    return out;
}

// Slew limit, carried over in spirit from thruster_manager's rate limiter.
// Without this a mode change snaps every joint at once and browns out the
// servo rail, which is the servo-chain equivalent of the thruster inrush
// problem the old rate limiter existed to solve.
inline void rateLimit(std::vector<double> & target,
                      const std::vector<double> & last,
                      double dt, const ChainConfig & cfg)
{
    const double max_step = cfg.max_rate_rad_s * dt;
    for (std::size_t i = 0; i < target.size() && i < last.size(); ++i) {
        const double d = target[i] - last[i];
        if (d > max_step) { target[i] = last[i] + max_step; }
        if (d < -max_step) { target[i] = last[i] - max_step; }
    }
}

}  // namespace bur_gait

#endif
