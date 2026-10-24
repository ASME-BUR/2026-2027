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
    std::size_t n_joints = 6;

    // Amplitude envelope along the body, head (0) to tail (1). Real swimmers
    // are close to zero at the head and maximal at the tail.
    //
    // TODO(hw): this cubic is a placeholder taken from the carangiform
    // literature, not from your hull. Measure the real thing: drive a single
    // joint at a time and see what actually moves the vehicle.
    double envelope_exponent = 2.0;

    double joint_limit_rad = 0.6;   // TODO(hw): from the servo and the hull
    double max_rate_rad_s = 6.0;    // TODO(hw): measured servo slew under load
};

// Amplitude scaling at normalised body position s in [0, 1].
inline double envelope(double s, const ChainConfig & cfg)
{
    if (s < 0.0) { s = 0.0; }
    if (s > 1.0) { s = 1.0; }
    return std::pow(s, cfg.envelope_exponent);
}

// Joint angles at time t for the given parameters.
//
//   theta_i(t) = A * env(i/n) * sin(2*pi*f*t - 2*pi*i/lambda) + bias
//
// Only the wave and the turn bias appear here. There is no pitch term: the
// chain articulates about Z only (D-1), so it cannot produce a pitching moment,
// and adding a uniform pitch offset to every joint - as an earlier draft did -
// achieved nothing except stealing joint range from the wave and saturating the
// tail. Pitch and depth authority come from fins instead (D-3), commanded
// separately via GaitCommand.fin_angle_rad.
inline std::vector<double> jointAngles(double t, const GaitParams & p,
                                       const ChainConfig & cfg)
{
    std::vector<double> out(cfg.n_joints, 0.0);
    if (cfg.n_joints == 0) { return out; }

    const double two_pi = 2.0 * M_PI;
    // Guard against a zero or negative wavelength in a malformed command.
    const double lambda = (p.wavelength > 1e-3) ? p.wavelength : 1.0;
    const double denom =
        (cfg.n_joints > 1) ? static_cast<double>(cfg.n_joints - 1) : 1.0;

    for (std::size_t i = 0; i < cfg.n_joints; ++i) {
        const double s = static_cast<double>(i) / denom;
        const double phase = two_pi * p.frequency_hz * t - two_pi * s / lambda;
        double theta = p.amplitude * envelope(s, cfg) * std::sin(phase);
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
