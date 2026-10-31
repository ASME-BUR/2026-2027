// Unit tests for the ROS-free wave maths in gait.hpp (DEVELOPMENT.md C-7).
//
// This is the same code the MCU firmware compiles, so a regression here is a
// regression on the vehicle, not just in the RViz preview.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "bur_gait/gait.hpp"

namespace
{

constexpr double kTwoPi = 2.0 * bur_gait::kPi;
constexpr double kEps = 1e-9;

bur_gait::ChainConfig fourJoints()
{
    bur_gait::ChainConfig cfg;
    cfg.n_joints = 4;
    cfg.envelope_exponent = 2.0;
    cfg.joint_limit_rad = 0.6;
    cfg.max_rate_rad_s = 4.4;
    return cfg;
}

bur_gait::GaitParams swimming()
{
    bur_gait::GaitParams p;
    p.frequency_hz = 1.0;
    p.amplitude = 0.5;
    p.wavelength = 1.0;
    p.turn_bias = 0.0;
    return p;
}

}  // namespace

// ---- envelope ----

TEST(Envelope, ZeroAtHeadOneAtTail)
{
    const auto cfg = fourJoints();
    EXPECT_NEAR(bur_gait::envelope(0.0, cfg), 0.0, kEps);
    EXPECT_NEAR(bur_gait::envelope(1.0, cfg), 1.0, kEps);
}

TEST(Envelope, MonotonicHeadToTail)
{
    const auto cfg = fourJoints();
    double prev = bur_gait::envelope(0.0, cfg);
    for (int k = 1; k <= 100; ++k) {
        const double e = bur_gait::envelope(k / 100.0, cfg);
        EXPECT_GE(e, prev) << "envelope decreased at s = " << k / 100.0;
        prev = e;
    }
}

TEST(Envelope, ClampsOutsideBody)
{
    const auto cfg = fourJoints();
    EXPECT_NEAR(bur_gait::envelope(-0.5, cfg), bur_gait::envelope(0.0, cfg), kEps);
    EXPECT_NEAR(bur_gait::envelope(1.5, cfg), bur_gait::envelope(1.0, cfg), kEps);
}

// ---- jointAngles ----

TEST(JointAngles, OneAnglePerJoint)
{
    auto cfg = fourJoints();
    EXPECT_EQ(bur_gait::jointAngles(0.3, swimming(), cfg).size(), 4u);

    cfg.n_joints = 0;
    EXPECT_TRUE(bur_gait::jointAngles(0.3, swimming(), cfg).empty());
}

TEST(JointAngles, SingleJointIsFinite)
{
    // n = 1 is the edge of the i/(n-1) normalisation; it must not divide by 0.
    auto cfg = fourJoints();
    cfg.n_joints = 1;
    const auto a = bur_gait::jointAngles(1.0, swimming(), cfg);
    ASSERT_EQ(a.size(), 1u);
    EXPECT_TRUE(std::isfinite(a[0]));
}

TEST(JointAngles, WaveTravelsHeadToTail)
{
    // With a flat envelope every joint traces the same sine, so the tail-ward
    // neighbour must reach joint i's angle exactly 2*pi*ds/lambda of phase
    // later. If the sign of the phase lag flips, the wave runs tail to head and
    // the vehicle swims backwards.
    auto cfg = fourJoints();
    cfg.envelope_exponent = 0.0;
    cfg.joint_limit_rad = 10.0;
    auto p = swimming();
    p.wavelength = 1.3;

    const double ds = 1.0 / static_cast<double>(cfg.n_joints - 1);
    const double lag = kTwoPi * ds / p.wavelength;

    for (double phase = 0.0; phase < kTwoPi; phase += 0.37) {
        const auto now = bur_gait::jointAngles(phase, p, cfg);
        const auto later = bur_gait::jointAngles(phase + lag, p, cfg);
        for (std::size_t i = 0; i + 1 < cfg.n_joints; ++i) {
            EXPECT_NEAR(later[i + 1], now[i], 1e-9)
                << "joint " << i + 1 << " does not lag joint " << i;
        }
    }
}

TEST(JointAngles, TailSwingsWiderThanHead)
{
    auto cfg = fourJoints();
    cfg.joint_limit_rad = 10.0;
    const auto p = swimming();

    std::vector<double> peak(cfg.n_joints, 0.0);
    for (int k = 0; k < 360; ++k) {
        const auto a = bur_gait::jointAngles(kTwoPi * k / 360.0, p, cfg);
        for (std::size_t i = 0; i < a.size(); ++i) {
            peak[i] = std::fmax(peak[i], std::fabs(a[i]));
        }
    }
    for (std::size_t i = 0; i + 1 < peak.size(); ++i) {
        EXPECT_LE(peak[i], peak[i + 1] + kEps);
    }
    EXPECT_NEAR(peak.back(), p.amplitude, 1e-3);
}

TEST(JointAngles, ZeroAmplitudeIsPureTurnBias)
{
    const auto cfg = fourJoints();
    auto p = swimming();
    p.amplitude = 0.0;

    p.turn_bias = 0.0;
    for (double a : bur_gait::jointAngles(1.234, p, cfg)) {
        EXPECT_NEAR(a, 0.0, kEps);
    }

    p.turn_bias = 0.2;   // +ve = to port
    for (double a : bur_gait::jointAngles(1.234, p, cfg)) {
        EXPECT_NEAR(a, 0.2, kEps);
    }
}

TEST(JointAngles, ClampedToJointLimit)
{
    const auto cfg = fourJoints();
    auto p = swimming();
    p.amplitude = 1.0;

    for (double bias : {-1.0, -0.3, 0.0, 0.3, 1.0}) {
        p.turn_bias = bias;
        for (double phase = 0.0; phase < kTwoPi; phase += 0.1) {
            for (double a : bur_gait::jointAngles(phase, p, cfg)) {
                EXPECT_LE(a, cfg.joint_limit_rad + kEps);
                EXPECT_GE(a, -cfg.joint_limit_rad - kEps);
            }
        }
    }

    // A turn bias past the limit saturates rather than wrapping or overshooting.
    p.amplitude = 0.0;
    p.turn_bias = 5.0;
    for (double a : bur_gait::jointAngles(0.0, p, cfg)) {
        EXPECT_NEAR(a, cfg.joint_limit_rad, kEps);
    }
}

TEST(JointAngles, BadWavelengthFallsBackToOne)
{
    const auto cfg = fourJoints();
    auto p = swimming();
    p.wavelength = 1.0;
    const auto ref = bur_gait::jointAngles(0.7, p, cfg);

    for (double bad : {0.0, -2.0, 1e-6}) {
        p.wavelength = bad;
        const auto a = bur_gait::jointAngles(0.7, p, cfg);
        for (std::size_t i = 0; i < a.size(); ++i) {
            EXPECT_TRUE(std::isfinite(a[i]));
            EXPECT_NEAR(a[i], ref[i], kEps) << "wavelength = " << bad;
        }
    }
}

// ---- advancePhase ----

TEST(AdvancePhase, StaysWrapped)
{
    double phase = 0.0;
    for (double f : {0.5, 2.0, 37.0, -1.5}) {
        for (int k = 0; k < 1000; ++k) {
            phase = bur_gait::advancePhase(phase, f, 0.02);
            ASSERT_GE(phase, 0.0);
            ASSERT_LT(phase, kTwoPi);
        }
    }
    // One large step must wrap too.
    phase = bur_gait::advancePhase(0.0, 1.0, 1e6 + 0.25);
    EXPECT_GE(phase, 0.0);
    EXPECT_LT(phase, kTwoPi);
}

TEST(AdvancePhase, AdvancesByTwoPiFDt)
{
    EXPECT_NEAR(bur_gait::advancePhase(0.0, 1.0, 0.25), bur_gait::kPi / 2.0,
                kEps);
    EXPECT_NEAR(bur_gait::advancePhase(1.0, 0.0, 0.02), 1.0, kEps);
    // One full cycle lands back on the start, up to the wrap.
    const double back = bur_gait::advancePhase(1.0, 2.0, 0.5);
    EXPECT_NEAR(std::sin(back), std::sin(1.0), 1e-9);
    EXPECT_NEAR(std::cos(back), std::cos(1.0), 1e-9);
}

TEST(AdvancePhase, FrequencyStepDoesNotJumpTheWave)
{
    // The regression this function exists for. Computing the wave from
    // absolute time jumped the tail 0.75 rad in one tick when f stepped from
    // 1.0 to 1.1 Hz at t = 105.13 s. Integrated phase must never move a joint
    // further in one tick than a smooth wave at the higher frequency can.
    auto cfg = fourJoints();
    cfg.joint_limit_rad = 10.0;   // measure the wave, not the clamp
    auto p = swimming();
    const double dt = 0.02;   // 50 Hz servo frame
    const double f_hi = 1.1;
    const double smooth_bound = kTwoPi * f_hi * dt * p.amplitude;

    double phase = 0.0;
    std::vector<double> prev;
    for (int k = 0; k < 10000; ++k) {
        const double t = k * dt;
        p.frequency_hz = (t < 105.13) ? 1.0 : f_hi;
        phase = bur_gait::advancePhase(phase, p.frequency_hz, dt);
        const auto a = bur_gait::jointAngles(phase, p, cfg);
        if (!prev.empty()) {
            for (std::size_t i = 0; i < a.size(); ++i) {
                ASSERT_LE(std::fabs(a[i] - prev[i]), smooth_bound + 1e-9)
                    << "joint " << i << " jumped at t = " << t;
            }
        }
        prev = a;
    }
}

// ---- rateLimit ----

TEST(RateLimit, CapsStepAtMaxRate)
{
    const auto cfg = fourJoints();
    const double dt = 0.02;
    const double max_step = cfg.max_rate_rad_s * dt;

    std::vector<double> last(4, 0.0);
    std::vector<double> target = {0.5, -0.5, 0.01, 0.0};
    bur_gait::rateLimit(target, last, dt, cfg);

    EXPECT_NEAR(target[0], max_step, kEps);
    EXPECT_NEAR(target[1], -max_step, kEps);
    EXPECT_NEAR(target[2], 0.01, kEps);   // already within the step
    EXPECT_NEAR(target[3], 0.0, kEps);
}

TEST(RateLimit, MismatchedSizesAreSafe)
{
    const auto cfg = fourJoints();
    std::vector<double> last(2, 0.0);
    std::vector<double> target = {1.0, 1.0, 1.0, 1.0};
    bur_gait::rateLimit(target, last, 0.02, cfg);

    const double max_step = cfg.max_rate_rad_s * 0.02;
    EXPECT_NEAR(target[0], max_step, kEps);
    EXPECT_NEAR(target[1], max_step, kEps);
    EXPECT_NEAR(target[2], 1.0, kEps);   // no history, left alone
    EXPECT_NEAR(target[3], 1.0, kEps);
}
