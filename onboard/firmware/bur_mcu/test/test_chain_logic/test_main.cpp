// Host tests for include/chain_logic.hpp: `pio test -e native`.
//
// These cover the parts of F-1 and F-3 that can be checked without a Due:
// watchdog timing, mode handling, malformed commands, slew limiting, and the
// angle -> pulse-width map.

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "chain_logic.hpp"

using bur_mcu::ChainLimits;
using bur_mcu::ChainLogic;
using bur_mcu::GaitCommand;

namespace
{

constexpr std::uint32_t kTick = 20000;   // us

GaitCommand cruise(double f = 1.0, double a = 0.5, double bias = 0.0)
{
    GaitCommand c;
    c.mode = bur_mcu::MODE_CRUISE;
    c.params.frequency_hz = f;
    c.params.amplitude = a;
    c.params.wavelength = 1.0;
    c.params.turn_bias = bias;
    return c;
}

double maxAbs(const std::vector<double> & v)
{
    double m = 0.0;
    for (double x : v) { m = std::fmax(m, std::fabs(x)); }
    return m;
}

// Tick from t0 for n ticks while re-sending `c` every `every` ticks (0 = never).
std::uint32_t run(ChainLogic & chain, std::uint32_t t0, int n, const GaitCommand * c,
                  int every)
{
    std::uint32_t t = t0;
    for (int i = 0; i < n; ++i) {
        if (c && every > 0 && i % every == 0) { chain.onCommand(*c, t); }
        chain.tick(t);
        t += kTick;
    }
    return t;
}

}  // namespace

void setUp() {}
void tearDown() {}

// Before any command the chain sits straight.
void test_idle_is_straight()
{
    ChainLogic chain(bur_gait::ChainConfig{}, ChainLimits{});
    run(chain, 0, 50, nullptr, 0);
    TEST_ASSERT_TRUE(chain.stale(50 * kTick));
    TEST_ASSERT_EQUAL_DOUBLE(0.0, maxAbs(chain.angles()));
}

// A command refreshed at 10 Hz keeps the wave going.
void test_cruise_moves_joints()
{
    ChainLogic chain(bur_gait::ChainConfig{}, ChainLimits{});
    const GaitCommand c = cruise();
    run(chain, 0, 100, &c, 5);
    TEST_ASSERT_FALSE(chain.stale(99 * kTick));
    TEST_ASSERT_TRUE(maxAbs(chain.angles()) > 0.05);
}

// F-3: after the last command, the chain is still driven until 300 ms, then
// straightens, and must end exactly at zero rather than hold an angle.
void test_watchdog_coasts_to_zero()
{
    ChainLogic chain(bur_gait::ChainConfig{}, ChainLimits{});
    const GaitCommand c = cruise(1.0, 1.0, 0.3);   // strong turn: a held angle bends the body
    std::uint32_t t = run(chain, 0, 100, &c, 5);
    const std::uint32_t last_cmd = 95 * kTick;

    TEST_ASSERT_FALSE(chain.stale(last_cmd + 290000));
    TEST_ASSERT_TRUE(chain.stale(last_cmd + 310000));

    t = run(chain, t, 100, nullptr, 0);   // 2 s of silence
    TEST_ASSERT_EQUAL_DOUBLE(0.0, maxAbs(chain.angles()));
}

// The timeout must work across a micros() wrap (every ~71.6 minutes).
void test_watchdog_survives_micros_wrap()
{
    ChainLogic chain(bur_gait::ChainConfig{}, ChainLimits{});
    const std::uint32_t t0 = 0xFFFFFFFFu - 100000u;   // 100 ms before wrap
    chain.onCommand(cruise(), t0);
    TEST_ASSERT_FALSE(chain.stale(t0 + 200000u));     // wrapped, 200 ms later
    TEST_ASSERT_TRUE(chain.stale(t0 + 400000u));      // wrapped, 400 ms later
}

void test_estop_and_coast_straighten()
{
    for (std::uint8_t mode : {std::uint8_t(bur_mcu::MODE_ESTOP),
                              std::uint8_t(bur_mcu::MODE_COAST)}) {
        ChainLogic chain(bur_gait::ChainConfig{}, ChainLimits{});
        const GaitCommand c = cruise(1.0, 1.0, 0.3);
        std::uint32_t t = run(chain, 0, 100, &c, 5);
        GaitCommand stop = c;
        stop.mode = mode;
        t = run(chain, t, 50, &stop, 5);
        TEST_ASSERT_FALSE(chain.stale(t - kTick));
        TEST_ASSERT_EQUAL_DOUBLE(0.0, maxAbs(chain.angles()));
    }
}

// An unknown mode (a newer message definition, or a corrupted value) coasts.
void test_unknown_mode_coasts()
{
    ChainLogic chain(bur_gait::ChainConfig{}, ChainLimits{});
    GaitCommand c = cruise(1.0, 1.0, 0.3);
    c.mode = 42;
    run(chain, 0, 100, &c, 5);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, maxAbs(chain.angles()));
}

// A NaN anywhere in the command must not reach the servos.
void test_nan_command_coasts()
{
    ChainLogic chain(bur_gait::ChainConfig{}, ChainLimits{});
    const GaitCommand good = cruise(1.0, 1.0, 0.3);
    std::uint32_t t = run(chain, 0, 50, &good, 5);
    GaitCommand bad = good;
    bad.params.frequency_hz = std::numeric_limits<double>::quiet_NaN();
    run(chain, t, 100, &bad, 5);
    TEST_ASSERT_EQUAL_UINT32(20, chain.rejected());
    for (double a : chain.angles()) { TEST_ASSERT_TRUE(std::isfinite(a)); }
    TEST_ASSERT_EQUAL_DOUBLE(0.0, maxAbs(chain.angles()));
}

// No joint ever moves faster than max_rate_rad_s, including on mode changes.
void test_slew_limit_holds()
{
    bur_gait::ChainConfig cfg;
    ChainLogic chain(cfg, ChainLimits{});
    const double max_step = cfg.max_rate_rad_s * (kTick * 1e-6) + 1e-12;
    std::vector<double> prev(cfg.n_joints, 0.0);
    std::uint32_t t = 0;
    GaitCommand c = cruise(3.0, 1.0, 0.0);
    for (int i = 0; i < 300; ++i) {
        if (i == 100) { c.params.turn_bias = 0.6; }      // sudden hard turn
        if (i == 200) { c.mode = bur_mcu::MODE_ESTOP; }  // sudden stop
        if (i % 5 == 0) { chain.onCommand(c, t); }
        const std::vector<double> & a = chain.tick(t);
        for (std::size_t j = 0; j < a.size(); ++j) {
            TEST_ASSERT_TRUE(std::fabs(a[j] - prev[j]) <= max_step);
        }
        prev = a;
        t += kTick;
    }
}

// A stalled loop (say 500 ms in a blocking call) must not produce one huge step.
void test_stall_does_not_jump()
{
    bur_gait::ChainConfig cfg;
    ChainLimits lim;
    ChainLogic chain(cfg, lim);
    GaitCommand c = cruise(1.0, 0.2, 0.0);
    std::uint32_t t = run(chain, 0, 50, &c, 5);
    const std::vector<double> before = chain.angles();
    c.params.turn_bias = 0.6;
    chain.onCommand(c, t + 480000);
    chain.tick(t + 500000);
    const double max_step = cfg.max_rate_rad_s * lim.max_dt_s + 1e-12;
    for (std::size_t j = 0; j < before.size(); ++j) {
        TEST_ASSERT_TRUE(std::fabs(chain.angles()[j] - before[j]) <= max_step);
    }
}

// Joint limit from ChainConfig is respected.
void test_joint_limit()
{
    bur_gait::ChainConfig cfg;
    ChainLogic chain(cfg, ChainLimits{});
    const GaitCommand c = cruise(2.0, 1.0, 2.0);   // bias far past the limit
    run(chain, 0, 200, &c, 5);
    TEST_ASSERT_TRUE(maxAbs(chain.angles()) <= cfg.joint_limit_rad + 1e-12);
}

// Frequency and amplitude are clamped to sane ranges.
void test_command_clamping()
{
    ChainLimits lim;
    ChainLogic chain(bur_gait::ChainConfig{}, lim);
    chain.onCommand(cruise(50.0, 7.0), 0);
    TEST_ASSERT_EQUAL_DOUBLE(lim.max_frequency_hz, chain.command().params.frequency_hz);
    TEST_ASSERT_EQUAL_DOUBLE(1.0, chain.command().params.amplitude);
    chain.onCommand(cruise(-1.0, -1.0), 0);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, chain.command().params.frequency_hz);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, chain.command().params.amplitude);
}

// The phase keeps going where it left off after a coast (as gait_node does).
void test_phase_holds_through_coast()
{
    ChainLogic chain(bur_gait::ChainConfig{}, ChainLimits{});
    const GaitCommand c = cruise(1.0, 0.5);
    std::uint32_t t = run(chain, 0, 37, &c, 5);
    t = run(chain, t, 30, nullptr, 0);   // past the 300 ms timeout
    TEST_ASSERT_TRUE(chain.stale(t));
    const double p = chain.phase();
    TEST_ASSERT_TRUE(p > 0.0);
    t = run(chain, t, 50, nullptr, 0);
    TEST_ASSERT_EQUAL_DOUBLE(p, chain.phase());
}

void test_angle_to_micros()
{
    const bur_mcu::ServoMap m;
    TEST_ASSERT_EQUAL_INT(1500, bur_mcu::angleToMicros(0.0, +1, 0, m));
    TEST_ASSERT_EQUAL_INT(1510, bur_mcu::angleToMicros(0.0, +1, 10, m));
    // 2000 us per 2*pi rad: 90 deg = 500 us.
    TEST_ASSERT_EQUAL_INT(2000, bur_mcu::angleToMicros(bur_gait::kPi / 2, +1, 0, m));
    TEST_ASSERT_EQUAL_INT(1000, bur_mcu::angleToMicros(bur_gait::kPi / 2, -1, 0, m));
    TEST_ASSERT_EQUAL_INT(2500, bur_mcu::angleToMicros(10.0, +1, 0, m));
    TEST_ASSERT_EQUAL_INT(500, bur_mcu::angleToMicros(-10.0, +1, 0, m));
    TEST_ASSERT_EQUAL_INT(1500, bur_mcu::angleToMicros(std::nan(""), +1, 0, m));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_idle_is_straight);
    RUN_TEST(test_cruise_moves_joints);
    RUN_TEST(test_watchdog_coasts_to_zero);
    RUN_TEST(test_watchdog_survives_micros_wrap);
    RUN_TEST(test_estop_and_coast_straighten);
    RUN_TEST(test_unknown_mode_coasts);
    RUN_TEST(test_nan_command_coasts);
    RUN_TEST(test_slew_limit_holds);
    RUN_TEST(test_stall_does_not_jump);
    RUN_TEST(test_joint_limit);
    RUN_TEST(test_command_clamping);
    RUN_TEST(test_phase_holds_through_coast);
    RUN_TEST(test_angle_to_micros);
    return UNITY_END();
}
