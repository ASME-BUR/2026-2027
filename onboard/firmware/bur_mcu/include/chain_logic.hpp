#ifndef BUR_MCU_CHAIN_LOGIC_HPP
#define BUR_MCU_CHAIN_LOGIC_HPP

// The servo chain's per-tick logic, with no Arduino and no micro-ROS in it, so
// the same code runs on the Due and in the host unit tests (test/).
//
// Implements F-1 (gait on the MCU) and F-3 (watchdog and failsafe). It does
// exactly what bur_gait's gait_node.cpp does on the Pi, so the RViz preview and
// the vehicle agree, with two deliberate additions that only matter on hardware:
//
//   - malformed commands (NaN/inf, unknown mode) are treated as COAST, and
//   - dt is clamped, so a stalled loop cannot turn into one huge slew step.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "bur_gait/gait.hpp"

namespace bur_mcu
{

// Values of bur_msgs/GaitCommand.mode. Kept here, not taken from the generated
// micro-ROS header, so this file stays ROS-free.
enum GaitMode : std::uint8_t
{
    MODE_ESTOP = 0,
    MODE_COAST = 1,
    MODE_HOLD = 2,
    MODE_CRUISE = 3,
    MODE_TURN = 4,
};

struct GaitCommand
{
    std::uint8_t mode = MODE_COAST;
    bur_gait::GaitParams params;
    double fin_angle_rad = 0.0;   // carried, not driven: fin routing waits on D-3
};

struct ChainLimits
{
    // F-3: coast to neutral after this long without a GaitCommand. Matches the
    // 0.3 s deadman in bur_gait.
    std::uint32_t command_timeout_us = 300000;

    // TODO(hw): M-2. The controller (controller.yaml freq_max) should already
    // keep frequency below what the servos can follow. This is only a backstop
    // against a malformed command.
    double max_frequency_hz = 3.0;

    // A stalled loop (blocking micro-ROS call, long I2C transaction) must not
    // become a single huge phase or slew step. Ticks are nominally 20 ms.
    double max_dt_s = 0.04;
};

// Classify a command before it is accepted. Anything not finite, or with an
// unknown mode, coasts.
inline bool isDrivingMode(std::uint8_t mode)
{
    return mode == MODE_HOLD || mode == MODE_CRUISE || mode == MODE_TURN;
}

inline bool isFinite(const GaitCommand & c)
{
    return std::isfinite(c.params.frequency_hz) && std::isfinite(c.params.amplitude) &&
           std::isfinite(c.params.wavelength) && std::isfinite(c.params.turn_bias) &&
           std::isfinite(c.fin_angle_rad);
}

class ChainLogic
{
public:
    ChainLogic(const bur_gait::ChainConfig & cfg, const ChainLimits & limits)
    : cfg_(cfg), limits_(limits), last_(cfg.n_joints, 0.0), target_(cfg.n_joints, 0.0)
    {
    }

    // Accept a GaitCommand that arrived at now_us (micros(), may wrap).
    void onCommand(const GaitCommand & c, std::uint32_t now_us)
    {
        GaitCommand sane = c;
        if (!isFinite(c)) {
            sane = GaitCommand{};   // COAST
            ++rejected_;
        }
        if (sane.params.frequency_hz < 0.0) { sane.params.frequency_hz = 0.0; }
        if (sane.params.frequency_hz > limits_.max_frequency_hz) {
            sane.params.frequency_hz = limits_.max_frequency_hz;
        }
        if (sane.params.amplitude < 0.0) { sane.params.amplitude = 0.0; }
        if (sane.params.amplitude > 1.0) { sane.params.amplitude = 1.0; }

        cmd_ = sane;
        last_cmd_us_ = now_us;
        have_cmd_ = true;
        ++received_;
    }

    // True if the last command is older than the timeout, or none has arrived.
    // Wrap-safe: unsigned subtraction of micros() values.
    bool stale(std::uint32_t now_us) const
    {
        return !have_cmd_ ||
               static_cast<std::uint32_t>(now_us - last_cmd_us_) > limits_.command_timeout_us;
    }

    // True if this tick drives the wave rather than straightening the body.
    bool driving(std::uint32_t now_us) const
    {
        return !stale(now_us) && isDrivingMode(cmd_.mode);
    }

    // Advance one tick and return the joint angles (rad, head to tail) to send.
    //
    // ESTOP, COAST, an unknown mode and a stale command all give a target of
    // zero: the body straightens, rate-limited so the servo rail does not brown
    // out. Never hold the last angle, because a held angle on a dead link bends
    // the body into a turn. The phase stops advancing, so swimming resumes where
    // it left off.
    const std::vector<double> & tick(std::uint32_t now_us)
    {
        double dt = 0.0;
        if (ticked_) {
            dt = static_cast<double>(static_cast<std::uint32_t>(now_us - last_tick_us_)) * 1e-6;
        }
        if (dt > limits_.max_dt_s) { dt = limits_.max_dt_s; }
        last_tick_us_ = now_us;
        ticked_ = true;

        if (driving(now_us)) {
            phase_ = bur_gait::advancePhase(phase_, cmd_.params.frequency_hz, dt);
            if (!std::isfinite(phase_)) { phase_ = 0.0; }
            target_ = bur_gait::jointAngles(phase_, cmd_.params, cfg_);
        } else {
            target_.assign(cfg_.n_joints, 0.0);
        }

        bur_gait::rateLimit(target_, last_, dt, cfg_);
        last_ = target_;
        return target_;
    }

    const GaitCommand & command() const { return cmd_; }
    const std::vector<double> & angles() const { return last_; }
    double phase() const { return phase_; }
    std::uint32_t received() const { return received_; }
    std::uint32_t rejected() const { return rejected_; }
    const bur_gait::ChainConfig & config() const { return cfg_; }

private:
    bur_gait::ChainConfig cfg_;
    ChainLimits limits_;

    GaitCommand cmd_;
    bool have_cmd_ = false;
    std::uint32_t last_cmd_us_ = 0;

    bool ticked_ = false;
    std::uint32_t last_tick_us_ = 0;
    double phase_ = 0.0;

    std::vector<double> last_;
    std::vector<double> target_;

    std::uint32_t received_ = 0;
    std::uint32_t rejected_ = 0;
};

// ---------------------------------------------------------------------------
// Servo output
// ---------------------------------------------------------------------------

struct ServoMap
{
    // DFRobot SER0067: 500-2500 us over 360 deg, so 2000 us per 2*pi rad.
    double us_per_rad = 2000.0 / (2.0 * bur_gait::kPi);
    int center_us = 1500;
    int min_us = 500;
    int max_us = 2500;
};

// Joint angle (rad, +ve to port) to pulse width. `direction` is +1 or -1
// depending on which way the servo is mounted; `trim_us` zeroes the joint.
inline int angleToMicros(double angle_rad, int direction, int trim_us, const ServoMap & m)
{
    if (!std::isfinite(angle_rad)) { angle_rad = 0.0; }
    const double us = static_cast<double>(m.center_us + trim_us) +
                      static_cast<double>(direction) * angle_rad * m.us_per_rad;
    long r = std::lround(us);
    if (r < m.min_us) { r = m.min_us; }
    if (r > m.max_us) { r = m.max_us; }
    return static_cast<int>(r);
}

}  // namespace bur_mcu

#endif
