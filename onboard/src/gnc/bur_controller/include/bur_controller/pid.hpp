#ifndef BUR_PID_HPP
#define BUR_PID_HPP

// Carried over from bur-onboard/src/gnc/bur_controller/controller.hpp.
//
// The cascaded position->velocity structure and the runtime-retunable gains
// are the most reusable things in last year's stack, so they come across
// intact. Three fixes applied on the way over, all of them pre-existing bugs
// rather than anything to do with the distro:
//
//   1. `position_setpoint` was uninitialised and could be read on the first
//      callback. Now zero-initialised.
//   2. The integrator had no clamp and was never reset, so `rsum` accumulated
//      across disarms and came back as a kick on re-arm. Added an integral
//      limit and reset().
//   3. `abs()` was called unqualified on doubles. It resolves correctly on
//      libstdc++ but is not guaranteed to; now std::abs.

#include <algorithm>
#include <cmath>

#include "utility_func.hpp"

class PID
{
public:
    PID() {}

    void setGains(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    // Anti-windup. Set to 0 to disable the clamp.
    void setIntegralLimit(double limit) { i_limit_ = limit; }

    void reset()
    {
        past_e_ = 0.0;
        rsum_ = 0.0;
        primed_ = false;
    }

    double computeCommand(double error, double dt)
    {
        if (dt <= 0.0) { return 0.0; }

        // Suppress the derivative spike on the first sample after a reset,
        // which is what made re-arming jolt the old vehicle.
        double deriv = primed_ ? (error - past_e_) / dt : 0.0;
        primed_ = true;

        rsum_ += error * dt;
        if (i_limit_ > 0.0) {
            rsum_ = clamp_d(rsum_, -i_limit_, i_limit_);
        }

        past_e_ = error;
        return kp_ * error + kd_ * deriv + ki_ * rsum_;
    }

private:
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double past_e_ = 0.0;
    double rsum_ = 0.0;
    double i_limit_ = 0.0;
    bool primed_ = false;
};

// Position outer loop feeding a velocity inner loop, with the teleop
// behaviour from bur-onboard preserved: a stick past the deadband drives the
// velocity loop directly while the position loop latches a hold setpoint.
class CascadedPositionVelocityController
{
public:
    CascadedPositionVelocityController() {}

    void setPositionGains(double kp, double ki, double kd) { position_.setGains(kp, ki, kd); }
    void setVelocityGains(double kp, double ki, double kd) { velocity_.setGains(kp, ki, kd); }
    void setIntegralLimits(double pos, double vel)
    {
        position_.setIntegralLimit(pos);
        velocity_.setIntegralLimit(vel);
    }
    void setUsingExternalVelocityTarget(bool v) { using_external_target_vel_ = v; }
    void setUsingAngles(bool v) { using_angles_ = v; }

    void reset()
    {
        position_.reset();
        velocity_.reset();
        position_setpoint_ = 0.0;
    }

    double computeCommand(double current_pos, double target_pos,
                          double current_vel, double target_vel, double dt)
    {
        if (using_external_target_vel_) {
            if (std::abs(target_vel) < kVelDeadband) {
                target_pos = position_setpoint_;   // hold where we were
            } else {
                position_setpoint_ = current_pos;
            }
        }

        double position_error = target_pos - current_pos;
        if (using_angles_) {
            position_error = angle_wrap_pi(position_error);
        }
        double vel_target = position_.computeCommand(position_error, dt);

        double velocity_error;
        if (using_external_target_vel_ && std::abs(target_vel) >= kVelDeadband) {
            velocity_error = target_vel - current_vel;
        } else {
            velocity_error = vel_target - current_vel;
        }
        return velocity_.computeCommand(velocity_error, dt);
    }

private:
    static constexpr double kVelDeadband = 0.1;

    PID position_;
    PID velocity_;

    double position_setpoint_ = 0.0;
    bool using_external_target_vel_ = false;
    bool using_angles_ = false;
};

#endif
