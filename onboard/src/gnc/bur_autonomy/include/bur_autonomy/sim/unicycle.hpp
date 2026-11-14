#ifndef BUR_AUTONOMY_SIM_UNICYCLE_HPP
#define BUR_AUTONOMY_SIM_UNICYCLE_HPP

// Kinematic unicycle for the mission tests and bench_sim_node.
//
// NOT a model of the vehicle. Its numbers are invented; it exists to prove the
// plumbing and the tree logic (the vehicle cannot strafe, so a unicycle is the
// right shape). Real speed and turn rate come from M-3.

#include <cmath>

#include "bur_autonomy/geometry.hpp"

namespace bur_autonomy
{
namespace sim
{

struct Unicycle
{
    double x = 0.0, y = 0.0, yaw = 0.0;
    double v = 0.0, r = 0.0;    // current forward speed and yaw rate
    double tau_s = 0.3;         // first-order lag on both, so commands are not instant

    void step(double surge_cmd, double yaw_rate_cmd, double dt)
    {
        if (dt <= 0.0) { return; }
        const double a = (tau_s > 0.0) ? std::fmin(1.0, dt / tau_s) : 1.0;
        v += a * (surge_cmd - v);
        r += a * (yaw_rate_cmd - r);
        yaw = wrapAngle(yaw + r * dt);
        x += v * std::cos(yaw) * dt;
        y += v * std::sin(yaw) * dt;
    }

    // Bearing from the body to a point, +ve to port.
    double bearingTo(double tx, double ty) const
    {
        return wrapAngle(std::atan2(ty - y, tx - x) - yaw);
    }

    double rangeTo(double tx, double ty) const
    {
        return std::hypot(tx - x, ty - y);
    }
};

}  // namespace sim
}  // namespace bur_autonomy

#endif
