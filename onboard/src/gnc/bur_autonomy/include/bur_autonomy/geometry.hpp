#ifndef BUR_AUTONOMY_GEOMETRY_HPP
#define BUR_AUTONOMY_GEOMETRY_HPP

// Small geometry helpers for the behaviour-tree leaves.
//
// D-7: these replace nav2_util::geometry_utils::euclidean_distance, the only
// thing the previous bur_autonomy used from nav2_util. Pulling in nav2 (a very
// large install on a Pi) for a square root is not worth it. Do not add nav2
// back for helpers like these; put them here instead.
//
// Header-only and ROS-free: the templates accept anything with x/y/z members,
// such as geometry_msgs::msg::Point or a pose's `.position`, so the leaves
// need no conversions and the tests need no ROS.

#include <cmath>

namespace bur_autonomy
{

// Distance in the horizontal (x, y) plane, ignoring depth.
//
// This is what the old manager actually computed: nav2's euclidean_distance
// defaults to is_3d = false. It is also the right measure for "have I reached
// the target" on this vehicle, because depth is held by a separate loop.
template <typename A, typename B>
double horizontalDistance(const A & a, const B & b)
{
    return std::hypot(static_cast<double>(b.x) - static_cast<double>(a.x),
                      static_cast<double>(b.y) - static_cast<double>(a.y));
}

// Full 3D distance, depth included.
template <typename A, typename B>
double distance3d(const A & a, const B & b)
{
    const double dx = static_cast<double>(b.x) - static_cast<double>(a.x);
    const double dy = static_cast<double>(b.y) - static_cast<double>(a.y);
    const double dz = static_cast<double>(b.z) - static_cast<double>(a.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

constexpr double kPi = 3.14159265358979323846;

// Wrap an angle to (-pi, pi].
inline double wrapAngle(double a)
{
    if (!std::isfinite(a)) { return 0.0; }
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0.0) { a += 2.0 * kPi; }
    return a - kPi;
}

// Proportional heading law shared by every leaf that steers: yaw rate from a
// heading error (already wrapped), clamped to +/- max_rate. +ve = to port.
inline double headingToYawRate(double heading_error, double kp, double max_rate)
{
    double r = kp * heading_error;
    if (r > max_rate) { r = max_rate; }
    if (r < -max_rate) { r = -max_rate; }
    return r;
}

}  // namespace bur_autonomy

#endif
