#include "utility_func.hpp"

// Carried over from bur-onboard. The quaternion->Euler maths is unchanged.
//
// NOTE: bur-onboard also declared a `template <typename T> T clamp(...)` in the
// header and *defined* it in this .cpp, which means it could never actually be
// instantiated by another translation unit. Nothing used it, so it went
// unnoticed. Replaced with a plain non-template clamp_d.

EulerAngles ToEulerAngles(Quaternion q)
{
    return ToEulerAngles(q.w, q.x, q.y, q.z);
}

EulerAngles ToEulerAngles(double w, double x, double y, double z)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (w * y - x * z));
    double cosp = std::sqrt(1 - 2 * (w * y - x * z));
    angles.y = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.z = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

double angle_wrap_pi(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double clamp_d(double val, double low, double high)
{
    if (val < low) { return low; }
    if (val > high) { return high; }
    return val;
}
