#ifndef BUR_UTILITY_FUNC_HPP
#define BUR_UTILITY_FUNC_HPP

// Carried over verbatim from bur-onboard/src/gnc/bur_controller.
// Distro-agnostic; nothing here needed changing for Jazzy.

#include <cmath>

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double x, y, z;
};

EulerAngles ToEulerAngles(Quaternion q);
EulerAngles ToEulerAngles(double w, double x, double y, double z);

// wrap an angle to [-pi, pi]
double angle_wrap_pi(double angle);

double clamp_d(double val, double low, double high);

#endif
