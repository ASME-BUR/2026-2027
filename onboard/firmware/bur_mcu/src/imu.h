#ifndef BUR_MCU_IMU_H
#define BUR_MCU_IMU_H

// Xsens MTi-3 on Wire1 (F-5). Thin wrapper around the MTi driver in
// lib/Xsens_MTi_I2C that keeps every call short enough to sit inside the 20 ms
// servo tick: poll() reads only when DRDY says a packet is waiting, and reads at
// most a few packets.

#include <stdint.h>

class MTi;

namespace bur_mcu
{

struct ImuSample
{
    float quat_wxyz[4];   // orientation
    float gyro_rad_s[3];  // angular velocity, rad/s
    float accel_m_s2[3];  // linear acceleration incl. gravity, m/s^2
};

// Diagonal variances for sensor_msgs/Imu.
//
// Derived from MTi-3 noise densities as variance = (density)^2 * bandwidth,
// with bandwidth taken as half the output rate (100 Hz -> 50 Hz).
// TODO(verify): the densities are the MTi 1-series datasheet figures as
// remembered, not re-read: gyro 0.007 deg/s/sqrt(Hz), accelerometer
// 120 ug/sqrt(Hz), roll/pitch 0.5 deg and yaw 2 deg dynamic accuracy. Check
// them against the current datasheet. These are sensor-only numbers: they say
// nothing about the undulation the body puts into the gyro (M-7), and C-5
// retunes the filter anyway.
//
// Last year's values were standard deviations stored as variances, in mixed
// units (X-5 review, defect 5). Don't reuse them.
namespace imu_cov
{
constexpr double kBandwidthHz = 50.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kGyroDensity = 0.007 * kDegToRad;          // rad/s/sqrt(Hz)
constexpr double kAccelDensity = 120e-6 * 9.80665;          // m/s^2/sqrt(Hz)
constexpr double kGyroVar = kGyroDensity * kGyroDensity * kBandwidthHz;
constexpr double kAccelVar = kAccelDensity * kAccelDensity * kBandwidthHz;
constexpr double kRollPitchVar = (0.5 * kDegToRad) * (0.5 * kDegToRad);
constexpr double kYawVar = (2.0 * kDegToRad) * (2.0 * kDegToRad);
}  // namespace imu_cov

class Imu
{
public:
    // Detect and configure the MTi, then put it in measurement mode. Blocks for
    // up to about two seconds; call from setup() only. Returns false if no MTi
    // answered, in which case poll() always returns false and the rest of the
    // firmware runs without an IMU (bench use).
    bool begin();

    // Read any waiting packets. Returns true if a complete new sample is
    // available in `out`. Cheap when DRDY is low.
    bool poll(ImuSample & out);

    bool present() const { return present_; }

private:
    MTi * mti_ = nullptr;
    bool present_ = false;
};

}  // namespace bur_mcu

#endif
