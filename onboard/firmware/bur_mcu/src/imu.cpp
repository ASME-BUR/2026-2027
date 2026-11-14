#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "MTi.h"
#include "config.h"

namespace bur_mcu
{

bool Imu::begin()
{
    Wire1.begin();
    Wire1.setClock(config::kImuI2cClockHz);
    pinMode(config::kImuDrdyPin, INPUT);

    static MTi mti(config::kImuI2cAddress, config::kImuDrdyPin);
    mti_ = &mti;

    // detect() sends goToConfig until the MTi acknowledges. It used to be
    // skipped (last year's sketch configured blindly), which made a missing
    // IMU indistinguishable from a working one.
    if (!mti_->detect(config::kImuDetectTimeoutMs)) {
        Serial.println("bur_mcu: no MTi on Wire1, running without IMU");
        present_ = false;
        return false;
    }

    // Quaternion, acceleration, rate of turn (rad/s) and magnetic field.
    mti_->configureOutputs(config::kImuRateHz);

    // Last year's sketch called resetInclination() here. It is dropped on
    // purpose: it re-zeroes roll/pitch to whatever attitude the vehicle has at
    // power-on, so a watchdog reset mid-swim would bake in a tilt. The EKF reads
    // attitude in relative mode (ekf.yaml imu1) and handles the initial offset.

    mti_->goToMeasurement();
    present_ = true;
    Serial.println("bur_mcu: MTi detected and configured");
    return true;
}

bool Imu::poll(ImuSample & out)
{
    if (!present_ || digitalRead(config::kImuDrdyPin) == LOW) {
        return false;
    }
    if (!mti_->readMessages()) {
        return false;
    }

    const float * q = mti_->getQuat();
    const float * w = mti_->getRateOfTurn();
    const float * a = mti_->getAcceleration();

    // The driver initialises its buffers to NAN; wait until all three outputs
    // have arrived at least once.
    if (isnan(q[0]) || isnan(w[0]) || isnan(a[0])) {
        return false;
    }

    // TODO(verify): F-5 / M-5. The MTi's sensor frame and the vehicle's imu
    // link must agree, and positive yaw rate must be to port (NWU). If the
    // module is mounted z-up with x toward the nose, its frame already matches
    // and this is the identity. Otherwise, rotate here (or configure the MTi's
    // alignment matrix) rather than in the EKF.
    for (int i = 0; i < 4; ++i) { out.quat_wxyz[i] = q[i]; }
    for (int i = 0; i < 3; ++i) {
        out.gyro_rad_s[i] = w[i];
        out.accel_m_s2[i] = a[i];
    }
    return true;
}

}  // namespace bur_mcu
