#ifndef BUR_MCU_CONFIG_H
#define BUR_MCU_CONFIG_H

// Board wiring and timing for the Arduino Due. Gait shape (n_joints, envelope,
// joint limit, slew rate) is NOT here: it comes from bur_gait::ChainConfig's
// defaults in gait.hpp, which are kept equal to gait.yaml and the URDF.

#include <stdint.h>

namespace bur_mcu
{
namespace config
{

// ---- Timing ---------------------------------------------------------------

// Servo/gait tick. The SER0067 takes a 20 ms PWM frame, so 50 Hz is the most
// the chain can use.
constexpr uint32_t kTickUs = 20000;

// GaitCommand silence after which the chain coasts to neutral (F-3).
constexpr uint32_t kCommandTimeoutUs = 300000;

// ServoChainState publish period.
constexpr uint32_t kStatePeriodMs = 100;

// Hardware watchdog backstop. If loop() stops running at all (a hung USB write,
// a stuck I2C bus), the SAM3X resets and the servos come back at neutral. The
// 300 ms command timeout above is the primary failsafe; this one catches the
// case where the code that enforces it is not running.
constexpr uint32_t kHardwareWatchdogMs = 1000;

// ---- micro-ROS ------------------------------------------------------------

// Every call that waits on the agent has a short timeout, so a dead link
// cannot hold up the servo tick for long (X-5 review, defect 2).
constexpr int kPingTimeoutMs = 5;
constexpr uint32_t kPingPeriodMs = 500;
constexpr uint8_t kPingFailuresToDisconnect = 3;
constexpr int kTimeSyncTimeoutMs = 10;
constexpr uint32_t kTimeSyncPeriodMs = 10000;
constexpr int kEntityTimeoutMs = 100;

// ---- Servo chain ----------------------------------------------------------

// Chain servo PWM pins, head to tail. Pins 4-11 carried the thruster ESCs last
// year, so the wiring harness precedent is there. The array length must equal
// bur_gait::ChainConfig::n_joints; setup() halts if it does not.
// TODO(hw): confirm against the vehicle harness.
constexpr uint8_t kChainPins[] = {4, 5, 6, 7};
constexpr uint8_t kNumChainServos = sizeof(kChainPins) / sizeof(kChainPins[0]);

// +1 or -1 per joint, so that a positive angle bends the body to port (NWU).
// TODO(hw): depends on how each servo is mounted. Check by commanding a
// positive turn_bias on the bench and watching each joint.
constexpr int8_t kChainDirection[kNumChainServos] = {+1, +1, +1, +1};

// Per-joint pulse trim (us) that makes the commanded zero physically straight.
// TODO(hw): measure on the assembled chain.
constexpr int16_t kChainTrimUs[kNumChainServos] = {0, 0, 0, 0};

// Fin servos are not driven yet. GaitCommand.fin_angle_rad is received and
// ignored until D-3 settles the fin count and placement.
// TODO(port): fin pins (8 onward are free) once D-3 is decided.

// ---- IMU: Xsens MTi-3 -----------------------------------------------------
//
// Decided 2026-11-07: I2C on Wire1 (SDA1/SCL1), not UART or SPI. Wire1 is the
// Due's SECOND I2C bus; the Bar30 (F-4, deferred) goes on Wire (pins 20/21), so
// the two never share a bus and the Bar30's blocking conversion cannot stall
// an IMU read. This reuses last year's driver, which is known to work on the
// Due. Revisit only if M-7 or the gait shows a timing problem.
//
// Wire1 has no pull-ups on the Due. MTi-#-DK boards after rev 2.3 have them on
// board; older ones need 2.7 kOhm to 3.3 V on SDA1/SCL1. The MTi runs at 3.3 V
// (5 V forces it into USB mode), which matches the Due's logic level.
constexpr uint8_t kImuDrdyPin = 3;
constexpr uint8_t kImuI2cAddress = 0x6B;
constexpr uint32_t kImuI2cClockHz = 400000;
constexpr uint16_t kImuRateHz = 100;
constexpr uint16_t kImuDetectTimeoutMs = 1500;

// ---- Status LED -----------------------------------------------------------

constexpr uint8_t kLedPin = 13;   // on = agent connected

}  // namespace config
}  // namespace bur_mcu

#endif
