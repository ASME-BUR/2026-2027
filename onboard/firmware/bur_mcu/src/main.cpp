// bur_mcu: Arduino Due firmware for the 2026-2027 swimming vehicle.
//
//   /gait_command (bur_msgs/GaitCommand) --> phase accumulator + gait.hpp
//                                            --> 4 chain servos, 50 Hz PWM
//   MTi-3 on Wire1                         --> /imu (sensor_msgs/Imu)
//   chain status                           --> /servo_chain_state
//
// Tasks: F-1 (gait), F-3 (watchdog), F-5 (IMU). See onboard/DEVELOPMENT.md and
// firmware/X5_bur_arduino_review.md.
//
// Structure of loop(), in priority order:
//   1. the servo tick, on a fixed 20 ms schedule from micros(). It never waits
//      on anything else, and it enforces the 300 ms command timeout itself,
//      whatever state the micro-ROS link is in.
//   2. the IMU, read only when its DRDY pin is high.
//   3. the micro-ROS link. Every call that waits on the agent has a timeout of
//      a few milliseconds.
// The SAM3X hardware watchdog resets the board if loop() stops running at all.

// Standard headers BEFORE Arduino.h: the SAM core defines function-like macros
// (Max, min, max) that break <vector> and <cmath> if they are parsed first.
#include <cmath>
#include <cstring>
#include <vector>

#include "chain_logic.hpp"

#include <Arduino.h>
#include <Servo.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <bur_msgs/msg/gait_command.h>
#include <bur_msgs/msg/servo_chain_state.h>
#include <sensor_msgs/msg/imu.h>

#include "config.h"
#include "imu.h"

namespace cfg = bur_mcu::config;

// ---------------------------------------------------------------------------
// Hardware watchdog
// ---------------------------------------------------------------------------
//
// The SAM core's init() calls watchdogSetup(), whose default disables the
// watchdog. The WDT mode register can be written only once per reset, so a
// disabled watchdog can never be re-enabled. Defining this (empty) keeps the
// reset-default watchdog (~16 s) running through setup(); setup() then sets the
// real timeout with watchdogEnable(), the one permitted write.
void watchdogSetup(void) {}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

namespace
{

bur_mcu::ChainLimits makeLimits()
{
    bur_mcu::ChainLimits l;
    l.command_timeout_us = cfg::kCommandTimeoutUs;
    return l;
}

// Gait shape comes from ChainConfig's defaults in gait.hpp, which match
// gait.yaml and the URDF.
bur_mcu::ChainLogic chain(bur_gait::ChainConfig{}, makeLimits());
const bur_mcu::ServoMap servo_map;
Servo servos[cfg::kNumChainServos];

bur_mcu::Imu imu;
bur_mcu::ImuSample imu_sample;
bool imu_fresh = false;

uint32_t next_tick_us = 0;
uint32_t tick_overruns = 0;

// micro-ROS
enum class Link { kWaiting, kAvailable, kConnected, kDisconnected };
Link link = Link::kWaiting;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t gait_sub;
rcl_publisher_t state_pub;
rcl_publisher_t imu_pub;
rclc_executor_t executor;

bur_msgs__msg__GaitCommand gait_msg;
char gait_frame_buf[32];   // receive buffer for header.frame_id (micro-ROS
                           // does not allocate; a longer frame_id is dropped)
bur_msgs__msg__ServoChainState state_msg;
sensor_msgs__msg__Imu imu_msg;

char frame_base_link[] = "base_link";
char frame_imu[] = "imu";

uint32_t last_ping_ms = 0;
uint8_t ping_failures = 0;
uint32_t last_sync_ms = 0;
uint32_t last_state_ms = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Teardown and best-effort publishes have nothing useful to do on failure: a
// lost publish is replaced 10 ms later, and a failed fini is followed by a
// fresh session. (A plain (void) cast does not silence GCC's warn_unused_result.)
inline void ignoreResult(rcl_ret_t) {}

void setString(rosidl_runtime_c__String & s, char * text)
{
    s.data = text;
    s.size = strlen(text);
    s.capacity = s.size + 1;
}

void stampNow(builtin_interfaces__msg__Time & t)
{
    // X-5 review, defect 4: last year's code stored the full epoch nanoseconds
    // in the uint32 nanosec field.
    const int64_t ns = rmw_uros_epoch_nanos();
    t.sec = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

void writeServos(const std::vector<double> & angles)
{
    for (uint8_t i = 0; i < cfg::kNumChainServos && i < angles.size(); ++i) {
        servos[i].writeMicroseconds(bur_mcu::angleToMicros(
            angles[i], cfg::kChainDirection[i], cfg::kChainTrimUs[i], servo_map));
    }
}

void haltForever(uint16_t blink_ms)
{
    // Configuration error. Servos are not attached, so they receive no pulses.
    pinMode(cfg::kLedPin, OUTPUT);
    for (;;) {
        digitalWrite(cfg::kLedPin, !digitalRead(cfg::kLedPin));
        delay(blink_ms);
        watchdogReset();
    }
}

// ---------------------------------------------------------------------------
// micro-ROS
// ---------------------------------------------------------------------------

void gaitCallback(const void * msgin)
{
    const auto * m = static_cast<const bur_msgs__msg__GaitCommand *>(msgin);
    bur_mcu::GaitCommand c;
    c.mode = m->mode;
    c.params.frequency_hz = m->frequency_hz;
    c.params.amplitude = m->amplitude;
    c.params.wavelength = m->wavelength;
    c.params.turn_bias = m->turn_bias;
    c.fin_angle_rad = m->fin_angle_rad;   // TODO(port): no fin servo until D-3
    chain.onCommand(c, micros());
}

#define RCCHECK(fn)                       \
    do {                                  \
        if ((fn) != RCL_RET_OK) {         \
            return false;                 \
        }                                 \
        watchdogReset();                  \
    } while (0)

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Do not wait long for the agent to confirm each entity.
    rmw_context_t * ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_creation_session_timeout(ctx, cfg::kEntityTimeoutMs);

    RCCHECK(rclc_node_init_default(&node, "bur_mcu", "", &support));

    // Best effort everywhere. A reliable XRCE publisher waits for delivery
    // confirmation, which would stall the servo tick. GaitCommand is sent at
    // 10 Hz and the watchdog covers losses.
    RCCHECK(rclc_subscription_init_best_effort(
        &gait_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(bur_msgs, msg, GaitCommand),
        "gait_command"));
    RCCHECK(rclc_publisher_init_best_effort(
        &state_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(bur_msgs, msg, ServoChainState),
        "servo_chain_state"));
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &gait_sub, &gait_msg, &gaitCallback,
                                           ON_NEW_DATA));

    // Sync the clock once now, then every kTimeSyncPeriodMs. Not on every
    // sample (X-5 review, defect 2).
    rmw_uros_sync_session(cfg::kTimeSyncTimeoutMs);
    last_sync_ms = millis();
    return true;
}

void destroyEntities()
{
    rmw_context_t * ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);

    ignoreResult(rcl_subscription_fini(&gait_sub, &node));
    ignoreResult(rcl_publisher_fini(&state_pub, &node));
    ignoreResult(rcl_publisher_fini(&imu_pub, &node));
    ignoreResult(rclc_executor_fini(&executor));
    ignoreResult(rcl_node_fini(&node));
    ignoreResult(rclc_support_fini(&support));
}

bool pingAgent()
{
    return rmw_uros_ping_agent(cfg::kPingTimeoutMs, 1) == RMW_RET_OK;
}

void publishImu()
{
    // Without a synced clock the stamp would be time since boot, and the EKF
    // would treat every sample as decades old.
    if (!rmw_uros_epoch_synchronized()) {
        return;
    }
    stampNow(imu_msg.header.stamp);
    imu_msg.orientation.w = imu_sample.quat_wxyz[0];
    imu_msg.orientation.x = imu_sample.quat_wxyz[1];
    imu_msg.orientation.y = imu_sample.quat_wxyz[2];
    imu_msg.orientation.z = imu_sample.quat_wxyz[3];
    imu_msg.angular_velocity.x = imu_sample.gyro_rad_s[0];
    imu_msg.angular_velocity.y = imu_sample.gyro_rad_s[1];
    imu_msg.angular_velocity.z = imu_sample.gyro_rad_s[2];
    imu_msg.linear_acceleration.x = imu_sample.accel_m_s2[0];
    imu_msg.linear_acceleration.y = imu_sample.accel_m_s2[1];
    imu_msg.linear_acceleration.z = imu_sample.accel_m_s2[2];
    ignoreResult(rcl_publish(&imu_pub, &imu_msg, NULL));
}

void publishState()
{
    if (rmw_uros_epoch_synchronized()) {
        stampNow(state_msg.header.stamp);
    }
    // position_rad / current_a / temperature_c stay empty: the SER0067 has no
    // feedback over PWM (F-2). fault_flags stays 0 until F-2 defines the bits.
    // `sequence` counts GaitCommands received (mod 256), since GaitCommand has
    // no sequence field to echo; a frozen value means commands are not arriving.
    state_msg.fault_flags = 0;
    state_msg.sequence = static_cast<uint8_t>(chain.received());
    ignoreResult(rcl_publish(&state_pub, &state_msg, NULL));
}

void serviceLink()
{
    const uint32_t now_ms = millis();
    switch (link) {
    case Link::kWaiting:
        if (now_ms - last_ping_ms >= cfg::kPingPeriodMs) {
            last_ping_ms = now_ms;
            if (pingAgent()) {
                link = Link::kAvailable;
            }
        }
        break;

    case Link::kAvailable:
        if (createEntities()) {
            link = Link::kConnected;
            ping_failures = 0;
            last_ping_ms = now_ms;
        } else {
            destroyEntities();
            link = Link::kWaiting;
        }
        break;

    case Link::kConnected:
        if (now_ms - last_ping_ms >= cfg::kPingPeriodMs) {
            last_ping_ms = now_ms;
            ping_failures = pingAgent() ? 0 : ping_failures + 1;
            if (ping_failures >= cfg::kPingFailuresToDisconnect) {
                link = Link::kDisconnected;
                break;
            }
        }
        (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
        if (imu_fresh) {
            publishImu();
            imu_fresh = false;
        }
        if (now_ms - last_state_ms >= cfg::kStatePeriodMs) {
            last_state_ms = now_ms;
            publishState();
        }
        if (now_ms - last_sync_ms >= cfg::kTimeSyncPeriodMs) {
            last_sync_ms = now_ms;
            rmw_uros_sync_session(cfg::kTimeSyncTimeoutMs);
        }
        break;

    case Link::kDisconnected:
        // No command can arrive now, so the chain logic is already counting
        // down to coast. Nothing to do for the servos here.
        destroyEntities();
        link = Link::kWaiting;
        break;
    }
    digitalWrite(cfg::kLedPin, link == Link::kConnected ? HIGH : LOW);
}

void initMessages()
{
    memset(&gait_msg, 0, sizeof(gait_msg));
    gait_msg.header.frame_id.data = gait_frame_buf;
    gait_msg.header.frame_id.size = 0;
    gait_msg.header.frame_id.capacity = sizeof(gait_frame_buf);

    memset(&state_msg, 0, sizeof(state_msg));
    setString(state_msg.header.frame_id, frame_base_link);

    memset(&imu_msg, 0, sizeof(imu_msg));
    setString(imu_msg.header.frame_id, frame_imu);
    namespace c = bur_mcu::imu_cov;
    // Row-major 3x3, diagonal only (lesson 3 in DEVELOPMENT.md).
    imu_msg.orientation_covariance[0] = c::kRollPitchVar;
    imu_msg.orientation_covariance[4] = c::kRollPitchVar;
    imu_msg.orientation_covariance[8] = c::kYawVar;
    imu_msg.angular_velocity_covariance[0] = c::kGyroVar;
    imu_msg.angular_velocity_covariance[4] = c::kGyroVar;
    imu_msg.angular_velocity_covariance[8] = c::kGyroVar;
    imu_msg.linear_acceleration_covariance[0] = c::kAccelVar;
    imu_msg.linear_acceleration_covariance[4] = c::kAccelVar;
    imu_msg.linear_acceleration_covariance[8] = c::kAccelVar;
}

}  // namespace

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup()
{
    pinMode(cfg::kLedPin, OUTPUT);
    digitalWrite(cfg::kLedPin, LOW);

    // Debug text goes to the PROGRAMMING port. SerialUSB (Native port) is
    // reserved for micro-ROS: anything else written there corrupts the link.
    Serial.begin(115200);

    if (chain.config().n_joints != cfg::kNumChainServos) {
        Serial.println("bur_mcu: kChainPins does not match ChainConfig::n_joints");
        haltForever(100);
    }

    // Servos first, straight, so the chain is at neutral before anything slow
    // happens. The Servo library's pulses are timer-driven and keep running
    // while setup() blocks. 500-2500 us must be passed to attach(): the
    // library's default range is 544-2400 us and would clip the SER0067.
    for (uint8_t i = 0; i < cfg::kNumChainServos; ++i) {
        servos[i].attach(cfg::kChainPins[i], servo_map.min_us, servo_map.max_us);
    }
    writeServos(std::vector<double>(cfg::kNumChainServos, 0.0));

    imu.begin();
    initMessages();

    // The Native port is USB CDC, so the baud rate passed here has no effect.
    // Do not wait for the host to open the port.
    SerialUSB.begin(115200);
    set_microros_serial_transports(SerialUSB);

    watchdogEnable(cfg::kHardwareWatchdogMs);
    next_tick_us = micros();
}

void loop()
{
    const uint32_t now_us = micros();
    if (static_cast<int32_t>(now_us - next_tick_us) >= 0) {
        writeServos(chain.tick(now_us));
        next_tick_us += cfg::kTickUs;
        if (static_cast<int32_t>(now_us - next_tick_us) >= 0) {
            // More than a whole tick late. Resynchronise instead of firing a
            // burst of catch-up ticks.
            next_tick_us = now_us + cfg::kTickUs;
            ++tick_overruns;
        }
    }

    watchdogReset();

    if (imu.poll(imu_sample)) {
        imu_fresh = true;
    }

    serviceLink();
}
