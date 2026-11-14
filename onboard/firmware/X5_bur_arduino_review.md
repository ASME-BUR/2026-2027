# X-5: what to keep from last year's firmware

Review of [`ASME-BUR/bur-arduino`](https://github.com/ASME-BUR/bur-arduino)
at `131181e` (2025-08-19, its only commit). Written 2026-11-07 as input to
F-1, F-3 and F-5. The firmware built on it is [`bur_mcu/`](bur_mcu/README.md).

## What is in the repo

| Sketch | What it does | Status |
|---|---|---|
| `arduino_thruster/` | Subscribes to `ThrusterCommand` and drives 8 ESCs through `Servo` on pins 4-11, plus 3 utility outputs on pins 24/26/28 | Ran on the vehicle |
| `arduino_imu/` | Reads the MTi-3 over I2C and publishes `/imu` and `/mag` | Ran on the vehicle |
| `arduino_merge/` | The two above combined into one sketch on one agent | README says **"unused + untested"** |

All three are PlatformIO projects with `board = dueUSB`, so they target the
**Arduino Due on its Native USB port**. They use
[`micro_ros_platformio`](https://github.com/micro-ROS/micro_ros_platformio)
with `board_microros_distro = humble` and `board_microros_transport = serial`.

`arduino_merge` is the closest to what we need: one Due and one agent. It is
also the least trustworthy of the three, so treat it as a layout sketch and
take the working parts from the other two.

## Reuse as-is

**Build setup.** Copy this `platformio.ini` almost unchanged:

```ini
[env:dueUSB]
platform = atmelsam
board = dueUSB
framework = arduino
board_microros_distro = humble        ; must match the agent's distro, see below
board_microros_transport = serial
lib_deps = https://github.com/micro-ROS/micro_ros_platformio
           arduino-libraries/Servo@1.2.1
           Wire
platform_packages = framework-arduino-sam@https://github.com/ASME-BUR/ArduinoCore-sam.git
```

- **The `ArduinoCore-sam` fork changes exactly one line.** It raises
  `BUFFER_LENGTH` in `libraries/Wire/src/Wire.h` from 32 to 256 (2024-03-06,
  plus a `package.json` so PlatformIO accepts it). The MTi driver reads a whole
  measurement packet in one `requestFrom`, and that packet is longer than 32
  bytes. Keep the fork for as long as the IMU is on I2C. The thruster sketch
  didn't need it.
- **Distro:** the agent and the firmware should be on the same ROS distro. The
  vehicle computer runs Ubuntu 22.04 / Humble, so keep `humble` for now and
  switch both together when the new laptop arrives. The rest of the workspace
  targets Jazzy.

**Custom messages.** `micro_ros_platformio` builds any ROS package it finds
under `<project>/extra_packages/` into the firmware's message library.
`arduino_thruster` filled that directory from `bur-interfaces` with
`vcs import` (`scripts/setup.sh`, `required.repos`). For F-1, put
`src/interfaces/bur_msgs` there, preferably as a symlink, so the firmware can
never drift from the Pi's copy of the messages.

**Transport.** `set_microros_serial_transports(SerialUSB)` on the Native USB
port. Baud rates differ between the sketches (115200 for thrusters, 250000 for
the IMU, matching the launch file). The Native port is USB CDC, so the number
passed to `SerialUSB.begin()` has no effect on it. X-2's udev rule should be
keyed to the Native port's serial number.

**Agent-connection state machine.** All three sketches use the standard
micro-ROS reconnect pattern:

```
WAITING_AGENT -> AGENT_AVAILABLE -> AGENT_CONNECTED -> AGENT_DISCONNECTED -> (destroy, retry)
```

- The `RCCHECK` / `EXECUTE_EVERY_N_MS` macros and the
  `create_entities()` / `destroy_entities()` pair carry over.
- Use `rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0)` on destroy.
- Use best-effort pub/sub.

**Servo output.** `Servo::attach(pin)` plus `writeMicroseconds()` gives a 50 Hz
frame by default, which is what the SER0067 wants. The SAM `Servo` library
drives up to 12 servos per timer, enough for 4 chain servos plus fins. Pins
4-11 already have a wiring precedent.

**MTi-3 driver** (`lib/Xsens_MTi_I2C/`, Xsens' official Arduino example,
lightly modified).

- It works on the Due on **`Wire1`** (SDA1/SCL1) at 400 kHz, with DRDY on D3
  and I2C address `0x6B`.
- `configureOutputs(100)` sets 100 Hz quaternion, acceleration, rate of turn
  and magnetic field. Rate of turn arrives in **rad/s**.
- `resetInclination()` is called at boot.

## Do not carry over

These are defects in the old code. Most of them would break the servo loop or
the watchdog.

1. **There is no command watchdog. The outputs go to neutral only when the
   agent stops answering pings** (`arduino_thruster/src/main.cpp:156-176`).
   - If the Pi's controller node dies but the agent stays up, the last
     command is held forever. That is lesson 2 in DEVELOPMENT.md, and it is
     exactly what F-3 must fix.
   - F-3 needs a timestamp of the last `GaitCommand` and a coast to neutral
     after about 300 ms of silence, independent of the agent state.
2. **The loop blocks for much longer than one 20 ms servo frame.** The gait has
   to be ticked on a fixed 20 ms schedule (`micros()`-based) that nothing
   below can delay. Keep every micro-ROS call short.
   - `rmw_uros_ping_agent(200, 3)` can block for about 600 ms.
   - `rclc_executor_spin_some(..., 500 ms)` can block for up to 500 ms.
   - `rmw_uros_sync_session(100)` is called on **every IMU sample**, so each
     sample can block for up to 100 ms. Sync time once on connect, then every
     few seconds.
3. **The driver prints debug text to `SerialUSB`, which is also the micro-ROS
   transport.**
   - `goToConfig()`, `goToMeasurement()`, `configureOutputs()` and `detect()`
     print unconditionally (`MTi.cpp:14,45,166,175`). Any print after
     `set_microros_serial_transports()` corrupts the XRCE stream.
   - `arduino_merge` commented out two of these prints and missed the rest.
   - Strip every `SerialUSB.print` from the driver, or redirect it to the
     programming port (`Serial`).
4. **The IMU timestamp is wrong.**
   - `nanosec = rmw_uros_epoch_nanos()` stores the full epoch time in
     nanoseconds into a `uint32` (`arduino_imu/src/main.cpp:81`).
   - It should be `epoch_nanos % 1000000000`, and `sec` should use integer
     division, not `pow(10, 9)`.
   - The EKF will reject or misorder the samples otherwise.
5. **The IMU covariances look like standard deviations stored as variances,
   in mismatched units.** One example: `0.003 deg/s/sqrt(Hz) * sqrt(100)`
   becomes `5.2e-4`, which is a standard deviation in rad/s, placed in the
   variance slot. They are diagonal, which is the one thing they got right.
   Recompute them from the MTi-3 datasheet; C-5 retunes them anyway.
6. **`Xbus::parseNotification` only updates `configState` when `verbose` is
   true** (`Xbus.cpp`, `GOTOCONFIGACK` / `GOTOMEASUREMENTACK`). The `break`
   sits inside `if (verbose)`, so with `verbose = false` the cases fall
   through and the flag is never set.
   - As a result, every config call re-sends `goToConfig` (and prints).
   - Fix it by moving the `configState` assignment and the `break` out of the
     `if`.
7. **The old command handlers index arrays without checking their length.**
   - `ThrusterCommand` used `data[i+1]`, and its length check is commented
     out.
   - `arduino_merge` indexes `buttons.data[i*3-2]`, which is `-2` when
     `i = 0`.
   - `GaitCommand` has only fixed fields, so it avoids this. Do not
     reintroduce array indexing in `ServoChainCommand` handling without a
     bounds check.
8. **Two separate executors in `arduino_merge`.** One executor with one timer
   and one subscription is enough. Two just add another blocking
   `spin_some`.

## Things F-1 has to handle that the old code never did

- **Message memory.** micro-ROS does not allocate memory on receive.
  - `GaitCommand` contains a `std_msgs/Header`, which holds a `frame_id`
    string. Preallocate it with
    `micro_ros_utilities_create_message_memory` (`arduino_merge/test/main2.cpp`
    started on this), or a non-empty `frame_id` from the Pi will fail
    deserialisation and the message will be dropped silently.
  - The same applies to `ServoChainState`'s unbounded `float32[]` fields when
    publishing. Give them capacity 4.
- **Wave maths.** Include `src/gnc/bur_gait/include/bur_gait/gait.hpp`
  directly, as F-1 says. It uses `std::vector` and `<cmath>`, which
  arm-none-eabi-g++ supports through newlib. Check that the PlatformIO build
  flags select C++17 (`build_unflags = -std=gnu++11`,
  `build_flags = -std=gnu++17`); the default SAM core uses gnu++11.
- **`ServoChainState.sequence`** says it echoes the last `GaitCommand`, but
  `GaitCommand` has no sequence field. Echo `header.stamp`, or add a field.
  That decision belongs with F-2.

## IMU bus (decided 2026-11-07: keep `Wire1`)

The hardware baseline notes say to put the MTi-3 on **UART or SPI, not I2C**,
to avoid sharing a bus with the Bar30's blocking read. The old setup already
puts the MTi on **`Wire1`**, the Due's *second* I2C bus. Putting the Bar30 on
`Wire` (the first bus) removes the shared-bus problem.

Reading one MTi packet on its own bus is short:

- about 60 bytes at 400 kHz, so roughly 1.5-2 ms
- triggered by DRDY, so the read happens only when data is ready

Keeping I2C on `Wire1` reuses a driver that is known to work on this board.
Moving to UART means writing a new Xbus-over-UART reader. **Recommendation:
keep `Wire1` for F-5**, and revisit only if M-7 or the gait shows a timing
problem. This is what `bur_mcu` does; the hardware baseline in
DEVELOPMENT.md records it.
