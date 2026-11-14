# TAIL_DEV: horizontal tail motion (surge + yaw)

Part of [DEVELOPMENT.md](DEVELOPMENT.md). Task IDs, the `BLOCKED` rule and the
hardware baseline are defined there. IDs are shared across all three files and
never reused.

**Status, 2026-11-14: paused.** The tail segments are not built, and the chain
servos are not wired to the Due. Nothing on this page can make progress until
they are, except C-2. Active work moved to [sensors_dev.md](sensors_dev.md).

**Goal:** the vehicle swims forward and turns on command in a pool. Surge is
open-loop (speed -> tail-beat frequency). The yaw-rate loop closes on the IMU
gyro, which comes from [sensors_dev.md](sensors_dev.md) F-5.

**Done when:** a wet test where commanded surge and yaw rate are tracked, the
watchdog makes the body go limp on link loss, and the gains come from measured
tables, not guesses.

---

## Critical path

```
                           F-1 gait firmware ─┬─> F-3 watchdog ──────────────┐
                                              ├─> M-3 plant table ──> C-1 ───┤
                                              ├─> M-4 envelope               ├─> wet test
           F-1 + F-5 IMU (sensors_dev) ───────┴─> M-7 IMU noise ──> C-4 ─────┤
                  A-1 manager ──> A-2 leaves ──> A-3 load test ──────────────┘
```

**When the chain is wired, start with:** the F-1 and F-3 items in the firmware
hardware checklist,
[`firmware/bur_mcu/README.md`](firmware/bur_mcu/README.md#not-verified-yet).

| Task | Status |
|---|---|
| F-1 gait firmware | Written, compiles for the Due, never run on a board |
| F-2 servo telemetry | Blocked by F-1 |
| F-3 watchdog | Written, unit-tested on host, never run on a board |
| M-1, M-2, M-8 | Need the built tail |
| M-3, M-4, M-7 | Blocked by F-1 (need a swimming vehicle) |
| C-1, C-4 | Blocked by M-3, M-7 |
| C-2 | **Unblocked**, software only |

---

## M — Measurements

These need the physical vehicle. Until they exist, every related number in the
tree is a placeholder.

### M-1 — Body dimensions and masses
**Feeds:** URDF, MuJoCo

- **Input:** the mechanical design or the built vehicle; a scale; calipers
- **Output:** segment lengths, widths, heights, masses written into
  `bur_description/urdf/bur.urdf.xacro`
- **Done when:** the URDF matches the vehicle to within a few mm and the masses
  sum to the measured total

### M-2 — Servo bandwidth and stall current
**Feeds:** `gait.yaml` (`max_rate_rad_s`), `controller.yaml` (`freq_max`), C-1

Datasheet speed (4.4 rad/s at 6 V) is unloaded. What matters is behaviour under
hydrodynamic load, inside the covering.

- **Input:** one covered servo, a representative tail segment, a current meter,
  water
- **Output:** maximum sustainable slew rate, stall current, thermal limit
- **Done when:** `freq_max` reflects a rate the servos can hold without
  overheating

### M-3 — The plant table
**Blocked by:** F-1 · **Feeds:** C-1, A-4, all controller tuning

**The most valuable measurement on the list.** Without it, tuning is guessing
and commanded speed will not match actual speed.

```
for frequency in 0.5 1.0 1.5 2.0:
  for amplitude in 0.3 0.5 0.7:
    hold the gait, measure steady-state surge speed
for turn_bias in 0.05 0.1 0.2 0.3:
    hold it, measure steady-state yaw rate
```

- **Input:** a swimming vehicle, a pool, a measured distance, a stopwatch
- **Output:** two lookup tables, committed next to
  `bur_controller/config/controller.yaml`
- **Done when:** C-1 can replace the `surge_to_freq` gain with it

### M-4 — Amplitude envelope shape
**Blocked by:** F-1 · **Feeds:** `gait.yaml` (`envelope_exponent`)

With 4 joints the current `s^2` envelope gives joint_0 an amplitude of exactly
zero (0, 0.11, 0.44, 1.0), so it contributes only turn bias and wastes a
quarter of the actuation. Give it a non-zero floor or lower the exponent, then
measure.

- **Input:** the vehicle; the ability to drive one joint at a time
- **Output:** a measured envelope, or a confirmed exponent
- **Done when:** every joint contributes thrust

### M-7 — IMU undulation noise
**Blocked by:** F-1, F-5 · **Feeds:** C-4, C-5

**The biggest unknown in estimation, and it hits yaw directly.** The undulating
body injects a periodic angular rate at the tail-beat frequency into the gyro
that the yaw-rate loop closes on. The same sway reaches the ZED in the nose
(see [sensors_dev.md](sensors_dev.md) P-3).

- **Input:** raw `/imu` recordings while swimming at a range of tail-beat
  frequencies
- **Output:** noise spectra; a choice between notch-filtering (the frequency is
  known exactly, because the controller commands it) and inflating process noise
- **Done when:** C-4 is implemented against real data

### M-8 — Joint limits from hull geometry
**Blocked by:** M-1

The SER0067 turns 360 deg, so the limit is the skin or covering, not the servo.

- **Output:** `joint_limit_rad` in `gait.yaml` and the URDF `<limit>` elements
- **Done when:** the chain cannot command an angle that fouls the hull

---

## F — Firmware

F-1, F-3 and F-5 are one PlatformIO project,
[`firmware/bur_mcu`](firmware/bur_mcu/README.md). It compiles for the Due
against Jazzy micro-ROS, and its chain logic passes host unit tests. **It has
never run on a board.** What it reuses from last year's
[`ASME-BUR/bur-arduino`](https://github.com/ASME-BUR/bur-arduino), and which
defects it avoids, is in
[`firmware/X5_bur_arduino_review.md`](firmware/X5_bur_arduino_review.md).
F-5 (IMU) is tracked in [sensors_dev.md](sensors_dev.md), because it can be
brought up with no servos attached.

### F-1 — Gait firmware
**Written, untested on hardware** · **Blocks:** M-3, M-4, M-7, everything wet

Use `src/gnc/bur_gait/include/bur_gait/gait.hpp` directly. It is ROS-free on
purpose so the **same source file** compiles into the firmware and the Pi
preview can't drift from the vehicle.

1. Get the micro-ROS hello-world running on the Due against the Pi's agent.
   (Done first by the IMU-only bring-up in sensors_dev F-5, if that happens
   first.)
2. Subscribe to `GaitCommand`, run the phase accumulator locally
   (`bur_gait::advancePhase`, then `jointAngles` and `rateLimit`, exactly as
   `gait_node.cpp` does), and drive the 4 chain servos. Time one tick on the
   Due to confirm it fits comfortably inside the 20 ms frame. It should; the
   maths is tiny. Also confirm `gait.hpp` compiles with the Due toolchain
   (`std::vector`, `<cmath>`). It already builds under strict `-std=c++17`
   (its own `kPi`, no `M_PI`), and its `ChainConfig` defaults match
   `gait.yaml`.

- **Input:** `bur_msgs/GaitCommand` over micro-ROS; `gait.hpp`
- **Output:** servo PWM at **50 Hz**; `bur_msgs/ServoChainState` published back
- **Done when:** the chain holds a clean travelling wave on the bench and
  responds to a `GaitCommand` change within one beat

### F-2 — Servo telemetry
**Blocked by:** F-1

- **Input:** servo feedback, if any is reachable. **TODO(verify):** the
  SER0067's encoder is internal and PWM is one-way. Check whether the part
  exposes a feedback pin. If it doesn't, `position_rad` stays empty and the
  only telemetry is current draw through a shunt on the servo rail.
- **Output:** `ServoChainState` populated (position, current, temperature,
  fault bits); `fault_flags` bit layout defined in `ServoChainState.msg`
- **Done when:** a stalling servo is visible on a topic rather than by smell

### F-3 — Watchdog and failsafe
**Written, untested on hardware** (with F-1): 300 ms command timeout in
`include/chain_logic.hpp` (unit-tested), plus a 1 s SAM3X hardware watchdog

- **Input:** `GaitCommand` arrival times; `MODE_ESTOP` and `MODE_COAST`
- **Output:** chain coasts to neutral after ~300 ms of silence; slew-limited
  transitions so a mode change does not brown out the servo rail
- **Done when:** pulling the USB cable mid-swim makes the body go limp rather
  than hold a turn and swim into a wall

> A held angle on a dead link bends the body into a turn. Coast, never hold.

---

## C — Controls

### C-1 — Replace the surge gain with the plant table
**Blocked by:** M-3

`bur_controller` maps commanded speed to tail-beat frequency through one guessed
gain (`surge_to_freq: 2.0`).

- **Input:** M-3's lookup tables
- **Output:** table lookup with interpolation in `controller_node.cpp`
- **Done when:** commanded speed matches measured speed within ~20%

### C-2 — Prevent double-driving
**Unblocked** (F-1 is written)

Because the Due owns the phase, both it and `bur_gait` subscribe to
`/gait_command`. `use_gait_node:=false` exists for this, but nothing enforces
it.

- **Output:** either a runtime guard, or `use_gait_node` defaulting to `false`
  with the bench launch overriding it
- **Done when:** it is impossible to drive the servos from two places by accident

### C-4 — Filter undulation out of the IMU
**Blocked by:** M-7

- **Input:** M-7's spectra; the commanded tail-beat frequency
- **Output:** a notch or low-pass stage ahead of the EKF, or justified process
  noise inflation in `ekf.yaml`
- **Done when:** yaw rate and attitude in `/odometry/filtered` stay stable while
  swimming. The final EKF retune with swimming data is C-5 in
  [sensors_dev.md](sensors_dev.md).
