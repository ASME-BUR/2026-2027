# DEVELOPMENT

What still needs building, in dependency order, with the inputs and outputs of
each task. Finished and settled work is not listed here. It shows up in the
[hardware baseline](#hardware-baseline) and in the code.

For setup, structure and how to run things, see [README.md](README.md).

---

## Current state

| | |
|---|---|
| Builds on Jazzy | **yes**: 10 packages, 0 errors, 0 warnings |
| Control chain verified in sim | **yes**: `SwimCommand` -> `GaitCommand` -> joint angles |
| Run on hardware | **no** |
| Gains, dimensions, limits | **all placeholder** |
| MCU firmware | **does not exist** (`firmware/` is empty) |
| Autonomy node | **does not exist** |
| Decisions outstanding | D-6, D-7; D-3 fin layout (deferred) |
| **Current focus** | **horizontal motion only: surge + yaw** |

The two big unbuilt pieces are the **MCU firmware** and the **autonomy node**.

---

## Hardware baseline

These are settled. Don't reopen them here; they constrain everything below.

| | | Consequence |
|---|---|---|
| Chain | **4 joints, 4 servos, all revolute about Z** | Yaw moments only, no pitch authority from the body |
| Servos | **DFRobot SER0067**: 360 deg, PWM 500-2500 us, 20 ms frame, internal 12-bit encoder | Servo loop is **50 Hz** at most; no position readback over PWM (see F-2) |
| Pitch / depth | **Actuated fins with their own servos** | Fin count and placement still open (D-3, deferred) |
| Computers | **Raspberry Pi 4B + one Arduino Due** | One micro-ROS agent, not two (X-2) |
| Gait phase | **The Due owns the phase accumulator.** The Pi sends `GaitCommand` (wave parameters) at 10 Hz; the Due computes joint angles with `gait.hpp` every 20 ms | Serial jitter and drops don't reach the servos. `ServoChainCommand` is bring-up/debug only; `bur_gait` is a bench preview only (C-2) |
| MCU loads | 4 chain servos, Xsens MTi-3 IMU, (later) Bar30 depth, fin servos | All on the Due |
| Estimation | **`robot_localization` EKF**, reconfigured, not a custom filter | Depth + IMU now, ZED odometry after P-1 |
| Servo sealing | **A covering, built by the mechanical team** | No software work |

**Arduino Due notes for firmware work:**

- micro-ROS supports the Due only through the **community**, not on the
  official `jazzy` list. Expect some build friction. The organisation forks
  `ArduinoCore-sam`, so there is history to lean on (X-5).
- **3.3 V logic, and its pins are not 5 V tolerant.** Confirm the SER0067 takes
  a 3.3 V PWM signal, and level-shift anything that drives 5 V into the Due.
- **Two USB ports** (Programming and Native) enumerate differently. Pick one for
  the micro-ROS transport and key X-2's udev rule to it.
- **Put the MTi-3 on UART or SPI, not I2C.** On I2C it would share the bus with
  the Bar30, whose blocking read (F-4) would stall IMU reads as well. And if
  two transactions interleave on a shared bus, they silently return garbage.

---

## Current focus: horizontal motion

**Decided 2026-10-31: depth and pitch are on the back burner.** Surge and yaw
need nothing from the depth stack. Surge is open-loop (speed -> tail-beat
frequency), and the yaw-rate loop closes on the IMU gyro, so an EKF fed by the
IMU alone is enough to drive them.

| In focus | Deferred (depth / pitch) |
|---|---|
| F-1, F-3, F-5, F-2 | D-3 fin count and placement |
| M-1, M-2, M-3, M-4, M-5, M-8 | F-4 Bar30 on the MCU, M-6 fluid density |
| M-7 / C-4 / C-5 (yaw rate and attitude terms) | C-8 speed-aware depth loop |
| C-1, C-2 | C-6 teleop depth setpoint |
| A-1, A-2 (horizontal leaves), A-3, A-4 | `HoldDepth` / `Surface` leaves; fin servo routing |

Deferred means not worked on, **not deleted.** The pitch/depth code paths,
`GaitCommand.fin_angle_rad` and the EKF's depth input all stay. Until fins are
routed, the controller's fin output goes nowhere, which is harmless.

### Critical path (horizontal)

```
X-5 read old firmware ───> F-1 gait firmware ─┬─> F-3 watchdog ──────────────┐
                                              ├─> M-3 plant table ──> C-1 ───┤
                                              ├─> M-4 envelope               ├─> wet test
                          F-1 + F-5 IMU ──────┴─> M-7 IMU noise ──> C-4 ─> C-5┤
D-7 drop nav2 ──> A-1 manager ──> A-2 leaves ──> A-3 load test ──────────────┘
```

**Next:** do X-5, then write F-1 and F-3 together, then F-5. Nothing blocks
them. The yaw loop needs F-5: without `/imu` there is no `/odometry/filtered`,
and the controller will not act. In parallel,
anyone can take the unblocked small tasks: D-7, X-1, X-3, X-6.

---

## How to read this

Task IDs are grouped by area: **D** decisions, **M** measurements,
**F** firmware, **C** controls, **A** autonomy, **P** perception,
**X** infrastructure. Every task states its **Input** and its **Output**.

**IDs are never reused.** Gaps are retired tasks:

| Retired | Why |
|---|---|
| D-1, D-2, D-4, D-5, D-9 | Decided; see [hardware baseline](#hardware-baseline) |
| D-8 | Became X-6 |
| C-3 | Done: pitch no longer added to chain joints; fin angle is its own field |
| C-7 | Done: `bur_gait/test/test_gait.cpp` (gtest) covers the wave maths, phase continuity and rate limit |
| C-9 | Approach decided (reconfigure `robot_localization`); remaining work is F-4, P-3, C-5 |
| X-4 | Folded into F-4 |
| X-7 | Servo sealing handed to the mechanical team |

`BLOCKED` means waiting on another task, not on effort. Don't work around a
block by guessing. The guess becomes an assumption nobody remembers making,
which is how the previous vehicle ended up with a behaviour tree that could not
load.

### Source markers

The source carries 55 `TODO(...)` markers that cross-reference these tasks:

| Marker | Count | Meaning |
|---|---|---|
| `TODO(hw)` | 31 | needs a measured number from the vehicle |
| `TODO(port)` | 13 | mechanical carry-over not finished |
| `TODO(fish)` | 8 | inherited logic that is wrong for a swimming body |
| `TODO(decide)` | 2 | open design decision |
| `TODO(verify)` | 1 | unconfirmed assumption |

```bash
grep -rn "TODO(fish)\|TODO(hw)\|TODO(decide)\|TODO(port)\|TODO(verify)" src
```

The `TODO(decide)` in `micro_ros.launch.py` (one MCU or two) is stale, because
one Due is settled. X-2 removes it.

---

## D — Decisions

### D-3 — Fin count and placement
**Deferred** with depth · **Blocks:** fin routing, C-8, `HoldDepth`

Fins with their own actuators are decided. Still open is how many fins there
are and where they sit. Things the team should know when this resumes:

- **Fin authority scales with the square of speed.** Lift is `½ρv²SC_L`, so at
  the ~0.3 m/s target a palm-sized fin gives a couple of hundred millinewtons.
  Halving speed quarters it.
- **So the vehicle must be trimmed neutrally buoyant, accurately.** Fins at this
  speed cannot overcome a ballast error. Trim matters more than fin area.
- **Pitch stability fights the fins.** A large CB-above-CM offset is a restoring
  moment the fins must overcome.

| Placement | Mechanism | Controller shape |
|---|---|---|
| Nose or tail | Pitching **moment**; the body pitches, then `surge x sin(pitch)` changes depth | Depth -> pitch -> fin cascade, **as currently coded** |
| Near centre of mass | Near-direct **lift** | Drop the pitch inner loop; depth maps to fin angle |

Each fin servo needs its own Due PWM pin and the same covering as the chain.

- **Input:** hull layout; measured buoyancy and pitch stability
- **Output:** fin count and placement, so `fin_angle_rad` can be routed to a
  servo (the `TODO(port)` in `gait_node.cpp`) and given a URDF link
- **Done when:** depth can be commanded and held at actual cruise speed

### D-6 — `bur_msgs` vendored or upstreamed
**Open, low urgency**

Currently vendored in `src/interfaces/bur_msgs` so the workspace builds offline.
A separate `bur-interfaces` repo only pays off if a shore station or second
machine needs the messages built independently.

- **Output:** either a `jazzy` branch on `bur-interfaces`, or a note here that
  vendoring is deliberate

### D-7 — Drop the nav2 dependency
**Open, recommendation is yes, unblocked** · **Blocks:** A-1

The previous `bur_autonomy` pulled in `nav2_behavior_tree`, `nav2_util` and
`nav2_common` for exactly `euclidean_distance` and `RewrittenYaml`. That is a
very large install on a Pi for a square root.

- **Output:** both inlined; nav2 absent from `package.xml`
- **Done when:** `bur_autonomy` builds without nav2 installed

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
**Blocked by:** F-1 · **Feeds:** C-1, all controller tuning

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

### M-5 — Sensor mounting offsets
**Feeds:** URDF, `bur_localization/launch/ekf.launch.py`

All current offsets come from last year's hull and are certainly wrong. The IMU
offset matters most, because a wrong yaw offset makes the heading loop chase
itself.

- **Input:** the built vehicle; calipers
- **Output:** measured `base_link` -> `imu`, `depth_sensor`, `zed_camera_link`
- **Done when:** TF matches the physical layout and the IMU is in the rigid nose

### M-6 — Pool fluid density
**Deferred** with F-4

Fresh water is ~997-1000 kg/m^3, sea water ~1025. A 3% density error is a 3%
depth error.

- **Output:** `fluid_density` set in the firmware with a comment naming the pool

### M-7 — IMU undulation noise
**Blocked by:** F-1 · **Feeds:** C-4, C-5

**The biggest unknown in estimation, and it hits yaw directly.** The undulating
body injects a periodic angular rate at the tail-beat frequency into the gyro
that the yaw-rate loop closes on.

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

Nothing exists yet; `onboard/firmware/` is empty. Last year's firmware lives in
[`ASME-BUR/bur-arduino`](https://github.com/ASME-BUR/bur-arduino). Read it
first (X-5).

### F-1 — Gait firmware
**Unblocked** (read X-5 first) · **Blocks:** M-3, M-4, M-7, everything wet

Use `src/gnc/bur_gait/include/bur_gait/gait.hpp` directly. It is ROS-free on
purpose so the **same source file** compiles into the firmware and the Pi
preview can't drift from the vehicle.

1. Get the micro-ROS hello-world running on the Due against the Pi's agent.
2. Subscribe to `GaitCommand`, run the phase accumulator locally
   (`bur_gait::advancePhase`, then `jointAngles` and `rateLimit`, exactly as
   `gait_node.cpp` does), and drive the 4 chain servos. Time one tick on the
   Due to confirm it fits comfortably inside the 20 ms frame. It should; the
   maths is tiny. Also confirm `gait.hpp` compiles with the Due toolchain
   (`std::vector`, `<cmath>`).

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
**Blocked by:** F-1 · **Priority: write it with F-1, don't defer it**

- **Input:** `GaitCommand` arrival times; `MODE_ESTOP` and `MODE_COAST`
- **Output:** chain coasts to neutral after ~300 ms of silence; slew-limited
  transitions so a mode change does not brown out the servo rail
- **Done when:** pulling the USB cable mid-swim makes the body go limp rather
  than hold a turn and swim into a wall

> A held angle on a dead link bends the body into a turn. Coast, never hold.

### F-5 — IMU firmware
**Unblocked** (read X-5 first) · **Blocks:** M-7, the yaw loop, every closed-loop test

The Xsens MTi-3 lives on the Due (hardware baseline), and the yaw-rate loop
closes on its gyro, so this is as essential as F-1 for horizontal motion. Last
year this was its own sketch, `arduino_imu/` in `bur-arduino`. It now shares
one sketch and one micro-ROS agent with the gait loop.

- **Input:** MTi-3 on a hardware UART or SPI (not I2C; see the Due notes);
  last year's `arduino_imu/`
- **Output:** `sensor_msgs/Imu` on `/imu`, `frame_id: imu`, with orientation,
  angular velocity and linear acceleration, and **diagonal** covariances.
  `ekf.yaml`'s `imu0` (rates and accelerations) and `imu1` (attitude, relative)
  already read this topic.
- **TODO(verify):** the MTi-3's output frame convention. Configure it, or
  rotate in firmware, so the data lands in the frame `ekf.yaml` and the URDF's
  `imu` link expect (M-5). Positive yaw rate must be to port.
- **Watch the shared loop:** IMU reads must not stall the 50 Hz servo tick.
  Use non-blocking serial reads, not a wait-for-packet loop.
- **Done when:** `/imu` publishes at a steady rate, rotating the vehicle by
  hand to port shows positive yaw rate in `/odometry/filtered`, and the gait
  shows no hitch while both run

### F-4 — Pressure sensor on the MCU
**Deferred** with depth

The Bar30 moves to the Due. Use BlueRobotics' upstream MS5837 **Arduino**
library rather than back-porting the Pi version in `drivers/`.

**The read blocks for ~41 ms at OSR 5,** which stalls the servo loop. That's
acceptable at low tail-beat frequency and slow depth sampling. If a joint
visibly hitches, drop the OSR (conversion wait is `2.5 x 2^(8+OSR)` us, twice
per reading: OSR 2 is ~5 ms, OSR 0 is ~1.3 ms), and average several low-OSR
reads between servo updates.

**Fix the inherited model bug:** bur-onboard called `setModel(1)`
(`MS5837_02BA`) right after `init()`, overriding the auto-detected model. A
Bar30 is a 30BA part. Check what detection reports and validate against a tape
measure.

- **Input:** MS5837 Arduino library; the Due's I2C bus
- **Output:** `nav_msgs/Odometry` on `/depth_sensor` (what `ekf.yaml`'s `odom0`
  already expects) with a **diagonal** covariance
- **Done when:** depth appears in `/odometry/filtered` and the gait shows no
  visible hitch

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
**Blocked by:** F-1

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
  swimming

### C-5 — Retune the EKF
**Blocked by:** C-4

Every covariance in `ekf.yaml` is inherited from a vehicle with completely
different dynamics. Keep the `imu0`/`imu1` split: one entry for rates and
accelerations, one in `relative` mode for attitude. It is deliberate.

- **Input:** tank recordings; C-4's filtering
- **Output:** retuned `process_noise_covariance` and measurement variances
- **Done when:** estimates are stable and converge at a sensible rate

### C-6 — Teleop depth setpoint from odometry
**Small, self-contained** · **Deferred** with depth

`joy_to_swim.py` resets its depth setpoint to `0.0` on disarm, which is only
correct at the surface.

- **Output:** setpoint initialised from `/odometry/filtered` on arm
- **Done when:** re-arming at depth does not command a surface run

### C-8 — Make the depth loop speed-aware
**Confirmed defect** · **Deferred** · **Blocked by:** D-3 · `controller_node.cpp:176`

The depth loop has no idea of forward speed, but fin authority goes as `v²`.
At low speed it winds up against an error it cannot affect. With
extremity-mounted fins (the current cascade), vertical rate also goes as
`surge x sin(pitch)`, so at zero surge it pitches the nose down and goes
nowhere.

- **Input:** D-3's placement; `state_.twist.twist.linear.x`; M-3's table
- **Output:** a depth loop that cannot demand depth change it has no authority
  for, and does not wind up when it has none
- **Done when:** commanding a depth change at zero surge either refuses or
  commands forward motion, but does not silently pitch and stall

---

## A — Autonomy

### A-1 — Manager node
**Blocked by:** D-7 · **Blocks:** A-2, A-3

Not ported on purpose. The previous node emitted a goal **pose**, assuming the
vehicle could translate in any direction and hold station.

- **Input:** `/odometry/filtered`, `/vision` (`CVDetections`), a behaviour tree
  XML
- **Output:** `bur_msgs/SwimCommand` on `/swim_command`; BehaviorTree.CPP v4
  tree ticked at 10 Hz
- **Done when:** `fish_tree.xml` loads, ticks, and drives the vehicle in the
  bench preview

Avoid the old bug: the old manager started its tick timer in the constructor
but built the tree later in `main()`.

### A-2 — Behaviour tree leaf nodes
**Blocked by:** A-1

`behavior_trees/fish_tree.xml` names the target leaf set. There is deliberately
**no `GoToPose`**: reaching a point means turning to face it and then swimming.

| Leaf | Input | Output |
|---|---|---|
| `TurnToHeading` | heading, or a detection label | `SwimCommand.yaw_rate_rps`; SUCCESS when aligned |
| `SwimForward` | distance, speed | `SwimCommand.surge_mps`; SUCCESS on odometry delta |
| `SearchPattern` | sweep angle, timeout | alternating yaw rate; SUCCESS on detection |
| `IsDetected` | label | condition on `/vision` |
| `HoldDepth` *(deferred)* | target depth, tolerance, timeout | `SwimCommand.depth_m`; SUCCESS inside tolerance |
| `Surface` *(deferred)* | — | depth 0, zero surge; SUCCESS at the surface |

Declare and read parameters from one list. Last year `params.yaml` declared
`gate_target`, the node read `gate_position`, and the YAML was silently ignored.

- **Done when:** every node referenced by any XML in `behavior_trees/` is
  registered with the factory

### A-3 — Tree load test
**Blocked by:** A-2 · **Small, high value**

The previous `comp_tree.xml` could never load. It referenced four node types
nobody had registered, and nothing caught it.

- **Input:** every XML in `behavior_trees/`
- **Output:** a test that loads each one against the real factory
- **Done when:** the build fails if a tree references an unregistered node

### A-4 — Curvature-aware planner
**Blocked by:** A-2

The old `bur_planner` ignored turning radius and pushed waypoints sideways. That
is the wrong shape for a vehicle that cannot strafe.

- **Input:** current pose, goal pose, minimum turning radius (from M-3),
  obstacles
- **Output:** a path the vehicle can follow. Dubins arcs are the cheap,
  correct answer.
- **Done when:** a planned path never asks for a turn tighter than the vehicle
  can execute

---

## P — Perception

Nothing in this workspace. The ZED and YOLO run on the Jetson, and their code
lives in [`ASME-BUR/bur-vision-nano`](https://github.com/ASME-BUR/bur-vision-nano)
and [`ASME-BUR/bur-vision`](https://github.com/ASME-BUR/bur-vision). The Jetson
stays off the ROS graph until P-1, because Humble<->Jazzy DDS is unsupported.

### P-1 — Jetson to JetPack 7.2
**Blocks:** P-2, P-3, P-4

JetPack 6.2.1 is Ubuntu 22.04 with no Jazzy packages. JetPack 7.2 (Ubuntu
24.04) covers Orin and installs Jazzy natively. ZED SDK 5.5 and the ZED ROS 2
wrapper support it.

**Install to a spare microSD and keep the 6.2.1 card.** That makes an
irreversible reflash a thirty-second swap.

> The risk is not ROS. JetPack 7.2 has no official PyTorch wheel, doesn't
> ship cuDNN/NCCL/cuSPARSELt, and moves CUDA 12.6 to 13.2, which invalidates
> every cached TensorRT engine. Spike it before committing.

- **Output:** Jetson on JetPack 7.2 running Jazzy, ZED SDK 5.5 installed
- **Done when:** the ZED streams and `ros2 topic list` shows the wrapper's topics

### P-2 — Re-export the YOLO model
**Blocked by:** P-1

- **Output:** a working TensorRT engine on CUDA 13.2, and a documented export
  command
- **Done when:** inference runs at a usable frame rate

### P-3 — Re-enable ZED odometry in the EKF
**Blocked by:** P-1

The `odom1` block in `ekf.yaml` is commented out and ready.

- **Output:** `odom1` uncommented; `ROS_DOMAIN_ID` pinned; multicast confirmed
  across the tether
- **Done when:** x and y stop drifting in `/odometry/filtered`

### P-4 — Close the detection loop
**Blocked by:** A-2, P-1

In the previous stack, detections were stored in a member nothing read, and the
gate target came from a static parameter. **The camera could not influence
behaviour at all.**

- **Output:** `/vision` detections driving `TurnToHeading` and `IsDetected`
- **Done when:** moving the physical target changes what the vehicle does

---

## X — Infrastructure

### X-1 — Topic viewer
**Small, unblocked**

A one-screen TUI in `bur_bringup` showing `/swim_command`, `/gait_command`,
`/servo_chain_state` and `/odometry/filtered`, replacing bur-onboard's
thruster-era `topic_viewer.py`.

- **Done when:** someone on the pool deck can see what the vehicle thinks it is
  doing without four `ros2 topic echo` windows

### X-2 — Single micro-ROS agent and udev rule
**Small, unblocked**

`micro_ros.launch.py` still starts two agents (`ttyACM0` servo, `ttyACM1` IMU)
from the two-Arduino era, and `/dev/ttyACM*` numbering is not stable across
reboots.

- **Input:** the Due's USB serial number, for whichever USB port F-1 uses
- **Output:** one agent on `/dev/bur_mcu`; a udev rule for it; the stale
  `TODO(decide)` removed
- **Done when:** the launch matches the hardware and survives a reboot and replug

### X-3 — RViz config
**Small, unblocked**

`bench.launch.py` opens RViz with no configuration, so it shows nothing useful.

- **Output:** a `.rviz` file with RobotModel and TF, installed and referenced by
  the launch file
- **Done when:** `bench.launch.py` opens on a visible, animating vehicle

### X-5 — Read last year's firmware
**Small, unblocked. Do this before F-1.**

[`ASME-BUR/bur-arduino`](https://github.com/ASME-BUR/bur-arduino) has
`arduino_imu/`, `arduino_merge/` and `arduino_thruster/`.

- **Output:** a short summary of what is reusable: micro-ROS setup on the Due
  core, serial transport and baud rate, MTi-3 driver, watchdog behaviour (if
  any), and which board it targeted
- **Done when:** F-1 builds on it instead of guessing

### X-6 — Delete the unused `Command.msg`
**Small, unblocked**

Nothing consumes it, and it still carries a sway field the vehicle cannot
execute.

- **Output:** `Command.msg` removed from `msg/` and from
  `rosidl_generate_interfaces` in `bur_msgs/CMakeLists.txt`
- **Done when:** the workspace builds without it

> `ServoChainState`, `CVDetection` and `CVDetections` also have no consumers
> yet, but they are the contracts for F-2, P-4 and X-1. Keep them.

---

## Appendix: lessons from the previous vehicle

1. **A behaviour tree that references unregistered nodes fails silently at
   load.** That is why A-3 exists.
2. **Losing the actuation node did not stop the actuators.** F-3 is the fix
   here, and it is the one task on this list that can damage hardware if
   skipped.
3. **Covariance is easy to get wrong in a way nothing notices.** The old Bar30
   node filled all 36 entries with the same value, which is singular. Use a
   diagonal (F-4).
4. **The same constant disagreed in three places.** `max_torque` was 4000.0,
   40.0 and 8.0 depending on where you looked. Keep one source of truth per
   number.
5. **Porting more from bur-onboard:** `tf2_geometry_msgs.h` was removed in
   Jazzy. Use `.hpp`.
