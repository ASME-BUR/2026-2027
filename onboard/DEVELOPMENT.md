# DEVELOPMENT

What still needs building, in dependency order, with the inputs and outputs of
each task. Finished and settled work is not listed here. It shows up in the
[hardware baseline](#hardware-baseline) and in the code.

The work is split into three files:

- **this file:** state, baseline, decisions, autonomy, deferred depth work;
- **[tail_dev.md](tail_dev.md):** horizontal tail motion (paused);
- **[sensors_dev.md](sensors_dev.md):** sensors and state estimation (current
  focus).

For setup, structure and how to run things, see [README.md](README.md).

---

## Current state

| | |
|---|---|
| Builds on Jazzy | **yes**: 10 packages, 0 errors, 0 warnings |
| Control chain verified in sim | **yes**: `SwimCommand` -> `GaitCommand` -> joint angles |
| Unit tests | **pass**: `bur_gait` gtest (15) + `bur_msgs` lint, on Humble / 22.04, 2026-10-31; `bur_mcu` chain logic (13, `pio test -e native`), 2026-11-07 |
| Dry bench | **runs** on Humble / 22.04 (`bench.launch.py`, RViz opens), 2026-10-31 |
| Run on hardware | **no** |
| Gains, dimensions, limits | **all placeholder** |
| MCU firmware | **written, compiles for the Due (Jazzy micro-ROS), never run on a board**: [`firmware/bur_mcu`](firmware/bur_mcu/README.md), F-1 + F-3 + F-5, 2026-11-07 |
| Autonomy node | **written; runs closed-loop against `bench_sim` on Humble**: [`bur_autonomy`](src/gnc/bur_autonomy/README.md), A-1 + A-2 + A-3, 2026-11-07. The RViz bench run and a Jazzy build are still to do |
| Tail hardware | **not wired**: no chain servos on the Due yet, so tail_dev is paused (2026-11-14) |
| Decisions outstanding | D-6; D-3 fin layout (deferred); mission start/kill switch and the detection contract (D-A1, D-A4 in [`bur_autonomy/PLAN.md`](src/gnc/bur_autonomy/PLAN.md#7-decisions-in-this-plan)) |
| **Current focus** | **sensors and state estimation: IMU + ZED visual odometry, fused on the Pi** ([sensors_dev.md](sensors_dev.md)) |

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
| IMU bus | **MTi-3 on I2C `Wire1`** (SDA1/SCL1, DRDY on D3); Bar30 later on `Wire` | Separate buses, so the Bar30's blocking read cannot stall the IMU. Reuses last year's driver. Decided 2026-11-07 |
| Estimation | **`robot_localization` EKF**, reconfigured, not a custom filter | IMU now; ZED velocity over the bridge (P-3); depth after F-4 |
| Jetson | **Stays on JetPack 6.2.1 / Humble.** Decided 2026-11-14 | Never on the Pi's ROS graph. ZED odometry and detections reach the Pi over a UDP bridge (P-5, P-6) |
| Servo sealing | **A covering, built by the mechanical team** | No software work |

**Arduino Due notes for firmware work:**

- micro-ROS supports the Due only through the **community**, not on the
  official `jazzy` list. Expect some build friction. The organisation forks
  `ArduinoCore-sam`, so there is history to lean on (X-5).
- **3.3 V logic, and its pins are not 5 V tolerant.** Confirm the SER0067 takes
  a 3.3 V PWM signal, and level-shift anything that drives 5 V into the Due.
- **Two USB ports** (Programming and Native) enumerate differently. Pick one for
  the micro-ROS transport and key X-2's udev rule to it.
- **Never put the MTi-3 and the Bar30 on the same I2C bus.** The Bar30's
  blocking read (F-4) would stall IMU reads, and two interleaved
  transactions on a shared bus silently return garbage. The MTi is on `Wire1`
  and the Bar30 goes on `Wire` (hardware baseline). `Wire1` has no on-board
  pull-ups.

---

## Task index

### Sensors and state estimation: [sensors_dev.md](sensors_dev.md)

> **Current focus.** Bring the MTi-3 up on the Due with no servos, carry ZED
> visual odometry from the Humble Jetson to the Pi over a UDP bridge, and fuse
> both in the existing `robot_localization` EKF.
>
> | Task | Status |
> |---|---|
> | F-5 IMU firmware, brought up alone | written, never run on a board |
> | X-2 single micro-ROS agent + udev | unblocked |
> | X-8 Pi-Jetson network, clock, graph isolation | unblocked |
> | X-9 one owner per sensor TF | unblocked, confirmed defect |
> | M-5 sensor mounting offsets | needs the nose or the rig |
> | P-5 UDP odometry bridge | unblocked |
> | P-3 ZED velocity in the EKF | blocked by P-5 |
> | C-5 EKF retune (dry, then wet) | blocked by P-3; wet by C-4 |
> | P-6 detections over the bridge | blocked by D-A4 |
> | P-4 detections drive behaviour | blocked by P-6 |
>
> **Done when:** the hand-carried rig closes a 4 m square within ~0.5 m, and
> losing the Jetson link degrades to IMU-only without the EKF diverging.

### Horizontal tail motion (surge + yaw): [tail_dev.md](tail_dev.md)

> **Paused** until the chain servos are wired to the Due. Only C-2 can move
> without hardware.
>
> | Task | Status |
> |---|---|
> | F-1 gait firmware | written, never run on a board |
> | F-3 watchdog and failsafe | written, never run on a board |
> | F-2 servo telemetry | blocked by F-1 |
> | M-1, M-2, M-8 body, servo and joint-limit measurements | need the built tail |
> | M-3 plant table, M-4 envelope, M-7 IMU undulation noise | blocked by F-1 |
> | C-1 plant-table surge, C-4 undulation filter | blocked by M-3, M-7 |
> | C-2 prevent double-driving | unblocked |
>
> **Done when:** a wet test tracks commanded surge and yaw rate, and the body
> goes limp on link loss.

### Everything else: this file

| Area | Tasks |
|---|---|
| Decisions | D-3 (deferred), D-6 |
| Autonomy | A-1 (RViz run left), A-4 |
| Infrastructure | X-1 |
| Deferred with depth | M-6, F-4, C-6, C-8 |

---

## Deferred: depth and pitch

**Decided 2026-10-31: depth and pitch are on the back burner.** Deferred means
not worked on, **not deleted.** The pitch/depth code paths,
`GaitCommand.fin_angle_rad` and the EKF's depth input all stay. Until fins are
routed, the controller's fin output goes nowhere, which is harmless. Deferred
tasks: D-3 fin layout, M-6, F-4, C-6, C-8, the `HoldDepth` / `Surface` leaves,
and fin servo routing.

---

## How to read this

Task IDs are grouped by area: **D** decisions, **M** measurements,
**F** firmware, **C** controls, **A** autonomy, **P** perception,
**X** infrastructure. Every task states its **Input** and its **Output**.
IDs are shared across this file, [tail_dev.md](tail_dev.md) and
[sensors_dev.md](sensors_dev.md); the [task index](#task-index) says where
each one lives.

**IDs are never reused.** Gaps are retired tasks:

| Retired | Why |
|---|---|
| D-1, D-2, D-4, D-5, D-9 | Decided; see [hardware baseline](#hardware-baseline) |
| D-7 | Done 2026-11-07: nav2 dropped. `euclidean_distance` -> `bur_autonomy/geometry.hpp` (gtest); `RewrittenYaml` -> launch_ros parameter overrides. `bur_autonomy` builds and tests on Jazzy with no nav2 installed; its CMakeLists rejects a nav2 dependency |
| D-8 | Became X-6 |
| C-3 | Done: pitch no longer added to chain joints; fin angle is its own field |
| C-7 | Done: `bur_gait/test/test_gait.cpp` (gtest) covers the wave maths, phase continuity and rate limit |
| C-9 | Approach decided (reconfigure `robot_localization`); remaining work is F-4, P-3, C-5 |
| X-3 | Done: `bur_bringup/rviz/bench.rviz` (RobotModel + TF). `bench.launch.py use_controller:=false` lets a hand-published `GaitCommand` animate it |
| X-4 | Folded into F-4 |
| X-5 | Done 2026-11-07: [`firmware/X5_bur_arduino_review.md`](firmware/X5_bur_arduino_review.md) (what to reuse from `bur-arduino`, and what not to) |
| X-6 | Done: `Command.msg` deleted. `ServoChainState` and `CVDetection(s)` have no consumers yet but are the contracts for F-2, P-4, X-1: keep them |
| X-7 | Servo sealing handed to the mechanical team |
| P-1 | Dropped 2026-11-14: the Jetson stays on JetPack 6.2.1 / Humble and is bridged (P-5). JetPack 7.2 does support the Orin Nano, but the reflash and CUDA 13 rebuild were not worth the risk mid-season |
| P-2 | Dropped with P-1: no CUDA change, so the YOLO engine needs no re-export |

`BLOCKED` means waiting on another task, not on effort. Don't work around a
block by guessing. The guess becomes an assumption nobody remembers making,
which is how the previous vehicle ended up with a behaviour tree that could not
load.

### Source markers

The source carries 72 `TODO(...)` markers that cross-reference these tasks
(64 in `src`, 8 in `firmware/bur_mcu`):

| Marker | Count | Meaning |
|---|---|---|
| `TODO(hw)` | 46 | needs a measured number from the vehicle |
| `TODO(port)` | 10 | mechanical carry-over not finished |
| `TODO(fish)` | 9 | inherited logic that is wrong for a swimming body |
| `TODO(decide)` | 4 | open design decision |
| `TODO(verify)` | 3 | unconfirmed assumption |

```bash
grep -rn "TODO(fish)\|TODO(hw)\|TODO(decide)\|TODO(port)\|TODO(verify)" src firmware/bur_mcu --exclude=*.md
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

---

## Deferred depth tasks

### M-6 — Pool fluid density
**Deferred** with F-4

Fresh water is ~997-1000 kg/m^3, sea water ~1025. A 3% density error is a 3%
depth error.

- **Output:** `fluid_density` set in the firmware with a comment naming the pool

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
**Implemented 2026-11-07; the RViz bench run is still to do.** See
[`src/gnc/bur_autonomy`](src/gnc/bur_autonomy/README.md). The design was
reviewed and approved by an independent reviewer:
[`PLAN.md`](src/gnc/bur_autonomy/PLAN.md).

The previous node emitted a goal **pose**, which assumed the vehicle could
translate in any direction and hold station. It was not ported.

- **Input:** `/odometry/filtered`, `/vision` (`bur_msgs/CVDetections`), a
  behaviour tree XML
- **Output:** `bur_msgs/SwimCommand` on `/swim_command`, from a
  BehaviorTree.CPP v4 tree ticked at 10 Hz; `~/status`; `~/start` and
  `~/stop`
- **Done when** (changed 2026-11-07, decision D-A2 in PLAN.md):
  - `fish_tree_horizontal.xml` loads, ticks, and drives the vehicle in the
    bench preview;
  - `fish_tree.xml` loads.
  - The original criterion, "`fish_tree.xml` drives the vehicle", cannot be
    met while its depth leaves are deferred.
- **Status:**
  - The whole closed loop runs headless on Humble: manager -> `bur_controller`
    -> `bench_sim` -> manager. The mission waits for odometry, searches,
    turns, swims 4 m, and reaches SUCCESS.
  - Still to do: `ros2 launch bur_bringup bench.launch.py use_autonomy:=true`
    with RViz, on a machine with `xacro`.
  - Not built on Jazzy: Jazzy has no BehaviorTree.CPP package installed here,
    and building it from source was not permitted in that session.

The old manager's bug is fixed: the tree is built before the tick timer
exists. The manager never calls `rclcpp::shutdown()`. It is the only guard
against stale odometry, because `bur_controller`'s `have_state_` never goes
false.

### A-2 — Behaviour tree leaf nodes
**Done 2026-11-07** (`include/bur_autonomy/leaves.hpp`; 23 leaf tests).

There is deliberately **no `GoToPose`**: to reach a point, the vehicle turns
to face it and then swims.

| Leaf | Input | Output |
|---|---|---|
| `TurnToHeading` | `heading_rad` or a detection `label` | yaw rate, plus `turn_surge_mps` (at zero surge the controller holds a static bend); SUCCESS when settled |
| `SwimForward` | distance, speed | surge, holding the start heading; SUCCESS on active time (`timed`, until P-3) or odometry distance |
| `SearchPattern` | `label`, sweep angle, timeout | alternating yaw; SUCCESS on detection |
| `IsDetected` | label | condition on a fresh sighting |
| `HoldDepth`, `Surface` | — | **Placeholders that always fail** (`TODO(fish)`). Blocked by D-3 / C-8 / F-4 |

- **Parameters:** they are declared and read from one table, and an unknown
  key in the YAML stops the node at startup. (Last year, `params.yaml`
  declared `gate_target`, the node read `gate_position`, and the YAML value
  was silently ignored.)
- **Done when** (changed 2026-11-07): every node referenced by any XML in
  `behavior_trees/` is registered with the factory, **excluding
  `behavior_trees/reference/`**. Last year's unloadable trees were moved
  there; they are kept, not deleted.

### A-3 — Tree load test
**Done 2026-11-07** (`test/test_tree_loading.cpp`).

The previous `comp_tree.xml` could never load. It referenced four node types
nobody had registered, and nothing caught it.

- **Input:** every top-level XML in `behavior_trees/`
- **Output:** a test that builds each tree through the same
  `buildMissionTree()` the manager uses. Unregistered nodes, undeclared
  attributes and missing required ports all fail it.
  `test/test_mission_sim.cpp` also runs the horizontal mission closed-loop.
- **Done when:** a tree that references an unregistered node fails
  `colcon test`. ("The build fails" is read as the test failing.)

### A-4 — Curvature-aware planner
**Outline only** (PLAN.md section 6). In practice it is blocked on M-3
(minimum turning radius) and P-3 (usable x/y).

The old `bur_planner` ignored turning radius and pushed waypoints sideways. That
is the wrong shape for a vehicle that cannot strafe.

- **Input:** current pose, goal pose, minimum turning radius (from M-3),
  obstacles
- **Output:** a path the vehicle can follow. Dubins arcs are the cheap,
  correct answer.
- **Done when:** a planned path never asks for a turn tighter than the vehicle
  can execute

---

## X — Infrastructure

### X-1 — Topic viewer
**Small, unblocked**

A one-screen TUI in `bur_bringup` showing `/swim_command`, `/gait_command`,
`/servo_chain_state` and `/odometry/filtered`, replacing bur-onboard's
thruster-era `topic_viewer.py`.

- **Done when:** someone on the pool deck can see what the vehicle thinks it is
  doing without four `ros2 topic echo` windows

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
