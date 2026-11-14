# Autonomy implementation plan (A-1, A-2, A-3; A-4 outline)

Status: **revision 3, approved by the independent reviewer and IMPLEMENTED (2026-11-07).** The verification state and one deviation are in [section 13](#13-implementation-record). The four review notes in section 12 are folded in. Revision 2
was reviewed "APPROVE WITH CHANGES" (1 blocking item: subtree blackboard
isolation). The answers are in [section 11](#11-review-responses-revision-3).

Earlier history: Revision 1
was written 2026-11-07 and reviewed the same day ("APPROVE WITH CHANGES": 1
blocking item, 11 non-blocking). Every review item is answered in
[section 10](#10-review-responses-revision-2).

This covers the `A` tasks in [`DEVELOPMENT.md`](../../../DEVELOPMENT.md). A-1
became unblocked when D-7 was done. P-4 is out of scope: it is blocked on P-6 (was P-1; see sensors_dev.md).

## 0. Constraints this plan has to respect

| Constraint | Source | Consequence |
|---|---|---|
| The vehicle cannot move sideways and cannot hold station. There is no sway and no `GoToPose` | CLAUDE.md, `SwimCommand.msg` | Leaves output only forward speed (surge) and turn rate. To reach a point, the vehicle turns, then swims |
| The controller coasts if `/swim_command` is older than 0.5 s, or if `/odometry/filtered` has never arrived. `have_state_` never becomes false again, so **the controller has no guard against odometry that goes stale later** | `controller_node.cpp:103-104, 124-127`; `controller.yaml` `cmd_timeout` | The manager publishes every tick (10 Hz) while it is active, and its own odometry-staleness check is the only guard. The code will say so |
| The controller takes `abs(surge_mps)` | `controller_node.cpp:158` | There is no reverse. Leaves never command negative surge |
| When `freq <= freq_min`, the controller sends `MODE_HOLD`, which holds a static bend and produces no thrust | `controller_node.cpp:190` | Turning needs forward speed: `turn_surge_mps * surge_to_freq` must be greater than `freq_min`. That ties two packages together, so it is documented in both config files |
| The EKF uses only the IMU until P-3: x/y drift, and yaw is relative (`imu1_relative: true`) | `ekf.yaml` | Odometry distance can't be trusted on the vehicle yet. Heading 0 is the heading at power-on |
| **Depth is deferred, and D-3 blocks `HoldDepth` and C-8.** Commanding depth at zero surge is C-8's confirmed defect. z is meaningless until F-4 | DEVELOPMENT.md:156, C-8 | `HoldDepth` and `Surface` are **registered placeholders** that fail with a message. They contain no depth logic. See D-A2 |
| The old manager ticked before its tree existed, and shut itself down when the mission ended | old `manager_node.cpp:130`, A-1 note | Build the tree before creating the timer. Never call shutdown |
| The old params.yaml declared `gate_target` while the code read `gate_position`, so the setting was silently ignored | A-2 note | One parameter table, and the node refuses unknown overrides |
| The old `comp_tree.xml` could not load | A-3 | Every tree is loaded against the real factory in a test |
| There is no detection contract. `yolo.py` publishes **`yolo_msgs/CVDetections`** (not `bur_msgs`) on `yolo_detections`, at 2 Hz plus inference time. It puts the pixel centre in `bbox.pose.position`, the box size in `orientation.x/y`, and sets no header, width, height or depth. `bur-vision-nano` publishes nothing to ROS | read 2026-11-07 | Turning a detection into a bearing goes through one adapter function with an assumed contract, marked `TODO(decide)` for P-4 (D-A4) |

## 1. Package layout (all under `src/gnc/bur_autonomy`)

```
include/bur_autonomy/
  geometry.hpp        (exists, D-7) + wrapAngle(), headingToYawRate()
  mission_context.hpp ROS-free shared state that the leaves read and write
  leaves.hpp          leaf classes (BT.CPP v4), ROS-free
  register.hpp        registerLeaves(factory, ctx): the ONE registration function
  detection.hpp       detectionBearing(): the assumed CVDetection contract
  sim/unicycle.hpp    kinematic model for the tests and the bench sim
src/
  leaves.cpp
  manager_node.cpp    ROS wrapper: subscriptions, parameters, tick timer, SwimCommand
  bench_sim_node.cpp  bench only: GaitCommand -> fake /odometry/filtered, TF, /vision
behavior_trees/
  fish_tree.xml              full target mission; its depth leaves are placeholders
  fish_tree_horizontal.xml   NEW: gate task only, the tree launched now
  reference/comp_tree.xml, reference/qualification_tree.xml   moved here, never loaded
config/autonomy.yaml
launch/autonomy.launch.py
test/ test_geometry.cpp (exists), test_leaves.cpp, test_tree_loading.cpp, test_mission_sim.cpp
```

**Why the leaves don't use ROS.** They depend only on BT.CPP and
`MissionContext`; all ROS input and output stays in `manager_node.cpp`. That
way every leaf, every tree load and a whole simulated mission can be gtested
with a fake clock. It is the same pattern as `gait.hpp` and the firmware's
`chain_logic.hpp`.

### `MissionContext` (one instance, owned by the manager, shared with the leaves)

```cpp
struct Detection { int label; double conf; double bearing_rad; double stamp_s;
                   double yaw_at_receipt; };  // vehicle yaw when it arrived (review #4)
struct MissionContext {
  // inputs, written by the manager's callbacks
  double now_s;                          // set before every tick (a fake clock in tests)
  double tick_dt_s;                      // time since the last tick, clamped to 2 x the period (review #6)
  bool   have_odom; double odom_stamp_s;
  double x, y, z, yaw;                   // odom frame; yaw comes from the quaternion (tf2, in the node)
  std::vector<Detection> detections;
  // outputs, written by leaves and read by the manager when it publishes
  double surge_mps = 0, yaw_rate_rps = 0;
  bool   enable = false;
  // depth_m and pitch_rad are always published as 0: the depth paths stay, unused (D-A2)
  LeafConfig cfg;   // heading_kp, max_yaw_rate_rps, turn_surge_mps, detection_max_age_s,
                    // detection_lost_s, swim_forward_mode, ... (all from the parameter table)
};
```

**How the outputs reach the controller.**

- Each tick, the manager clears the outputs, sets `enable = false`, ticks the
  tree, then publishes. A leaf that stops running therefore stops
  commanding motion.
- In these trees at most one action leaf runs at a time. The tree files must
  never put action leaves under a `Parallel` node; a comment in each XML will
  say so.
- **Accepted, and documented in code:** on a tick where
  `RetryUntilSuccessful` starts its next attempt and no leaf is active yet,
  the manager publishes a single coast tick. The controller then resets its
  PID controllers. The effect is a 100 ms coast, which is harmless (review
  #11).

**Leaf timers count only time the leaf actually runs** (review #6).

- Leaf timers (timed distance, timeouts, settle time) add up `tick_dt_s` on
  each tick the leaf runs. They do not use the wall clock since `onStart`.
- While the manager is paused, it doesn't tick, so no time accumulates: a
  timed `SwimForward` never counts coasting as distance swum.
- `tick_dt_s` is clamped, so the first tick after a pause adds at most one
  normal period.

## 2. A-1: the manager node

`manager_node.cpp`, node name `bur_autonomy`.

- **Subscribes** to `/odometry/filtered` (nav_msgs/Odometry) and `/vision`
  (bur_msgs/CVDetections); both topic names are parameters.
- **Publishes** `/swim_command` (bur_msgs/SwimCommand) while it is active,
  and `~/status` (std_msgs/String: tree status and the running leaf) for X-1.
- **Construction order:**
  1. Declare and read the parameters.
  2. Create the context and the factory, call `registerLeaves`, then
     `createTreeFromFile`. Nothing is written to any blackboard: every
     setting reaches the leaves through `ctx.cfg` (rev 3, item 1).
  3. Create the subscriptions and services.
  4. Create the timer, last.

  `test_mission_sim` builds the tree with the **same** function the manager
  uses, `buildMissionTree(xml, ctx)`, so the test and the node cannot
  differ.

  If the tree fails to load, throw from the constructor and exit with a
  non-zero code.
- **Each tick (10 Hz, single-threaded executor):**
  1. Set `ctx.now_s` and `ctx.tick_dt_s`.
  2. **Idle** (not started yet, or the mission has ended): see the idle
     behaviour below.
  3. **Odometry missing or older than `odom_timeout_s` (0.5 s):**
     - publish `enable=false`, and do not tick the tree;
     - after `odom_loss_abort_s` (2 s) of this, `haltTree()` and end the
       mission as FAILURE.
  4. Clear the outputs, call `tree.tickOnce()` inside a try/catch, and
     publish the SwimCommand.
     - A `BT::RuntimeError` from a leaf (for example a missing required
       port) is logged and ends the mission as FAILURE (review #12).
  5. On SUCCESS or FAILURE: log it, halt the tree, and go idle.
     **`rclcpp::shutdown()` is never called.**
- **Idle behaviour (review #7):** after a stop or the end of a mission,
  publish `enable=false` for `idle_coast_s` (1 s), then **stop publishing**.
  The controller's 0.5 s timeout keeps the vehicle coasting, and a teleop
  node started by hand is not drowned out. Before the mission starts, the
  manager publishes nothing at all.
- **Parameters:** one static table of `{name, default, description}` drives
  both the declarations and the reads.
  - The node is created with
    `automatically_declare_parameters_from_overrides(false)`.
  - At startup, it compares
    `get_node_parameters_interface()->get_parameter_overrides()`, which does
    include values from `--params-file`, against the table plus a whitelist
    (`use_sim_time`, `qos_overrides.*`). **It throws on any unknown key**
    (review #9).
  - Leaf parameters live in XML ports. BT.CPP v4 rejects undeclared
    attributes when the tree loads. A *missing* required input fails only at
    runtime, so the leaves throw a clear `BT::RuntimeError`, and a test
    covers it.
- **Start and stop:** see D-A1.

**Done-when (changed; see D-A2 and review #2).** The changed definition
replaces the line in DEVELOPMENT.md: `fish_tree_horizontal.xml` loads, ticks,
and drives the vehicle in the bench preview, and `fish_tree.xml` loads. Step
7 records the change in DEVELOPMENT.md as a decision.

## 3. A-2: the leaves

**Shared helpers** go in `geometry.hpp`: `wrapAngle()`, which maps an angle
into (-π, π], and `headingToYawRate(err, kp, max)`.

- Gains and limits come from `ctx.cfg`, which is filled from the parameter
  table.
- The XML carries only the values that change from one use to the next:
  targets, tolerances and timeouts.

| Leaf | Type | Ports | Behaviour | SUCCESS | FAILURE |
|---|---|---|---|---|---|
| `TurnToHeading` | StatefulAction | `source` (`absolute` or `detection`), `heading_rad`, `label`, `tolerance_rad`, `timeout_s` (20) | Yaw rate = clamp(kp·wrap(target − yaw)); surge = `turn_surge_mps`. With `source=detection`, **target = `yaw_at_receipt` + `bearing`** of the freshest matching detection (review #4), so a 0.5 s-old bearing is not added to today's yaw | \|error\| < tolerance for `settle_ticks` (3) ticks in a row | timeout, or `source=detection` with no fresh detection for `detection_lost_s` |
| `SwimForward` | StatefulAction | `distance_m`, `speed_mps`, `timeout_s`, optional `mode` (`timed` or `odometry`; if absent, `ctx.cfg.swim_forward_mode` is used) | Surge = speed; holds the heading captured at `onStart` with the same proportional law | `timed`: accumulated active time ≥ distance/speed. `odometry`: `horizontalDistance(start, now)` ≥ distance | timeout (default 2× the expected time) |
| `SearchPattern` | StatefulAction | `label` (**new port**), `sweep_rad`, `timeout_s` | Turns to start + sweep/2, then start − sweep/2, alternating, at `turn_surge_mps` | a fresh detection of `label` | timeout |
| `IsDetected` | Condition | `label`, `min_conf` | — | a matching detection younger than `detection_max_age_s` | otherwise |
| `HoldDepth` | **placeholder** (TODO(fish), D-3/C-8) | `depth_m`, `tolerance_m`, `timeout_s` (same as the XML today) | logs "deferred: blocked by D-3 / C-8" | never | always, on its first tick |
| `Surface` | **placeholder** (TODO(fish), D-3/F-4) | `timeout_s` | same as `HoldDepth` | never | always |

**Detection timing** (review #4). `yolo.py` publishes at about 2 Hz plus
inference time, so the defaults are:

| Parameter | Default |
|---|---|
| `detection_max_age_s` | 1.5 |
| `detection_lost_s` | 2.0 |

`autonomy.yaml` explains why: at 0.5 s or less, a healthy detector would
look lost between frames.

**Invariant** (rev 3, item 2): `detection_lost_s` must be greater than
`detection_max_age_s`.

- When `TurnToHeading` fails as "lost", `IsDetected` is then guaranteed to be
  false on the retry, so `SearchPattern` runs.
- The manager rejects parameters that break this at startup, and the leaf
  comment states it.
- Detection ages use `now_s`, so they keep growing during an odometry pause.
  Failing as "lost" on the first tick after resuming is correct, and no test
  asserts otherwise (rev 3, item 7).

**Detection bearing** (`detection.hpp`, D-A4).

- `bearing = -atan((px - W/2) / f)`, where `f = (W/2) / tan(hfov/2)`.
  The sign makes a target to the left of centre give a positive bearing
  (port, NWU).
- `W` and `hfov` are `TODO(hw)` parameters.
- The contract is `TODO(decide)` for P-4. It names the message-type mismatch
  (`yolo_msgs` vs `bur_msgs`) and the unset header. The manager stamps
  detections on receipt.

**XML changes** (review #3):

- `GateTask` uses a plain **`Sequence`**, not `SequenceWithMemory`.
  - In BT.CPP v4, `SequenceWithMemory` does not reset its child index on
    FAILURE.
  - So after a lost detection, `RetryUntilSuccessful` would resume at the
    failed `TurnToHeading` instead of searching again.
  - A plain `Sequence` restarts from the first child on FAILURE. It also
    resumes a RUNNING child without re-ticking earlier children, and each
    leaf is stateful anyway.
  - `test_mission_sim` locks this behaviour in (section 4). If either BT.CPP
    version behaves differently, the test catches it.
- `fish_tree.xml` keeps its `MainTree` (`SequenceWithMemory` over HoldDepth,
  Retry(GateTask), Surface). It loads, and at runtime it fails immediately at
  the placeholder. That is honest: depth is not available.
- Add `label="0"` to `SearchPattern`.
- `SwimForward` has **no** `mode` attribute in either tree, so it takes the
  mode from `ctx.cfg` (D-A3).
  - No blackboard keys are used anywhere, which sidesteps v4's per-`SubTree`
    blackboard isolation (rev 3, item 1).
- `fish_tree.xml`'s header comment says it **fails on its first tick**,
  because of the depth placeholders, and must not be launched expecting a
  mission (rev 3, item 5).
- Add a new `fish_tree_horizontal.xml`: `MainTree` =
  `RetryUntilSuccessful(3)` over the same `GateTask`.
- Move `comp_tree.xml` and `qualification_tree.xml` to `reference/`, with a
  README note. Nothing is deleted.

## 4. A-3: the load test, plus mission tests

- **`test_tree_loading.cpp`:**
  - Loads every top-level `behavior_trees/*.xml` (found by globbing at test
    time) with the real `registerLeaves()`.
  - An unregistered node or an undeclared attribute makes it throw.
  - Sanity cases:
    - a tree with a bogus node name must fail;
    - a tree that omits a required port must fail at the first tick
      (review #12).
  - The DEVELOPMENT.md requirement "the build fails" is met by `colcon test`
    failing. That reading is noted in DEVELOPMENT.md.
- **`test_leaves.cpp`:**
  - success, timeout, settle ticks, wrap at ±π, and halt then restart;
  - active-time accounting across a pause (review #6);
  - a turn toward a detection uses `yaw_at_receipt`;
  - the placeholders always return FAILURE.
- **`test_mission_sim.cpp`** runs `fish_tree_horizontal.xml` against
  `sim/unicycle.hpp` and a fake detector. The detector inverts
  `detectionBearing`, applies the field of view, and publishes at 2 Hz with
  0.3 s of latency. Cases:
  1. The target is ahead: SUCCESS.
  2. The target is outside the initial field of view but within
     sweep/2 + HFOV/2, so the vehicle must search: SUCCESS.
     - A target outside that coverage gives FAILURE after 3 attempts.
     - The XML's sweep value is not changed to make a test pass (rev 3,
       item 3).
  3. The target is never visible: FAILURE after 3 attempts.
  4. **Seen, then lost during the turn** (review #3): the next attempt must
     run `SearchPattern` again. The test asserts the order of leaf
     activations through a test-only status hook.
  5. Odometry pauses mid-`SwimForward`: no distance is credited while
     paused (review #6).
  6. Invariants on every tick: surge ≥ 0, |yaw rate| ≤ max, and no NaN.

## 5. Bench integration

- **`bench_sim_node`** runs on the bench only, never in `vehicle.launch.py`.
  - It subscribes to `/gait_command` and integrates a unicycle model:
    - surge = freq / `surge_to_freq`;
    - yaw rate = `turn_gain` · turn_bias.
  - It publishes `/odometry/filtered` with **`twist.angular.z` filled in**,
    because the controller's yaw-rate loop reads it (review #10). It also
    publishes TF `odom`→`base_link`, and `/vision` for one configured
    target.
  - **`turn_gain` default 5.0** (rev 3, item 4).
    - With the controller's P-only `yaw_rate.kp` of 0.4, the closed loop
      reaches 2/3 of the commanded rate.
    - At `max_yaw_rate_rps` 0.4 that is about 0.27 rad/s, so a π turn takes
      about 12 s, inside `TurnToHeading`'s 20 s timeout.
    - The `turn_bias_max` cap of 0.3 allows up to 1.5 rad/s, so it never
      binds here.
    - The comment states the steady-state fraction, not "headroom".
  - The sim proves the plumbing and the logic only. Its gains are invented,
    and the node header says so.
- **`bench.launch.py`** gains `use_autonomy` (default false). When it is
  true, the launch:
  - starts `bench_sim_node`;
  - starts the manager with `autostart:=true`, the horizontal tree and
    `swim_forward_mode:=odometry`;
  - loads a **second RViz config**, `bench_autonomy.rviz`, with fixed frame
    `odom`.

  `bench.rviz` stays fixed to `base_link`, so `use_autonomy:=false` still
  works (review #10).
- **`vehicle.launch.py`** gains `use_autonomy` (default false). An
  `OpaqueFunction` makes the launch **error** if both `use_teleop` and
  `use_autonomy` are true.
- **`autonomy.launch.py`** uses
  `parameters=[config_file, {"behavior_tree": <path>}]` (D-7). Tree names
  are resolved against `share/bur_autonomy/behavior_trees`.
- **`turn_surge_mps` coupling** (review #8). Default 0.15; with
  `surge_to_freq` 2.0 that gives 0.3 Hz, above `freq_min` 0. Comments in
  `autonomy.yaml` and `controller.yaml` each point to the other.

## 6. A-4: outline only, not implemented in this pass

In practice A-4 is blocked on M-3 (the minimum turning radius) and P-3
(usable x/y). The intended design:

- a `PlanDubins` leaf that hands a path to `FollowPath` (through `MissionContext`, or a blackboard with explicit `SubTree` remapping, since v4 isolates subtree blackboards);
- a `FollowPath` leaf that tracks it with pure pursuit;
- `min_turn_radius_m` as a `TODO(hw)` parameter.

It gets its own plan and review once A-1 to A-3 run on the bench.

## 7. Decisions in this plan

- **D-A1 Mission start.** A parameter, `autostart` (default false; true on
  the bench), plus two services, `~/start` and `~/stop` (std_srvs/Trigger).
  - **`~/start`:**
    - When idle, including after a finished mission, it calls `haltTree()`
      and starts the tree fresh. Every leaf re-initialises in `onStart`
      (rev 3, item 8).
    - While a mission is active, it refuses and says so.
  - **`~/stop`** always goes idle, even while odometry is stale or missing.
  - **A physical start or kill switch** is `TODO(decide)` for the team.
    Nothing here assumes one.
  - **Teleop conflict:** the launch guard against teleop and autonomy
    together covers only `vehicle.launch.py`. A teleop node started by hand
    during an active mission still interleaves with the manager on
    `/swim_command`. The README states this.
- **D-A2 Depth leaves are placeholders** (review #1). They are registered
  with the ports in the XML, so `fish_tree.xml` loads and A-3 covers it.
  - They always return FAILURE, with a `TODO(fish)` naming D-3 and C-8 (or
    F-4).
  - There is no depth setpoint logic: `depth_m` and `pitch_rad` are
    published as 0.
  - The depth paths in the controller and in `SwimCommand` stay untouched.
  - The tree that is launched is `fish_tree_horizontal.xml`. A-1's
    done-when changes accordingly (section 2).
- **D-A3 SwimForward mode.** The manager parameter `swim_forward_mode` sets
  it, through `ctx.cfg`.
  - The default is `timed`, because odometry x/y can't be trusted until P-3.
  - The bench sets `odometry`.
  - A `mode` attribute in the XML, if one is ever added, overrides the
    parameter for that leaf.
  - In `odometry` mode, any drift while coasting during a pause counts toward
    the distance. That is accepted; only the bench uses this mode (rev 3,
    item 7).
- **D-A4 Detection contract.** Pixel x plus HFOV, in one function, marked
  `TODO(decide)` for P-4. It lists the `yolo_msgs` vs `bur_msgs` type
  mismatch, the topic name, the missing header and the 2 Hz rate.
- **D-A5 End of mission.** Coast for 1 s, then go quiet and stay alive.
  Never shut down.

## 8. Order of work and verification

1. Geometry additions, `MissionContext`, `detection.hpp` and
   `sim/unicycle.hpp`, with their gtests.
2. The leaves, `registerLeaves` and `test_leaves`.
3. The XML changes and `test_tree_loading` (A-3).
4. `test_mission_sim`.
5. `manager_node`, the config and `autonomy.launch.py` (A-1).
6. `bench_sim_node`, `bench_autonomy.rviz`, and the bench and vehicle launch
   arguments.
7. Docs:
   - DEVELOPMENT.md:
     - A-1/A-2/A-3 status.
     - **The changed A-1 done-when.**
     - **The changed A-2 done-when:** "excluding `behavior_trees/reference/`"
       (rev 3, item 6). `install()` still installs `reference/`, which is
       harmless.
     - The A-3 "build fails" reading.
     - D-A1 and D-A4 as open team decisions.
     - Marker counts, including the new `TODO(fish)` markers in the two
       placeholders (rev 3, item 5).
   - CLAUDE.md: rules for autonomy.
   - The package README.
   - A comment in `controller.yaml` about the coupling.

**Verification** (all in WSL, no sudo):

- **Ubuntu 22.04 / Humble:** its apt package is `behaviortree_cpp`
  **4.7.1**.
- **Ubuntu 24.04 / Jazzy:**
  - BehaviorTree.CPP is built from source and **pinned to 4.7.1** to match
    Humble (review #12).
  - Groot, SQLite logging, examples and tests are turned off.
- **On both:**
  - Run `colcon build --packages-up-to bur_bringup`, then
    `colcon test --packages-select bur_autonomy`.
  - Run the bench headless with RViz off. Then use
    `ros2 topic echo` and `ros2 topic hz` to check that `/swim_command` and
    `/odometry/filtered` change over time and that `~/status` reaches
    SUCCESS.

Nothing that needs real odometry, real detections or water can be verified
here.

## 9. Risks

- **The bench sim's gains are invented.** It proves plumbing and logic, not
  performance.
- **Timed `SwimForward` and relative heading are crude.** They are isolated
  behind a port and a parameter until P-3 and M-3.
- **Node semantics might differ between BT.CPP builds.** Both distros run the
  same 4.7.1 sources, and the mission tests pin down the semantics that
  matter (review #3).

## 10. Review responses (revision 2)

| # | Review item | Resolution |
|---|---|---|
| 1 | BLOCKING: `HoldDepth` went past D-3 / C-8 | Placeholders that always FAIL, `TODO(fish)`, no depth logic, depth published as 0 (D-A2) |
| 2 | Done-when misquoted | Changed explicitly in section 2; DEVELOPMENT.md is updated as a decision in step 7 |
| 3 | `SequenceWithMemory` index survives a retry | `GateTask` uses `Sequence`; mission test case 4 |
| 4 | Detection latency | `yaw_at_receipt`; ages 1.5 s / 2.0 s with the reasoning |
| 5 | `yolo_msgs` type mismatch | Listed under D-A4 / P-4 |
| 6 | Paused time counted | Active-time accumulation with clamped `tick_dt_s`; the controller's missing stale-odometry guard is noted; test case 5 |
| 7 | An idle manager starves teleop | Coast for 1 s, then go quiet; `OpaqueFunction` check in the vehicle launch |
| 8 | `turn_surge` falling into HOLD | Defaults documented in both configs |
| 9 | Override check | `get_parameter_overrides()` plus a whitelist |
| 10 | Bench sim details | `angular.z` filled; `turn_gain` 2.0 vs max 0.4; second RViz file; `swim_forward_mode` from the blackboard (**superseded in rev 3**: from `ctx.cfg`) |
| 11 | One-tick coast on retry | Accepted and documented |
| 12 | Missing required input; version pin | Leaves throw, the manager catches and fails the mission, a test covers it; Jazzy pinned to 4.7.1 |

## 11. Review responses (revision 3)

| # | Review item (on revision 2) | Resolution |
|---|---|---|
| 1 | BLOCKING: the `{swim_forward_mode}` blackboard key was invisible inside the `GateTask` SubTree, so every mission would fail | No blackboard keys at all. The mode lives in `ctx.cfg`, with an optional XML override. The manager and the tests share one `buildMissionTree()` |
| 2 | `Sequence` semantics confirmed; keep the threshold relationship | `detection_lost_s > detection_max_age_s` is a stated invariant, checked at startup. Tests don't assert `RetryNode` tick timing |
| 3 | Test case 2 could not pass with a ±45 deg sweep | The case is redefined by sweep/2 + HFOV/2 coverage; an outside-coverage case expects FAILURE; the XML is unchanged |
| 4 | The bench turn gain gave turns at 44% of the command | `turn_gain` 5.0 (2/3 of the command, π in about 12 s); the comment is corrected |
| 5 | `fish_tree.xml` fails immediately | Stated in its header; the `TODO(fish)` counts are updated |
| 6 | A-2's done-when changes too | Recorded in step 7 |
| 7 | Odometry-mode drift and detection ages during a pause | Documented and accepted (D-A3, section 3) |
| 8 | Restart/stop semantics; a teleop conflict remains | Specified in D-A1; the remaining conflict is documented in the README |
| 9 | The `turn_surge` coupling is only documented | Kept as comments in both configs; the manager doesn't read the controller's parameters |

## 12. Implementation notes from the approving review

These go into the code and tests. None of them changes the design.

1. **Validate `mode` values in two places.**
   - At startup, the manager rejects a `swim_forward_mode` other than `timed`
     or `odometry`, alongside the `detection_lost_s > detection_max_age_s`
     check.
   - `SwimForward::onStart` throws `BT::RuntimeError` on an unknown XML
     `mode`.
2. **Ports that are optional depend on `source`.**
   - `TurnToHeading` declares `heading_rad` and `label` with **no default**.
   - `onStart` throws if the one that `source` needs is missing.
   - Optional ports are read as `if (auto r = getInput<T>(...))`, never with
     `.value()`. They are never declared with a `""` default, because the
     parser would then inject an empty attribute.
   - A test covers `source=absolute` with no `heading_rad`, and the
     `SwimForward` fallback to `ctx.cfg` when `mode` is absent.
3. **`~/start` resets the manager's own state** as well as calling
   `haltTree()`: the odometry-loss timer, the idle-coast timer and the
   `tick_dt_s` baseline.
4. **Mission test case 2: keep the geometry away from the edge of
   coverage.**
   - Put the "inside" target well within sweep/2 + HFOV/2.
   - Put the "outside" target directly astern and far away.
   - Use the test's own fixed HFOV, not the parameter value marked
     `TODO(hw)`.

## 13. Implementation record

- **Verified on Humble / 22.04 (BT.CPP 4.7.1 from apt):**
  - `colcon test` passes: 4 geometry tests, 23 leaf tests, 5 tree-loading
    tests and 5 mission tests.
  - **Mutation check:** switching `GateTask` to `SequenceWithMemory` makes
    `MissionSim.LostDuringTurnSearchesAgain` fail. The mission starts
    `TurnToHeading` three times and fails; it never searches again. This
    confirms review item 3, and that the test catches it.
  - **Headless closed loop** (manager -> `bur_controller` -> `bench_sim` ->
    manager): the manager waits for odometry, runs `SearchPattern`, then
    `TurnToHeading` (2.6 s), then `SwimForward` (13.6 s, which is 4 m at
    0.3 m/s), and reports SUCCESS.
- **Not verified:**
  - **Jazzy.** No BT.CPP package is installed there, and building it from
    GitHub source was not permitted in that session.
  - **`bench.launch.py use_autonomy:=true` with RViz.** The WSL Humble
    install lacks `xacro`. The launch files were byte-compiled only, and the
    `vehicle.launch.py` teleop/autonomy guard has not been run.
- **Deviation found by the first headless run.** With `autostart`, the
  mission started before the first odometry message had arrived through DDS
  discovery, and the 2 s abort counted that as "odometry lost".
  - Odometry that has **never** arrived now means: wait, coasting, without
    ticking (status `WAITING_FOR_ODOM`).
  - The abort applies only to a **loss** after odometry has been received.
  - This is safer than the plan as written, and it matches the plan's
    wording ("odometry lost").
