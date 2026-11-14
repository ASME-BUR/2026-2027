# bur_autonomy

The mission manager. It ticks a BehaviorTree.CPP v4 tree at 10 Hz and
publishes `bur_msgs/SwimCommand` (surge and yaw rate) to `bur_controller`. It
covers A-1, A-2 and A-3 in [`DEVELOPMENT.md`](../../../DEVELOPMENT.md). The
design, and the review that approved it, are in [`PLAN.md`](PLAN.md).

**It has only run against `bench_sim`**, a kinematic stand-in whose numbers
are invented. Nothing has run with real odometry, real detections or water.

## Run

```bash
ros2 launch bur_bringup bench.launch.py use_autonomy:=true      # bench: bench_sim + manager, RViz fixed to odom
ros2 launch bur_bringup bench.launch.py use_autonomy:=true use_rviz:=false   # headless
ros2 launch bur_bringup vehicle.launch.py use_autonomy:=true     # vehicle: waits for ~/start
ros2 service call /bur_autonomy/start std_srvs/srv/Trigger
ros2 service call /bur_autonomy/stop  std_srvs/srv/Trigger
ros2 topic echo /bur_autonomy/status                              # state, tree status, running leaf
```

`autonomy.launch.py` takes the arguments `tree` (default
`fish_tree_horizontal.xml`), `config`, `autostart` and `swim_forward_mode`.

## Trees

| File | Use |
|---|---|
| `behavior_trees/fish_tree_horizontal.xml` | **The one to launch.** The gate task: detect or search, turn to face the target, swim 4 m |
| `behavior_trees/fish_tree.xml` | The full target mission. It **fails on its first tick**, because `HoldDepth` and `Surface` are deferred placeholders (D-3 / C-8 / F-4). It has to load, but don't launch it |
| `behavior_trees/reference/` | Last year's trees. They don't load and are never loaded; see the README there |

Rules for every tree. Each one is written down because breaking it breaks a
mission:

- **No `Parallel` node over action leaves.** Only one leaf may write the swim
  command per tick.
- **No blackboard `{keys}`.** Settings reach the leaves through the manager's
  parameters (`MissionContext.cfg`). In v4, each `SubTree` has its own
  isolated blackboard.
- **Retried subtrees use a plain `Sequence`, not `SequenceWithMemory`.** On
  a retry, `SequenceWithMemory` resumes at the child that failed, so a lost
  target never triggers a new search. `test_mission_sim` fails if this rule
  is broken; that was checked by mutation.

## Leaves

| Leaf | What it does |
|---|---|
| `TurnToHeading` | Turns to `heading_rad` (`source=absolute`; 0 means the heading at power-on) or toward the latest sighting of `label` (`source=detection`). It moves forward at `turn_surge_mps` while turning, because at zero surge the controller sends `MODE_HOLD`, which produces no thrust |
| `SwimForward` | Swims `distance_m` at `speed_mps`, holding the starting heading. With `mode` `timed` (the default until P-3) it runs for distance/speed; with `odometry` (used on the bench) it measures distance travelled |
| `SearchPattern` | Sweeps ±`sweep_rad`/2 around the starting heading until it sees `label` |
| `IsDetected` | Condition: a sighting of `label` younger than `detection_max_age_s` |
| `HoldDepth`, `Surface` | Placeholders that always fail, marked `TODO(fish)` (D-3 / C-8 / F-4) |

Missing or invalid ports fail the mission with a clear error on the first
tick; they never act on garbage.

## Safety behaviour

- **Odometry:**
  - Before the first odometry message, the manager waits while coasting.
  - If odometry goes stale (older than 0.5 s), the manager stops ticking and
    coasts.
  - After 2 s stale, the mission fails.
  - `bur_controller` has no stale-odometry guard of its own, so this is the
    only one.
- **Timers** count only ticks that actually ran, so a pause is never counted
  as distance swum.
- **When a mission ends or `~/stop` is called,** the manager coasts for 1 s
  and then stops publishing. It never shuts the node down.
- **`~/start`** refuses while a mission is active. After a mission it starts
  a fresh one.
- **Teleop conflict:** `vehicle.launch.py` refuses `use_teleop:=true` together
  with `use_autonomy:=true`. **A teleop node started by hand during a mission
  is not caught.** Both publish `/swim_command`, so don't do that.
- **Parameters:** an unknown key in `config/autonomy.yaml` stops the node at
  startup. So do `detection_lost_s <= detection_max_age_s`, a bad
  `swim_forward_mode` and `turn_surge_mps <= 0`.

## Open items

- **Detections, `TODO(decide)` (P-4 / D-A4).** `include/bur_autonomy/detection.hpp`
  assumes `bbox.pose.position.x` holds the pixel x of the box centre, which is
  what `bur-vision/yolo.py` does. But `yolo.py` publishes `yolo_msgs` (not
  `bur_msgs`) on `yolo_detections`. **Nothing real feeds `/vision` yet.**
- **Mission start, `TODO(decide)` (D-A1).** A physical start or kill switch
  is the team's call. Today the mission starts via the `~/start` service or
  `autostart`.
- **Placeholders that need hardware (`TODO(hw)`).** `max_yaw_rate_rps` and
  `turn_surge_mps` wait on M-3; the camera width and HFOV wait on the ZED
  settings.
- **A-4** (a curvature-aware planner) is only an outline in PLAN.md.

## Tests

```bash
colcon test --packages-select bur_autonomy && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `test_geometry` | Distance helpers, angle wrap |
| `test_leaves` | Every leaf against a fake clock |
| `test_tree_loading` | A-3: every top-level tree loads against the real factory; bogus nodes, undeclared attributes and missing ports fail |
| `test_mission_sim` | The horizontal tree closed-loop against a unicycle and a 2 Hz camera with latency: target ahead, search, outside coverage (fails after 3 attempts), lost mid-turn (searches again), pause not counted |
