# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Electrical hardware and software for the ASME Bruin Underwater Robotics 2026-2027 vehicle. The vehicle **swims**: a chain of servos drives a travelling wave down the body, replacing the eight-thruster AUV of 2025-2026. All software is the ROS 2 **Jazzy** workspace in `onboard/`; schematics and board files live on separate branches (e.g. `leak_sense`), not on `main`.

The workspace was ported from `ASME-BUR/bur-onboard` (Humble, thrusters). A local checkout of that repo may sit next to this one as `../bur-onboard` — it is reference only. Other code is split into separate repos **by which computer it runs on** (`bur-arduino`, `bur-vision-nano`, `bur-vision`, …), so something missing here may be missing on purpose; see `README.md`.

**Nothing has run on hardware.** It builds and the control chain is verified in simulation (MuJoCo, external, not in-tree). Every gain, dimension and limit is a placeholder. `onboard/firmware/` is empty and there is no autonomy manager node yet.

## Commands

Run from `onboard/` on Ubuntu 24.04 + Jazzy, or in the dev container (`docker compose -f docker/docker-compose.yml run --rm bur-onboard`). Humble / 22.04 also works for the tests and the bench (verified 2026-10-31, `--packages-up-to bur_bringup`); delete `build/ install/ log/` when switching distros.

```bash
sudo bash scripts/setup_repository.sh      # apt deps (one time)
sudo bash scripts/install_dependencies.sh  # vcs import required.repos (micro-ROS agent) + rosdep
scripts/build                              # colcon build --symlink-install, retries on cc1plus OOM
source install/setup.bash                  # needed: scripts/build's own `source` runs in a subshell and doesn't reach the caller
scripts/build --packages-select bur_gait   # one package (any colcon args pass through)
scripts/build --packages-up-to bur_bringup # what the launch files need; --packages-select bur_bringup alone is not enough
rm -r build/ install/ log/                 # clean

ros2 launch bur_bringup bench.launch.py    # dry bench: URDF + gait + controller + RViz (rviz/bench.rviz), no hardware. Start here.
ros2 launch bur_bringup bench.launch.py use_controller:=false  # then hand-publish /gait_command to animate the wave
ros2 launch bur_bringup vehicle.launch.py  # args: use_sensors, use_micro_ros, use_teleop, use_gait_node
```

Tests: `bur_gait` has a gtest for the ROS-free wave maths in `gait.hpp` (`test/test_gait.cpp`: envelope, wave direction, clamping, phase continuity, rate limit); `bur_msgs` runs `ament_lint_auto` (copyright, cpplint and xmllint skipped; xmllint needs network). Run with `colcon test --packages-select bur_gait bur_msgs && colcon test-result --verbose`; no hardware or network needed. `test-result` also reads stale reports left under `build/`, so `rm -r build/<pkg>` after changing a package's tests. Sanity check after launching: `ros2 topic hz /gait_command` should read ~10 Hz.

## Architecture

Three computers: **Raspberry Pi 4** (ROS 2: EKF, outer control loop, autonomy), **Arduino Due** (servo chain, gait phase accumulator, IMU, later the Bar30; micro-ROS over USB serial. `micro_ros.launch.py` still starts two agents on `ttyACM0`/`ttyACM1` from a two-MCU design; X-2 cuts it to one on `/dev/bur_mcu`), **Jetson Orin Nano** (ZED + YOLO, not in this workspace).

Control chain: `teleop/BT → /swim_command (SwimCommand) → bur_controller → /gait_command (GaitCommand) → MCU → servo PWM`, with `robot_localization` EKF feeding `/odometry/filtered` to the controller.

Design points that span several files and are easy to break:

- **`GaitCommand` carries gait parameters (frequency, amplitude, wavelength, turn_bias, mode), not joint angles.** The MCU runs the phase accumulator locally so serial jitter and dropped messages don't wobble or freeze joints. `ServoChainCommand` (per-joint angles) is for bring-up/debug only.
- **`bur_gait` is a bench/RViz preview, not in the vehicle control path.** Once firmware drives the servos, launch with `use_gait_node:=false` or both will drive them. `gait.hpp` is deliberately ROS-free so the same wave function can compile into MCU firmware — keep it that way, and keep it strict C++17 (use `bur_gait::kPi`, not the POSIX `M_PI`).
- **The vehicle is non-holonomic.** `SwimCommand` has no sway field by design; don't add one. The controller runs 4 axes at 10 Hz and outputs gait parameters, not a wrench. There is no force-allocation matrix (old `bur_thruster_manager` was dropped, not ported).
- **The chain is 4 joints, all revolute about Z: yaw only, no pitch authority.** Pitch/depth are expected to come from fins (`GaitCommand.fin_angle_rad`, a separate actuator — never add it to a chain joint). Keep the pitch/depth control paths.
- **`n_joints` lives in three places:** the unrolled URDF chain (`bur.urdf.xacro`), `bur_gait/config/gait.yaml`, and the defaults in `gait.hpp` / `gait_node.cpp`. Change them together.
- **Idle `MODE_COAST` with all joints at zero is correct**, not a bug: the controller won't act without `/odometry/filtered`, and the deadman is a stale-command timeout (0.3 s in `bur_gait`).
- **Keep the Jetson off the ROS graph** until it runs JetPack 7.2 (Jazzy). Humble↔Jazzy DDS is unsupported. The EKF runs on depth + IMU only until then.
- `bur_msgs` is vendored (not fetched from `bur-interfaces`) so the workspace builds offline.
- `bur_sensors` and `drivers` (Bar30 over Pi I2C) are superseded by the depth sensor moving to the MCU (F-4); kept only as fallback/reference.
- Conventions: **NWU** frame, depth below surface is **negative z**, positive yaw rate and `turn_bias` are **to port**.

## Working rules from DEVELOPMENT.md

`onboard/DEVELOPMENT.md` is the task list (IDs D=decisions, M=measurements, F=firmware, C=controls, A=autonomy, P=perception, X=infra), each with inputs, outputs and blockers. Read the relevant entry before changing related code. IDs are never reused; gaps are retired tasks.

- **Do not guess past a `BLOCKED` task or a `TODO(decide)`.** Open design decisions (fin count/placement under D-3, etc.) belong to the team. Settled: the MCU is an **Arduino Due** (D-2: community-supported by micro-ROS, 3.3 V logic); the Due owns the gait phase accumulator (D-5); pitch/depth come from **fins with their own actuators** (D-3); servo waterproofing (X-7) is a mechanical covering, not software work.
- **Current focus is horizontal motion (surge + yaw).** Depth/pitch work (F-4, M-6, C-6, C-8, fin routing, `HoldDepth`/`Surface` leaves) is deferred: don't build it out, but don't delete those code paths either.
- Source markers cross-reference tasks: `grep -rn "TODO(fish)\|TODO(hw)\|TODO(decide)\|TODO(port)\|TODO(verify)" onboard/src`. `TODO(hw)` values need a measured number, so don't "fix" them with a different guess.
- When a decision or task status changes, update DEVELOPMENT.md (status table, task entry, marker counts) in the same change.
