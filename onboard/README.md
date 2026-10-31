# onboard

ROS 2 **Jazzy** workspace for the BUR 2026-2027 autonomous underwater vehicle.

The vehicle swims: propulsion is a chain of servos driving a travelling wave down
the body, not thrusters. This workspace was ported from
[`ASME-BUR/bur-onboard`](https://github.com/ASME-BUR/bur-onboard) (`beta`,
`5114a82`), which ran ROS 2 Humble on an eight-thruster AUV.

It lives in `onboard/` rather than at the repository root so it does not collide
with the hardware design files this repository also carries.

> **Nothing here has run on hardware.** The workspace builds and the control
> chain has been verified in simulation, but every gain, dimension and limit is
> a placeholder. See [DEVELOPMENT.md](DEVELOPMENT.md) for what still needs doing.

---

## Quick start

```bash
# one time
sudo bash scripts/setup_repository.sh
sudo bash scripts/install_dependencies.sh
scripts/build

# dry bench: description + gait + controller + RViz. Start here.
ros2 launch bur_bringup bench.launch.py

# unit tests (no hardware needed)
colcon test --packages-select bur_gait bur_msgs && colcon test-result --verbose
```

---

## System overview

Three computers, each doing one job.

| Where | Runs | Link |
|---|---|---|
| **Raspberry Pi 4** | ROS 2 Jazzy: state estimation, outer control loop, autonomy (Bar30 over local I2C until it moves to the MCU) | — |
| **MCU** (Arduino Due) | servo chain, gait phase accumulator, IMU, later the Bar30 | micro-ROS over USB serial |
| **Jetson Orin Nano** | ZED stereo camera, YOLO detection | DDS, once it is on Jazzy |

### Topic flow

```
                     /swim_command  (bur_msgs/SwimCommand)
   teleop or BT  ──────────────────────────────────┐
                                                   ▼
   Bar30 ──/depth_sensor──┐              ┌──────────────────┐
   IMU ────/imu───────────┼── ekf_node ──┤  bur_controller  │
   (ZED, later) ──────────┘       │      └────────┬─────────┘
                                  │               │ /gait_command
                       /odometry/filtered         │  (bur_msgs/GaitCommand)
                                                  ▼
                                          ┌───────────────┐
                                          │   MCU, over   │  owns the phase
                                          │   micro-ROS   │  accumulator
                                          └───────┬───────┘
                                                  │ servo PWM
                         bur_gait ────────────────┘
                   (bench preview + /joint_states for RViz)
```

Two things in that diagram are easy to miss and matter:

**`/gait_command` carries parameters, not angles.** It says *frequency,
amplitude, wavelength, turn bias* — and the MCU runs the phase accumulator
locally, every servo frame. Serial jitter on a 50 Hz joint-angle topic shows up as
visible wobble at the tail, and a dropped message should coast the gait rather
than freeze a joint mid-beat.

**`bur_gait` is not in the control path on the vehicle.** It runs the same wave
maths on the Pi so a gait can be watched in RViz before the hull is wet. Once
the firmware is live, launch with `use_gait_node:=false` or both will drive the
servos.

### Coordinate conventions

- **NWU** (North-West-Up), carried over from the previous vehicle.
- Depth below the surface is **negative z**.
- Positive yaw rate and positive `turn_bias` are **to port**.

---

## Repository structure

```
onboard/
├── README.md               this file
├── DEVELOPMENT.md          what still needs building, task by task
├── required.repos          external deps (micro-ROS, pinned to jazzy)
├── scripts/
│   ├── setup_repository.sh apt dependencies
│   ├── install_dependencies.sh  vcs import + rosdep
│   └── build               colcon wrapper with OOM retry (useful on a Pi)
├── docker/                 Jazzy dev container
├── firmware/               MCU firmware — EMPTY, see DEVELOPMENT.md
└── src/
    ├── interfaces/
    │   └── bur_msgs        message definitions
    ├── gnc/
    │   ├── bur_controller  outer loop: pose/rate error -> gait parameters
    │   ├── bur_gait        travelling-wave gait generator
    │   ├── bur_localization  robot_localization EKF config + sensor TFs
    │   └── bur_autonomy    behaviour trees (manager node not written)
    ├── sensors/
    │   ├── bur_sensors     Bar30 depth node
    │   └── drivers         MS5837 driver + i2c-dev shim
    ├── description/
    │   └── bur_description URDF of the joint chain
    └── launch/
        ├── bur_bringup     vehicle / bench / micro-ROS launch
        └── bur_teleop      gamepad -> SwimCommand
```

### Where each package came from

| Package | Origin |
|---|---|
| `bur_msgs` | Ported from `bur-interfaces`, now vendored so the workspace builds offline. `CVDetection`, `CVDetections` verbatim; `SwimCommand`, `GaitCommand`, `ServoChainCommand`, `ServoChainState` new; `ThrusterCommand` and `Command` dropped. |
| `bur_controller` | PID classes ported intact; the node is new (4 axes not 6, 10 Hz not 30, emits gait parameters not a wrench). |
| `bur_gait` | **New.** Replaces `bur_thruster_manager` — a static pseudo-inverse force allocation matrix has no meaning for a serial-link chain. |
| `bur_localization` | Ported near-verbatim. `robot_localization` exists on Jazzy with the same parameter names. ZED input disabled until the Jetson is on Jazzy. |
| `bur_autonomy` | Behaviour tree XML ported. The manager node was deliberately not ported. |
| `bur_sensors` | Ported, **now superseded.** The Bar30 moves to the microcontroller (DEVELOPMENT.md F-4), so this node and its I2C path are on their way out. Kept for now as a fallback and as reference for the conversion maths. |
| `drivers` | Ported, **now superseded** along with `bur_sensors`. `MS5837.cpp` is byte-for-byte identical to the original; only its `#include` changed. The `i2c_bus` shim exists only to serve it. |
| `bur_description` | **New.** The previous vehicle had no URDF at all, just chained `static_transform_publisher` calls. |
| `bur_bringup` | Ported, plus a new `bench.launch.py`. |
| `bur_teleop` | Replaces `bur_rov`. Split into two jobs that were previously fused — see below. |

### Messages

| Message | Purpose |
|---|---|
| `SwimCommand` | High-level intent: surge, yaw rate, depth, pitch. **No sway term** — the vehicle cannot strafe, and a field that exists but is ignored eventually gets set. |
| `GaitCommand` | Gait parameters plus a mode (`ESTOP`/`COAST`/`HOLD`/`CRUISE`/`TURN`). |
| `ServoChainCommand` | Per-joint angles. Bring-up and debugging only. |
| `ServoChainState` | Joint telemetry: position, current, temperature, faults. |
| `CVDetection`, `CVDetections` | Carried over verbatim. |

---

## Prerequisites

| | |
|---|---|
| OS | Ubuntu 24.04 (Noble) |
| ROS | Jazzy |
| Simulation | none in-tree — MuJoCo, externally |

Ubuntu 24.04 arm64 on a Raspberry Pi 4 is a Tier 1 Jazzy platform, so `apt`
works normally there.

**On a Raspberry Pi, check this first.** Ubuntu's Pi images ship with the extra
apt suites disabled and the ROS packages will not resolve without them:

```bash
grep Suites /etc/apt/sources.list.d/ubuntu.sources
# the first entry must read:  Suites: noble noble-updates noble-backports
```

---

## Installation

```bash
git clone https://github.com/ASME-BUR/2026-2027.git
cd 2026-2027/onboard
sudo bash scripts/setup_repository.sh
```

That installs `robot_localization`, `behaviortree_cpp`, `robot_state_publisher`,
`xacro`, `joy`, `v4l2_camera` and the I2C libraries. It does **not** build
pigpio from source any more — see [Hardware notes](#hardware-notes).

Then fetch external repositories:

```bash
sudo bash scripts/install_dependencies.sh
```

This pulls the micro-ROS agent from its `jazzy` branch. Note that
`bur-interfaces` is no longer fetched: `bur_msgs` is vendored in
`src/interfaces/bur_msgs`, so the workspace builds with no network.

---

## Building

```bash
scripts/build
```

The wrapper retries on the `cc1plus` OOM kill that `colcon` triggers on
memory-constrained machines. On a Pi, add swap first — 4 GB is tight.

To build one package:

```bash
scripts/build --packages-select bur_gait
```

To run the tests (the `bur_gait` gtest checks the wave maths that also goes
into the MCU firmware; no ROS runtime or hardware involved):

```bash
colcon test --packages-select bur_gait bur_msgs && colcon test-result --verbose
```

`bur_msgs` and `bur_gait` also build on Humble / Ubuntu 22.04, which is enough
to run the tests on a machine without Jazzy. Everything else targets Jazzy: use
the dev container (`docker compose -f docker/docker-compose.yml run --rm
bur-onboard`). Delete `build/ install/ log/` when switching distros.

To start clean:

```bash
rm -r build/ install/ log/
```

---

## Running

```bash
# dry bench: no sensors, no MCU, no EKF. Description + gait + controller + RViz.
ros2 launch bur_bringup bench.launch.py

# the bench controller coasts (no odometry), so to see the wave move, turn it
# off and publish a gait by hand. Keep -r: bur_gait coasts after 0.3 s without
# a command.
ros2 launch bur_bringup bench.launch.py use_controller:=false
ros2 topic pub -r 10 /gait_command bur_msgs/msg/GaitCommand   "{mode: 3, frequency_hz: 1.0, amplitude: 0.5, wavelength: 1.0}"

# the vehicle
ros2 launch bur_bringup vehicle.launch.py

# with a gamepad
ros2 launch bur_bringup vehicle.launch.py use_teleop:=true

# once the MCU owns the gait phase, stop the Pi-side generator
ros2 launch bur_bringup vehicle.launch.py use_gait_node:=false
```

`vehicle.launch.py` arguments:

| Argument | Default | Effect |
|---|---|---|
| `use_sensors` | `true` | Bar30 over local I2C. Requires running on the Pi. |
| `use_micro_ros` | `true` | micro-ROS agents for the MCU link. |
| `use_teleop` | `false` | Gamepad. Off by default so autonomy can run alone. |
| `use_gait_node` | `true` | Pi-side gait generator. Set `false` when the MCU owns the phase. |

### Expected idle behaviour

With no `/swim_command` and no odometry, the controller publishes
`GaitCommand.mode = MODE_COAST` and every joint sits at zero. **That is correct,
not a failure.** The controller refuses to act without a state estimate, and the
deadman is a stale-command timeout rather than a held button.

### Checking it works

```bash
ros2 topic hz /gait_command              # should be ~10 Hz
ros2 topic echo /gait_command --once
ros2 topic echo /servo_chain_command --once --field position_rad
ros2 run rqt_reconfigure rqt_reconfigure # retune PID gains live
```

Tail amplitude should exceed head amplitude — that is the envelope working.

---

## Hardware notes

### I2C (Bar30 depth sensor)

> **Superseded.** The Bar30 is moving to the microcontroller, after which the
> Pi needs no I2C at all and this whole section — the `i2c` group, `i2cdetect`,
> `/dev/i2c-1` — stops applying. It is kept while `bur_sensors` remains as a
> fallback. See DEVELOPMENT.md F-4.

The depth node opens `/dev/i2c-1` directly, so **it must run on the Pi**. The
previous vehicle reached the Pi's GPIO over TCP through a `pigpiod` daemon with
the IP hardcoded in launch files; that indirection is gone.

```bash
sudo usermod -aG i2c $USER   # log out and back in
i2cdetect -y 1               # the Bar30 should answer at 0x76
```

**Why pigpio was removed:** the Bar30 driver only ever used five pigpio
functions and all five are plain I2C, so `src/sensors/drivers/src/i2c_bus/`
reimplements exactly those five on Linux `i2c-dev` with identical signatures and
return conventions. `MS5837.cpp` therefore needed no changes at all. That one
swap removed the daemon, the build-from-source step, the hardcoded addresses,
and any question about pigpio on Ubuntu 24.04.

The other old drivers were dropped: `raspi_dma_pwm` (only existed to drive
thruster ESCs from the Pi, which never happened), `xsens_mti_i2c` (the IMU
arrives over micro-ROS) and `lps331ap_i2c` (its node was a copy-paste of the IMU
node and had never compiled).

### Serial (micro-ROS)

Two agents by default, at 250000 baud. This dates from a two-Arduino design;
the vehicle now has one Due, so DEVELOPMENT.md X-2 cuts this to a single agent
on a stable `/dev/bur_mcu` udev name.

| Device | Carries |
|---|---|
| `/dev/ttyACM0` | servo chain: `GaitCommand` in, `ServoChainState` out |
| `/dev/ttyACM1` | IMU: `sensor_msgs/Imu` out |

Override with `ros2 launch bur_bringup micro_ros.launch.py servo_dev:=/dev/ttyACM2`.

> `/dev/ttyACM*` numbering is not stable across reboots. Add udev rules keyed on
> the USB serial number before this bites you at a competition.

### Jetson

The ZED and YOLO are **not** part of this workspace; they run on the Jetson and
are launched there. Until that board is on JetPack 7.2 (which is Ubuntu 24.04,
and therefore Jazzy), keep it out of the ROS graph entirely — Humble-to-Jazzy
DDS is unsupported and fails badly, including a known case of a Humble
subscriber consuming unbounded memory against a Jazzy publisher. The EKF is
configured to run on depth and IMU alone until then.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `/gait_command` stuck at `mode: 1` | `MODE_COAST`. No `/odometry/filtered`, no `/swim_command`, or `enable: false`. The controller will not act without a state estimate. |
| Joints saturate at `±0.6 rad` | `joint_limit_rad` in `bur_gait/config/gait.yaml`. Only the wave and `turn_bias` compete for joint range now; the fin command is a separate actuator and no longer steals from it. |
| `Bar30 not detected on /dev/i2c-N` | I2C not enabled, user not in the `i2c` group, or the node is not running on the Pi. |
| ROS packages will not resolve on a Pi | `noble-updates` / `noble-backports` disabled. See [Prerequisites](#prerequisites). |
| `colcon` killed mid-build | Out of memory. `scripts/build` retries; add swap. |
| Both the Pi and the MCU driving the servos | Launch with `use_gait_node:=false`. |
| `Clock skew detected` during build | Building from `/mnt/c` under WSL. Harmless; clone into the WSL filesystem to avoid it. |

---

## Contributing

Read [DEVELOPMENT.md](DEVELOPMENT.md) first. It lists every outstanding task
with its inputs and outputs, and marks which ones are blocked on decisions
nobody has made yet rather than on effort.

`src/` carries 52 greppable markers (counts per kind are in DEVELOPMENT.md):

```bash
grep -rn "TODO(fish)" src     # inherited logic that is wrong for a swimming body
grep -rn "TODO(hw)" src       # needs a measured number from the vehicle
grep -rn "TODO(decide)" src   # open design decision — do not guess
grep -rn "TODO(port)" src     # mechanical carry-over not finished
grep -rn "TODO(verify)" src   # unconfirmed assumption
```
