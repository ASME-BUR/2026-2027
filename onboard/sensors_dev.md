# SENSORS_DEV: sensors to the Pi, vision to the Pi, IMU + VO fusion

Part of [DEVELOPMENT.md](DEVELOPMENT.md). Task IDs, the `BLOCKED` rule and the
hardware baseline are defined there. IDs are shared across all three files and
never reused.

**Status, 2026-11-14: the current focus.** Horizontal tail motion
([tail_dev.md](tail_dev.md)) is paused until the chain is wired.

**Goal:** `/odometry/filtered` on the Pi fuses the MTi-3 IMU (attitude, rates)
with ZED visual odometry (horizontal velocity), with the ZED's data arriving
from the Jetson over a small UDP bridge.

**Done when:** carrying the rig (Pi + Due with IMU + Jetson with ZED) around a
measured 4 m square brings `/odometry/filtered` back within ~0.5 m of its
start, its heading tracks the real heading, and unplugging the Jetson link
degrades the estimate to IMU-only without the EKF diverging or stopping.

---

## Decisions (2026-11-14)

| Decision | Consequence |
|---|---|
| **The Jetson stays on JetPack 6.2.1 / Humble** | P-1 and P-2 are retired. No reflash, no CUDA 13 / TensorRT rebuild |
| **A UDP bridge carries ZED odometry and detections only** | The Jetson's DDS graph must never reach the Pi's (X-8). Every bridged topic needs a wire format (P-5) |
| **Fusion starts with ZED visual odometry in the existing `robot_localization` EKF** | No custom filter. Landmark fixes from detections come later, if at all |
| **The IMU stays on the Due**, brought up alone, with no servos attached | F-5 firmware is reused. No Pi IMU driver |

## Data flow

```
 Jetson (JetPack 6.2.1, Humble, isolated graph)        Pi 4 (Jazzy)
 ┌───────────────────────────────────────────┐          ┌─────────────────────────────────────────┐
 │ zed_wrapper ── odom ──┐                    │   UDP    │ bridge_receiver ─> /zed/odom ──┐        │
 │                       ├─> bridge_sender ───┼─────────>│                 ─> /vision (P-6)│        │
 │ yolo.py ─ detections ─┘                    │ tether   │                                 v        │
 └───────────────────────────────────────────┘          │ micro_ros_agent ─> /imu ──> ekf_node ─> /odometry/filtered
                                                        └──────────────▲──────────────────────────┘
 Due ── MTi-3 on Wire1 ── micro-ROS over USB ───────────────────────────┘
 chrony: the Pi serves time, the Jetson follows it (X-8)
```

## Order of work

```
 F-5 IMU-only bring-up ─┬─> X-2 single agent + udev
                        ├─> X-9 one owner per sensor TF ─> M-5 offsets ──┐
 X-8 network + clock ───┴─> P-5 odometry bridge ─────────> P-3 VO in EKF ─┼─> C-5 (dry) ─> rig test (done)
                                                                         │
 D-A4 detection contract ─> P-6 detection bridge ─> P-4 detections drive behaviour
```

The two lines at the top are independent: one person can bring up the IMU
while another builds the bridge against a fake sender.

| Task | Status | Needs |
|---|---|---|
| F-5 IMU firmware | Written, never run on a board | Due + MTi-3, wired |
| X-2 single agent + udev | **Unblocked** (udev serial needs the Due) | — |
| X-9 one owner per sensor TF | **Unblocked**, software only | — |
| X-8 network + clock + isolation | Unblocked | Pi, Jetson, the link between them |
| P-5 odometry bridge | **Unblocked**, software only | — |
| P-3 VO in the EKF | Blocked by P-5 | — |
| M-5 sensor offsets | Needs the built nose, or the rig | — |
| C-5 EKF retune | Dry stage blocked by P-3; wet stage by C-4 (tail_dev) | — |
| P-6 detection bridge | Blocked by D-A4 | — |
| P-4 detection loop | Blocked by P-6 | — |

---

## F — Firmware

### F-5 — IMU firmware: bring it up alone
**Written, untested on hardware** · **Blocks:** X-2's udev rule, P-3, M-7, the yaw loop

The MTi-3 lives on the Due (hardware baseline), in the same sketch and
micro-ROS agent as the gait loop. **The servos don't need to be attached.**
With no `GaitCommand` the chain sits in `MODE_COAST`, and PWM into
unconnected pins is harmless. This bring-up also proves micro-ROS on the Due,
which is step 1 of F-1.

1. Wire the MTi-3 to `Wire1` (SDA1/SCL1), DRDY to D3, 3.3 V. **`Wire1` has no
   on-board pull-ups**: add them. See
   [`firmware/bur_mcu/README.md`](firmware/bur_mcu/README.md#wiring).
2. `pio run -t upload` over the Native USB port. Run a **Jazzy** agent on the
   Pi (`board_microros_distro = jazzy` must match).
3. Work through the IMU items in the firmware's
   [hardware checklist](firmware/bur_mcu/README.md#not-verified-yet):
   - MTi detected on `Wire1`; `/imu` at about 100 Hz (`ros2 topic hz /imu`);
   - rotating to port gives positive yaw rate (`TODO(verify)` in
     `src/imu.cpp`, frame convention);
   - `ros2 topic info -v /imu`: the firmware publishes best-effort; confirm
     `robot_localization`'s subscription accepts it;
   - covariances against the MTi-3 datasheet (`TODO(verify)` in `src/imu.h`).
4. Check micro-ROS time sync: `/imu` stamps should track the Pi's clock to
   within a few ms (`ros2 topic echo /imu --field header.stamp` against
   `date +%s.%N`). A drifting stamp shows up as a lagging estimate.

- **Input:** Due, MTi-3, pull-up resistors, the Pi on Jazzy
- **Output:** `sensor_msgs/Imu` on `/imu`, `frame_id: imu`, orientation,
  angular velocity and linear acceleration, **diagonal** covariances
- **Done when:** `/imu` publishes at a steady rate, and rotating the board by
  hand to port shows positive yaw rate in `/odometry/filtered`. (The "no gait
  hitch" half of the old criterion moves to F-1.)

---

## X — Infrastructure

### X-2 — Single micro-ROS agent and udev rule
**Unblocked** (the serial number needs the Due from F-5)

`micro_ros.launch.py` still starts two agents (`ttyACM0` servo, `ttyACM1` IMU)
from the two-Arduino era, and `/dev/ttyACM*` numbering is not stable across
reboots.

- **Input:** the Due's USB serial number for the **Native** port
  (`udevadm info -a -n /dev/ttyACM0 | grep serial`)
- **Output:** one agent on `/dev/bur_mcu`, launch argument `mcu_dev`; a udev
  rule in `bur_bringup/udev/` keyed on vendor, product and serial; the stale
  `TODO(decide)` removed (update the marker counts in DEVELOPMENT.md)
- **Done when:** the launch matches the hardware and survives a reboot and replug

### X-8 — Pi-to-Jetson network, clock and graph isolation
**Unblocked** · **Blocks:** P-5's hardware test

Three things must be true before any bridged data is trustworthy.

1. **Link.** Static IPs on the Pi-Jetson link (for example `192.168.50.1` Pi,
   `192.168.50.2` Jetson), set in netplan on both. No DHCP on the vehicle.
2. **Clock.** The EKF fuses by timestamp, and the bridge keeps the ZED's own
   stamps. Run chrony with the **Pi as server** (`local stratum 10`,
   `allow 192.168.50.0/24`) and the Jetson as client
   (`server 192.168.50.1 iburst`). Neither board has internet in the pool, so
   they agree with each other, not with UTC, and that is enough.
3. **Isolation.** The Jetson's Humble graph must not discover the Pi's Jazzy
   graph over the link (Humble-to-Jazzy DDS is unsupported; one known failure
   is a Humble subscriber consuming unbounded memory). In the Jetson's ROS
   environment set `ROS_LOCALHOST_ONLY=1` **and** a `ROS_DOMAIN_ID` different
   from the Pi's. Both, so one forgotten shell can't break it.

- **Input:** the physical link between the boards (tether switch or a direct
  cable)
- **Output:** netplan and chrony configs committed under
  `bur_bringup/config/network/`; the Jetson env lines in that repo's
  bringup notes
- **Done when:** `chronyc tracking` on the Jetson shows under 2 ms offset;
  `ros2 topic list` on the Pi shows **no** Jetson topics while the ZED is
  running

### X-9 — One owner per sensor transform
**Unblocked, small** · **Confirmed defect**

`base_link -> imu` and `base_link -> depth_sensor` are each published twice:
by the URDF (through `robot_state_publisher`) and by static publishers in
`bur_localization/launch/ekf.launch.py`, **with different values** (the IMU is
at x = 0.02 in one and -0.1 in the other). `vehicle.launch.py` starts both, so
TF flips between them. This is lesson 4 of the appendix again.

- **Output:** the two sensor static publishers removed from `ekf.launch.py`
  (keep `map -> odom`); the URDF is the only source of sensor offsets,
  including `zed_camera_link`
- **Done when:** `ros2 run tf2_ros tf2_echo base_link imu` is steady and
  matches the URDF

---

## M — Measurements

### M-5 — Sensor mounting offsets
**Feeds:** the URDF (after X-9, the only place they live)

All current offsets come from last year's hull and are certainly wrong. Two
matter most here: the IMU yaw offset (a wrong one makes the heading loop chase
itself), and `base_link -> zed_camera_link`, because P-3 fuses ZED velocities
rotated through that transform.

- **Input:** the built nose, or the test rig with the IMU and ZED fixed on it;
  calipers
- **Output:** measured `base_link` -> `imu`, `depth_sensor`, `zed_camera_link`
  in `bur.urdf.xacro`
- **Done when:** TF matches the physical layout, and walking the rig straight
  forward gives positive x velocity on `/zed/odom` after rotation into
  `base_link`

---

## P — Perception

Nothing on the Jetson is in this workspace except the bridge sender. The ZED
wrapper and YOLO live in
[`ASME-BUR/bur-vision-nano`](https://github.com/ASME-BUR/bur-vision-nano)
and [`ASME-BUR/bur-vision`](https://github.com/ASME-BUR/bur-vision).

### P-5 — UDP odometry bridge
**Unblocked** · **Blocks:** P-3, P-6

One `ament_python` package, `src/sensors/bur_vision_bridge`, with two nodes
and one shared wire format. It must build on **both** Humble (Jetson) and
Jazzy (Pi), so it uses only `rclpy`, standard messages and the Python standard
library. The Jetson builds this one package from this repo. Keeping the format
in one file is the point (lesson 4).

```
bur_vision_bridge/
  protocol.py       ROS-free: encode/decode, the only definition of the format
  sender_node.py    Jetson: subscribe, encode, sendto()
  receiver_node.py  Pi: recvfrom(), validate, publish
launch/  sender.launch.py, receiver.launch.py
config/  bridge.yaml
test/    test_protocol.py, test_receiver.py (pytest, no network)
```

**Wire format** (little-endian, one message per datagram, under 1400 bytes so
it never fragments):

| Field | Type | Notes |
|---|---|---|
| magic | 4 bytes `BURV` | drop anything else |
| version | u8 | bump on any change; the receiver drops other versions and logs once |
| msg_type | u8 | 1 = odometry, 2 = detections (P-6) |
| reserved | u16 | 0 |
| seq | u32 | per type; the receiver drops anything not newer |
| stamp_ns | i64 | **the ZED's own stamp**, not the send time |
| payload | | odometry: position (3), orientation quaternion (4), linear velocity (3), angular velocity (3), pose covariance (36), twist covariance (36), all f64: 680 bytes |

**Sender (Jetson, Humble):**
- Subscribes to the ZED wrapper's odometry. The topic depends on the wrapper
  version and camera name (`/zed/zed_node/odom` or `/zedm/zed_node/odom`):
  check `ros2 topic list` there and set it in `bridge.yaml`; don't hard-code it.
- Sends nothing while the ZED's positional tracking is not OK. Read the
  wrapper's tracking-status topic if the installed version has one; otherwise
  drop messages whose covariance the wrapper has flagged as invalid. Silence is
  a clean "no data" for the EKF; a stale or frozen pose is not.
- Sends to the Pi's address and port from `bridge.yaml`. Fire and forget:
  no acks, no retries. Odometry is a stream.

**Receiver (Pi, Jazzy):**
- Binds the port, validates magic, version and length, and drops
  out-of-order `seq`.
- Drops messages older than `max_age_s` (start at 0.25 s) or more than 50 ms
  in the future. Either means the clock (X-8) is wrong, so warn, throttled.
- Publishes `nav_msgs/Odometry` on **`/zed/odom`** with `frame_id: zed_odom`
  and `child_frame_id: zed_camera_link`. The `zed_odom` frame is never in TF;
  P-3 fuses only twist, which is expressed in `child_frame_id`.
- Logs rate, mean latency (`now - stamp`) and drop counts every 10 s.
- Keeps no state that could go stale: if packets stop, it publishes nothing.

**Tests** (`colcon test`, no network): encode/decode round-trips; wrong
magic, wrong version and truncated packets are rejected; stale, future and
out-of-order messages are dropped. A fake sender
(`ros2 run bur_vision_bridge sender_node` against a recorded or synthetic
odometry topic on the Pi itself, over `127.0.0.1`) tests the whole path with
no Jetson.

- **Input:** ZED odometry on the Jetson; X-8 for the real link
- **Output:** `/zed/odom` on the Pi at the ZED's rate; `use_vision_bridge`
  argument in `vehicle.launch.py` (default `false` until P-3 is done)
- **Done when:** with the ZED running, `/zed/odom` on the Pi matches the
  Jetson's topic in rate, with latency under 30 ms, and pulling the cable
  stops it cleanly

### P-3 — ZED visual odometry in the EKF
**Blocked by:** P-5 (and F-5 to be meaningful)

The `odom1` block in `ekf.yaml` is written but commented out, with a
JetPack 7.2 note that is now wrong. Replace both.

**Fuse the ZED's body-frame velocity only (vx, vy) to start.**

```yaml
odom1: /zed/odom
odom1_config: [false, false, false,
               false, false, false,
               true,  true,  false,
               false, false, false,
               false, false, false]
odom1_differential: false
odom1_relative: false
odom1_queue_size: 5
odom1_twist_rejection_threshold: 2.0   # Mahalanobis; tune in C-5
smooth_lagged_data: true               # the bridge delivers late
history_length: 0.5                    # seconds; must exceed bridge latency
```

Why not the ZED's pose:
- **Yaw:** the IMU already owns attitude (`imu1`, relative) and yaw rate
  (`imu0`). A second absolute yaw from a camera that starts at its own origin
  would fight it.
- **x, y:** the ZED's pose lives in `zed_odom`, whose relation to the EKF's
  `odom` is unknown. Fusing it means maintaining that transform. Fusing
  velocity sidesteps it. If drift is too large later, try x and y with
  `odom1_differential: true` **instead of** vx and vy, never both (double
  counting).
- **z:** unobserved until the Bar30 returns (F-4, deferred). Leave it.

**TODO(verify):** `robot_localization` rotates the ZED twist from
`zed_camera_link` into `base_link`. Check whether the installed Jazzy version
also removes the lever-arm term (angular velocity times offset). If not, a
yaw-only rotation of the rig reads as sideways velocity. With the camera in
the nose and the body swaying (M-7, tail_dev), that matters when swimming.

- **Output:** `ekf.yaml` with `odom1` enabled and the stale comment replaced
- **Done when:** the rig test in **Done when** at the top of this page passes

### P-6 — Detections over the bridge
**Blocked by:** D-A4 (the detection contract,
[`bur_autonomy/PLAN.md`](src/gnc/bur_autonomy/PLAN.md#7-decisions-in-this-plan))

`yolo.py` on the Jetson publishes `yolo_msgs/CVDetections` on
`yolo_detections` at about 2 Hz, with the pixel centre in
`bbox.pose.position`, box size in `orientation.x/y`, and no header, width,
height or depth. The autonomy node reads `bur_msgs/CVDetections` on `/vision`.
**Which fields mean what is D-A4, a team decision.** Don't guess it here.

- **Input:** D-A4; P-5's protocol
- **Output:** `msg_type = 2` in `protocol.py`; the sender maps YOLO output to
  the agreed fields and **stamps it with the image time**; the receiver
  publishes `bur_msgs/CVDetections` on `/vision`
- **Done when:** a target held in front of the ZED shows up on `/vision` on
  the Pi with a correct stamp and the agreed fields

### P-4 — Close the detection loop
**Blocked by:** P-6

In the previous stack, detections were stored in a member nothing read, and the
gate target came from a static parameter. **The camera could not influence
behaviour at all.**

- **Output:** `/vision` detections driving `TurnToHeading` and `IsDetected`
- **Done when:** moving the physical target changes what the vehicle does (on
  the bench: what `/swim_command` asks for)

---

## C — Controls

### C-5 — Retune the EKF
**Dry stage blocked by:** P-3 · **Wet stage blocked by:** C-4
([tail_dev.md](tail_dev.md))

Every covariance in `ekf.yaml` is inherited from a vehicle with completely
different dynamics. Keep the `imu0`/`imu1` split: one entry for rates and
accelerations, one in `relative` mode for attitude. It is deliberate.

1. **Dry, now:** measurement variances from data, not guesses. Record the rig
   standing still for 5 minutes (`ros2 bag record /imu /zed/odom`) and use the
   measured variance of each fused field. Then tune
   `process_noise_covariance` for x, y, vx, vy against the 4 m square.
2. **Wet, later:** redo it with swimming recordings, after C-4 filters the
   undulation out of the IMU.

- **Output:** retuned `process_noise_covariance` and measurement variances,
  each with a comment saying which recording it came from
- **Done when:** estimates are stable and converge at a sensible rate, dry
  and then wet

---

## Risks

1. **Visual odometry underwater may simply not track.** Pool floors have little
   texture, and a housing port bends light, so the ZED's factory (in-air)
   calibration gives a scale error. A flat port changes the effective focal
   length by about the refractive index of water, 1.33. **Test the ZED in its
   housing in the pool as early as possible**, before tuning anything else on
   this page. If it fails, the fallback is the IMU alone plus timed swims
   (A-2's `timed` mode), which is what the autonomy leaves already assume.
2. **Clock offset.** Every millisecond of Pi-Jetson offset is a millisecond of
   misplaced measurement. X-8 must be checked at every pool session, not once.
3. **The bridge is temporary code.** Keep it small. If the Jetson ever moves to
   a Jazzy-capable JetPack, delete the package and read the ZED topics
   directly.
