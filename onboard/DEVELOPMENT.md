# DEVELOPMENT

The big-picture roadmap for the vehicle. It says **what** each area of work has
to achieve, not **how**. Implementation choices are made when the work starts,
by the people doing it, and are recorded in the code and the package READMEs.

For setup, structure and how to run things, see [README.md](README.md).

---

## Where we are

Nothing has run on hardware yet. Everything below has been checked only in
simulation or with tests on a PC, and every gain, dimension and limit is a
placeholder.

| Area | State |
|---|---|
| ROS 2 workspace | Ported from the thruster vehicle to the swimming design; builds on Jazzy and Humble |
| Swimming wave | Wave maths written, unit-tested, and viewable in the RViz bench |
| Control | Controller turns speed/turn/depth requests into wave settings; checked in simulation |
| State estimation | EKF set up; IMU input only so far |
| Microcontroller firmware | Gait, servo safety timeout and IMU code written; passes PC tests, never run on a board |
| Autonomy | Behaviour-tree mission manager runs closed-loop against a rough bench simulator |
| Tooling | Setup and build scripts, dev container, bench and vehicle launch files |

---

## Settled

These constrain everything else. Don't reopen them without the team.

- The vehicle **swims** with a 4-joint servo chain bending side to side, so it
  steers by yaw only and can't strafe.
- **Pitch and depth will come from fins** with their own actuators, not from
  the chain.
- **Computers:** a Raspberry Pi 4 (ROS 2, estimation, control, autonomy), one
  Arduino Due (servos, wave timing, sensors), and a Jetson Orin Nano (camera
  and vision).
- **The Due generates the wave itself.** The Pi sends wave settings, not joint
  angles.
- **The Jetson stays on its own ROS version** and talks to the Pi over a
  bridge, not over the shared ROS network.
- **Estimation uses the standard `robot_localization` EKF**, not a custom
  filter.
- **Servo waterproofing** is handled by the mechanical team.

---

## Workstreams

### 1. Sensors and state estimation (current focus)

Give the vehicle a trustworthy estimate of where it is and how it's moving.

- Get the IMU running on the Due.
- Get camera-based motion estimates from the Jetson to the Pi.
- Fuse both into the existing EKF and tune it.

**Done when:** a hand-carried test rig can track its own motion around a
simple course, and losing the Jetson degrades the estimate gracefully instead
of breaking it.

### 2. Swimming (paused until the tail servos are wired)

Make the vehicle swim where it's told.

- Run the gait firmware on the real servo chain.
- Measure how the body actually moves, and replace the placeholder numbers.
- Tune the controller against real behaviour.

**Done when:** in the water, the vehicle follows commanded speed and turn rate,
and goes limp when it loses contact with the Pi.

### 3. Depth and pitch (deferred)

Control depth with fins. Not being worked on, but the existing depth/pitch code
paths stay in place.

- Decide the fin layout.
- Add the depth sensor.
- Close the depth loop.

**Done when:** the vehicle can reach and hold a commanded depth while swimming.

### 4. Perception

Turn what the camera sees into something the mission can act on.

- Get detections from the Jetson to the Pi.
- Feed them into autonomy decisions.

**Done when:** the vehicle reacts to a real target seen by the camera.

### 5. Autonomy

Run competition missions without a human driving.

- Finish validating the mission manager on the bench.
- Run missions on the real vehicle once swimming and estimation work.
- Add navigation that respects the fact that the vehicle can't turn on the
  spot or move sideways.

**Done when:** the vehicle completes a mission task on its own in the pool.

### 6. Infrastructure

Keep the system easy to run, watch and debug.

- Reliable connections between the Pi, the Due and the Jetson.
- Tools to see what the vehicle is doing from the pool deck.

**Done when:** a pool test can be started, monitored and stopped without
digging through terminals.

---

## Open decisions

These belong to the team. Don't guess past them in code; leave a
`TODO(decide)` and raise it.

- Fin count and placement.
- How a mission is started and killed on the vehicle.
- What the vision detections look like when they reach the Pi.
- Whether the message definitions stay inside this repo or move to their own.

---

## Known issues and risks

- **Sensor transforms are published twice with different values** (by the
  URDF and by `ekf.launch.py`), so TF can flip between them.
- **Nothing stops two things driving the servos** at once (the bench gait
  preview and the firmware).
- **Camera odometry may not work underwater** (low-texture pool floor, housing
  distortion). Test the camera in its housing in the pool early.
- **The Pi and Jetson clocks must agree**, or fused camera data lands at the
  wrong time.

---

## Ground rules

- **Don't work around a blocker by guessing.** The guess becomes an assumption
  nobody remembers making.
- **Placeholder values stay marked** (`TODO(hw)` and similar) until someone
  measures the real number.
- **One source of truth per number.** Last year the same limit had three
  different values in three files.
- **Safety first on hardware:** the vehicle must stop when it loses contact
  with the Pi.
- Older task IDs (`D-3`, `F-1`, …) still appear in code comments and package
  docs. They come from the previous task list (in git history before
  2026-11-21). Treat them as historical notes, not commitments.
