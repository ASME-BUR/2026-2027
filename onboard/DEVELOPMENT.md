# DEVELOPMENT

What still needs building, in dependency order, with the inputs and outputs of
each task.

For setup, structure and how to run things, see [README.md](README.md).

---

## Current state

| | |
|---|---|
| Builds on Jazzy | **yes** — 10 packages, 0 errors, 0 compiler warnings (verified over 3 clean builds) |
| Control chain verified | **yes** — `SwimCommand` -> `GaitCommand` -> joint angles, envelope confirmed growing head to tail |
| URDF expands | **yes** — 10 links, 9 joints, single unforked chain, zero warnings |
| Run on hardware | **no** |
| Gains, dimensions, limits | **all placeholder** |
| MCU firmware | **does not exist** |
| Autonomy node | **does not exist** |
| Estimation approach | **decided** — reconfigure `robot_localization` (C-9) |
| Depth sensor location | **decided** — on the MCU with the IMU (F-4) |
| Decisions outstanding | **5** — D-2 (board model), D-3 (fins, leaning), D-5, D-6, D-7 |
| Pitch / depth authority | **fins, not confirmed** — see D-3 |

The Humble-to-Jazzy port is effectively done, and it was cheap: the old
`bur_controller` and `bur_msgs` build on Jazzy completely unmodified. The
distro was never the work. **The actuation model is the work**, and the two
genuinely unbuilt pieces are the MCU firmware and the autonomy node.

---

## How to read this

Task IDs are grouped by area: **D** decisions, **M** measurements,
**F** firmware, **C** controls, **A** autonomy, **P** perception,
**X** infrastructure.

Every task states its **Input** (what it consumes, including what it needs from
a human or from hardware) and its **Output** (what it produces, and what counts
as done).

Gaps in the numbering are retired tasks, not mistakes: **D-8** became X-6 once
a grep showed `Command.msg` had no consumers, and **X-4** was folded into F-4
when the depth sensor moved to firmware. IDs are never reused.

`BLOCKED` means waiting on another task, not on effort. Do not work around a
block by guessing — the guess becomes an assumption nobody remembers making,
which is exactly how the previous vehicle ended up with a behaviour tree that
could not load.

### Source markers

The source carries 63 `TODO(...)` markers that cross-reference these tasks:

| Marker | Count | Meaning |
|---|---|---|
| `TODO(hw)` | 31 | needs a measured number from the vehicle |
| `TODO(port)` | 15 | mechanical carry-over not finished |
| `TODO(fish)` | 8 | inherited logic that is wrong for a swimming body |
| `TODO(decide)` | 5 | open design decision |
| `TODO(verify)` | 4 | unconfirmed assumption |

`TODO(decide)` fell from 8 to 5 as D-1, D-2, D-4 and D-9 were settled;
`TODO(hw)` rose as the fin and servo decisions turned design questions into
things that need measuring.

---

## Critical path

Five decisions gate almost everything else. None of them is a coding task.

```
D-1 joint axis layout ─┬─> M-1 dimensions ──> URDF final
                       ├─> D-9 servo type ──> D-2 board ──> F-1 firmware ──┐
                       └─> C-3 pitch fix                                   │
D-3 pitch source ──────┘                                                   │
                                                                           ▼
                                    M-2 servo limits ──> M-3 plant table ──> C-1 ──> wet test
                                                                           │
                                    M-7 IMU noise ──> C-4 ──> C-5 ─────────┘
                                                                           │
                                    A-1 manager ──> A-2 leaves ──> A-3 ────┘
```

**Start here, in this order: D-3, D-1, D-9, D-2.** D-3 comes first because the
pitch mechanism determines the axis layout, not the other way round. Then F-1
and M-1 can run in parallel. Nothing in the water happens before F-1.

---

## D — Decisions

Settle these in a meeting. They need judgement and hardware knowledge, not code.

### D-1 — Joint axis layout
**DECIDED: 4 joints, 4 servos, all revolute about Z.** One horizontal swimming
plane. Applied to `bur.urdf.xacro` and `gait.yaml`.

Consequence worth stating plainly: the chain can produce **yaw moments only**.
It has no pitch authority whatsoever. See D-3.

Also, with only 4 joints the `s^2` amplitude envelope gives joint_0 an
amplitude of exactly zero, so it contributes nothing but turn bias, throwing
away a quarter of the actuation. That was affordable at 6 joints. Give the
envelope a non-zero floor or lower the exponent; see M-4.

### D-2 — MCU board
**PARTLY ANSWERED: an Arduino, and a Raspberry Pi 4B.** The Pi is settled. The
Arduino **model is still needed**, and it is not a formality:

> **`micro_ros_arduino` supports no AVR board on any branch.** An Uno, Mega,
> Nano or Pro Mini cannot run micro-ROS at all. If the board on the shelf is
> one of those, either the board changes or the entire Pi-to-MCU transport does.

Supported on the `jazzy` branch: Teensy 4.1 and 3.x, Portenta H7, ESP32 Dev
Module, Nano RP2040 Connect, OpenCR. Community-supported: Due, Zero, Giga R1,
UNO R4 (WiFi and Minima), Opta, Pico.

The organisation forks both `Audio` (a Teensy library) and `ArduinoCore-sam`
(the Due core), so there is history on both.

- **Input:** which board you actually have
- **Output:** a confirmed micro-ROS-capable board
- **Done when:** it runs the micro-ROS hello-world against the Pi

### D-3 — Where pitch and depth authority come from
**LEADING OPTION: actuated fins (dive planes).** Not yet confirmed — recorded
as the direction, not a decision.

Note first that a *sensor* cannot answer this. The MTi-3 measures attitude; it
produces no moment. Pitch sensing was never in doubt — `ekf.yaml` already fuses
IMU attitude on `imu1`.

With 4 joints all revolute about Z (D-1), the chain produces yaw only. Fins
would therefore be the vehicle's **entire** pitch and depth authority.

#### The thing that decides whether this works

**Fin authority scales with the square of forward speed.** Lift goes as
`½ρv²SC_L`, so halving swim speed *quarters* the fin's effect. At the ~0.3 m/s
this vehicle is aiming for, a palm-sized fin produces on the order of a couple
of hundred millinewtons — enough to move a neutrally buoyant hull slowly, with
very little margin.

That leads to one hard requirement:

> **The vehicle must be ballasted neutrally buoyant, and accurately.** If it is
> even slightly heavy or light, fins at 0.3 m/s will not overcome the error and
> depth will drift regardless of what the controller asks for. Trim accuracy
> matters more here than fin area, and it is a mechanical task with a software
> symptom.

Pitch stability works against you too: a hull with centre of buoyancy well
above centre of mass is pendulum-stable in pitch, and the fins have to fight
that restoring moment. The more pitch-stable the hull, the more fin authority
is needed.

#### Sub-decision: where do the fins sit?

| Placement | Mechanism | Needs a pitch loop? |
|---|---|---|
| Near the nose or tail | Produces a pitching **moment**; the body pitches, then `surge x sin(pitch)` carries it up or down | Yes — depth outer loop, pitch inner loop, as currently coded |
| Near the centre of mass | Produces near-direct **lift** with little body rotation | No — depth maps almost straight to fin angle, which is simpler |

Centre-of-mass placement is the simpler control problem. Extremity placement
gives more moment per unit area. Worth settling alongside the hull layout.

#### What this means for the code

The pitch and depth paths **stay** — do not strip them. But two things change:

- **Done:** the field is now `GaitCommand.fin_angle_rad`, the controller
  publishes it as a fin deflection, and nothing adds it to a chain joint any
  more (C-3). Routing it to a real servo still waits on fin count and placement
  — see the `TODO(port)` in `gait_node.cpp`.
- **C-8 goes back up in priority.** A speed-blind depth loop was tolerable when
  depth barely moved; with fins it is worse than before, because fin authority
  collapses as `v²` and the loop has no idea. It will wind up against an error
  it cannot affect at low speed.

#### And it costs servos

Fins need 1-2 servos of their own on top of the 4 chain joints, which lands on
D-9 (the same SER0067, presumably), on the Arduino's PWM channel count, and on
**X-7** — fin servos need the same sealing solution as the chain, and they sit
further out where sealing is often harder.

- **Input:** hull layout; fin placement; measured buoyancy and pitch stability
- **Output:** confirmed mechanism, fin count and placement, so the fin command
  can be routed to a servo and given a link in the URDF
- **Done when:** depth can be commanded and held at the vehicle's actual
  cruise speed, not just in principle

### D-4 — One MCU or two
**DECIDED: one.** A single Arduino plus a Raspberry Pi 4B.

That board now carries all of this, which is worth seeing together:

| Load | Note |
|---|---|
| 4x servo PWM @ 50 Hz | SER0067, 500-2500 us |
| Xsens MTi-3 AHRS | I2C, SPI or UART |
| Bar30 pressure | I2C, address 0x76, ~41 ms blocking read |
| micro-ROS | USB serial to the Pi |

**If the MTi-3 goes on I2C it shares the bus with the Bar30**, and the Bar30's
blocking read then stalls IMU reads as well as the servo loop. The shim written
for the Pi side flagged exactly this hazard, and it now applies to the firmware
instead: interleaved transactions on a shared bus return silent garbage, so the
bus needs a single owner or a lock. Putting the MTi-3 on SPI or UART sidesteps
it completely and is worth doing for that reason alone.

- **Output:** `micro_ros.launch.py` reduced from two agents to one
- **Done when:** the launch file matches the hardware

### D-5 — Gait phase ownership
**Status:** proposed, needs confirming · **Blocks:** F-1, C-2

The design assumes **the MCU owns the phase accumulator** and subscribes to
`GaitCommand`. `bur_gait` on the Pi is a bench and RViz preview only. Confirm
before writing firmware: if it changes, `ServoChainCommand` becomes the primary
actuation path and needs a hard real-time story.

- **Input:** measured serial latency and jitter over micro-ROS
- **Output:** confirmation, or a revised design
- **Done when:** `F-1` is written against a settled answer

### D-6 — `bur_msgs` vendored or upstreamed
**Status:** open, low urgency

Currently vendored in `src/interfaces/bur_msgs` so the workspace builds offline.
A separate `bur-interfaces` repo only earns its keep if a shore station or
second machine needs the same messages built independently.

- **Input:** whether anything outside this workspace consumes the messages
- **Output:** either a `jazzy` branch on `bur-interfaces`, or a note here that
  vendoring is deliberate

### D-7 — Drop the nav2 dependency
**Status:** open, recommendation is yes · **Affects:** A-1

The previous `bur_autonomy` pulled in `nav2_behavior_tree`, `nav2_util` and
`nav2_common` for exactly two things: `euclidean_distance` and `RewrittenYaml`.
That is a very large apt install to carry on a Pi for a square root.

- **Input:** none
- **Output:** both inlined; nav2 absent from `package.xml`
- **Done when:** `bur_autonomy` builds without nav2 installed


### D-9 — Servo type
**DECIDED: DFRobot SER0067**, 35 kg-cm, 360 deg, 12-bit magnetic encoder, IP54.

| | |
|---|---|
| Stall torque | 28.8 kg-cm @ 6 V, 35.5 kg-cm @ 7.4 V |
| No-load speed | 0.238 s/60 deg @ 6 V = **4.4 rad/s**; 0.192 s/60 deg @ 7.4 V = 5.5 rad/s |
| Control | PWM 500-2500 us over 360 deg, **20 ms frame** |
| Feedback | 12-bit magnetic encoder, internal |
| Sealing | **IP54** |

Three consequences, now tasks rather than decisions:

1. **The frame is 20 ms, so the servo loop is 50 Hz, not 200 Hz.** A standard
   PWM servo cannot be commanded faster than its frame rate. F-1 updated;
   `bur_gait`'s `update_rate: 50` already matched by luck.
2. **The encoder is internal.** It makes the servo position itself better than
   a potentiometer would, but PWM is one-way, so there is no readback path
   unless this part exposes a feedback pin. F-2 is mostly blocked on that.
3. **IP54 is not submersible.** See X-7, which is now the most urgent item on
   this page.

## M — Measurements

These need the physical vehicle. Until they exist, every related number in the
tree is a placeholder.

### M-1 — Body dimensions and masses
**Blocked by:** D-1 · **Feeds:** URDF, MuJoCo

- **Input:** the mechanical design or the built vehicle; a scale; calipers
- **Output:** segment lengths, widths, heights, masses, joint limits written
  into `bur_description/urdf/bur.urdf.xacro`
- **Done when:** the URDF matches the vehicle to within a few mm and the masses
  sum to the measured total

### M-2 — Servo bandwidth and stall current
**Blocked by:** D-9 · **Feeds:** `gait.yaml` (`max_rate_rad_s`, `freq_max`), C-1

Datasheet figures are measured unloaded. What matters is behaviour under
hydrodynamic load.

- **Input:** one servo, a representative tail segment, a current meter, water
- **Output:** maximum sustainable slew rate (rad/s), stall current, thermal
  limit, written into `bur_gait/config/gait.yaml`
- **Done when:** `freq_max` in `bur_controller/config/controller.yaml` reflects
  a rate the servos can actually hold without overheating

### M-3 — The plant table
**Blocked by:** F-1 · **Feeds:** C-1, all controller tuning

**This is the single most valuable measurement on the list.** Without it,
tuning gains is guessing, and commanded speed will not match actual speed.

```
for frequency in 0.5 1.0 1.5 2.0:
  for amplitude in 0.3 0.5 0.7:
    hold the gait, measure steady-state surge speed
for turn_bias in 0.05 0.1 0.2 0.3:
    hold it, measure steady-state yaw rate
```

- **Input:** a swimming vehicle, a pool, a measured distance, a stopwatch or
  the EKF
- **Output:** two lookup tables, committed as a CSV or YAML next to
  `bur_controller/config/controller.yaml`
- **Done when:** C-1 can replace the open-loop `surge_to_freq` gain with it

### M-4 — Amplitude envelope shape
**Blocked by:** F-1 · **Feeds:** `gait.yaml` (`envelope_exponent`)

The current `s^2` envelope is from the carangiform swimming literature, not
from this hull.

- **Input:** the vehicle; the ability to drive one joint at a time
- **Output:** a measured envelope, or a confirmed exponent
- **Done when:** the envelope reflects which joints actually produce thrust

### M-5 — Sensor mounting offsets
**Feeds:** URDF, `bur_localization/launch/ekf.launch.py`

All current offsets are inherited from last year's hull and are certainly wrong.
The IMU one matters most: a wrong yaw offset makes the heading loop chase itself.

- **Input:** the built vehicle; calipers
- **Output:** measured `base_link` -> `imu`, `depth_sensor`, `zed_camera_link`
  transforms
- **Done when:** TF matches the physical layout and the IMU is in the rigid nose

### M-6 — Pool fluid density
**Feeds:** the MCU firmware's `setFluidDensity` call (F-4)

Fresh water is ~997-1000 kg/m^3, sea water ~1025. A 3% density error is a 3%
depth error. The previous repo used 1005.0 in launch and 1029.0 as the node
default, with no note explaining either.

- **Input:** which pool, and whether it is fresh or salt
- **Output:** `fluid_density` set deliberately, with a comment saying which pool

### M-7 — IMU undulation noise
**Blocked by:** F-1 · **Feeds:** C-4, C-5

**The biggest unknown in the estimation stack.** An undulating body injects a
periodic acceleration and angular-rate signal at the tail-beat frequency
straight into the IMU. The thruster vehicle had nothing like this. Fed to the
filter untouched, the estimate will oscillate in sympathy with the gait and the
depth and heading loops will fight it.

- **Input:** the vehicle swimming at a range of commanded tail-beat frequencies;
  a raw `/imu` recording at each
- **Output:** noise spectra; a decision between notch-filtering (the frequency
  is known exactly, because the controller commands it) and inflating process
  noise
- **Done when:** C-4 is implemented against real data rather than a guess

### M-8 — Joint limits from hull geometry
**Blocked by:** D-1, M-1

- **Input:** the hull; the skin or bellows design
- **Output:** `joint_limit_rad` in `gait.yaml` and the URDF `<limit>` elements
- **Done when:** the chain cannot command an angle that fouls the skin

---

## F — Firmware

Nothing exists. `onboard/firmware/` is empty.

> Last year's firmware is **not in this repository, but it does exist**:
> [`ASME-BUR/bur-arduino`](https://github.com/ASME-BUR/bur-arduino). Read it
> before starting F-1 — see X-5. The organisation splits repositories by which
> computer the code runs on, which is why nothing in `bur-onboard` references a
> `.ino` and why the firmware looked missing.

### F-1 — Gait firmware
**Blocked by:** D-1, D-2, D-5, D-9 · **Blocks:** M-3, M-4, M-7, everything wet

The reference wave implementation is already written and deliberately kept free
of ROS so the **same source file** can compile into the firmware:
`src/gnc/bur_gait/include/bur_gait/gait.hpp`. Use it rather than reimplementing,
so the Pi-side preview and the real vehicle cannot drift apart.

- **Input:** `bur_msgs/GaitCommand` over micro-ROS; `gait.hpp`; servo count and
  interface (D-1, D-9)
- **Output:** servo PWM at **50 Hz** (the SER0067's 20 ms frame is the
  ceiling; the 200 Hz in my earlier draft is not achievable with standard PWM),
  phase generated locally; `bur_msgs/ServoChainState` published back
- **Done when:** the chain holds a clean travelling wave on the bench and
  responds to a `GaitCommand` change within one beat

### F-2 — Servo telemetry
**Blocked by:** D-9, F-1

- **Input:** servo feedback, if any is reachable. **TODO(verify):** the
  SER0067 has a 12-bit magnetic encoder, but PWM is one-way — check whether
  this part exposes a feedback pin. If not, `position_rad` stays empty and the
  only telemetry available is current draw, via a shunt on the servo rail.
- **Output:** `ServoChainState` populated with position, current, temperature,
  fault bits; the `fault_flags` bit layout defined in `ServoChainState.msg`
- **Done when:** a stalling servo is visible on a topic rather than by smell

### F-3 — Watchdog and failsafe
**Blocked by:** F-1 · **Priority: do not defer this**

- **Input:** `GaitCommand` arrival times; `MODE_ESTOP` and `MODE_COAST`
- **Output:** chain coasts to neutral after ~300 ms of silence; slew-limited
  transitions so a mode change does not brown out the servo rail
- **Done when:** pulling the USB cable mid-swim makes the body go limp rather
  than hold a turn and swim into a wall

> A held angle on a dead link bends the body into a turn. Coast, never hold.

---

### F-4 — Pressure sensor on the MCU
**DECIDED: the Bar30 moves to the microcontroller** alongside the IMU, and the
~41 ms blocking read is accepted.

This returns the driver to its native habitat: BlueRobotics' MS5837 library *is*
an Arduino library, and what bur-onboard carried was a port of it onto pigpio.
Use the upstream Arduino library rather than back-porting the Pi version.

**The blocking read is accepted, but know what you accepted.** Robot speed is
not what makes it tolerable — the stall is in the servo loop and does not care
how fast the vehicle swims. What makes it tolerable is that (a) a low tail-beat
frequency makes 41 ms a small fraction of a beat, and (b) level tasks mean depth
can be sampled slowly. Watch for a joint freezing mid-beat and then jumping,
which the slew limiter will smooth into a brief wave distortion.

**Cheap mitigation if it does bite:** drop the OSR. The conversion wait is
`2.5 x 2^(8+OSR)` us per conversion, twice per reading:

| OSR | Per reading | Note |
|---|---|---|
| 5 | ~41 ms | what bur-onboard used |
| 2 | ~5 ms | |
| 0 | ~1.3 ms | 30x faster; noisier per sample |

Several low-OSR reads averaged cost about the same total blocking as one
high-OSR read for similar noise, but can be interleaved between servo updates
instead of stalling in one lump.

- **Input:** MS5837 upstream Arduino library; the I2C bus on the chosen board
- **Output:** depth published over micro-ROS as `nav_msgs/Odometry` on
  `/depth_sensor`, matching what `ekf.yaml` expects, with a **diagonal**
  covariance (bur-onboard's was singular)
- **Done when:** depth appears in `/odometry/filtered` and the gait shows no
  visible hitch at the intended depth sample rate

**Fix this inherited bug while you are in there** (was task X-4): bur-onboard
called `setModel(1)` (`MS5837_02BA`) immediately after `init()`, overriding the
model `init()` had just auto-detected from the PROM version word. A Bar30 is a
30BA part, so this was forcing the wrong conversion polynomial — which would
mean every depth reading last year was wrong. Check what detection reports, and
validate against a tape measure.


## C — Controls

### C-1 — Replace the surge gain with the plant table
**Blocked by:** M-3

`bur_controller` currently maps commanded speed to tail-beat frequency through a
single gain (`surge_to_freq: 2.0`). It is a guess.

- **Input:** M-3's lookup tables
- **Output:** table lookup with interpolation in `controller_node.cpp`
- **Done when:** commanded speed matches measured speed within ~20%

### C-2 — Confirm no double-driving
**Blocked by:** F-1, D-5

Once the MCU owns the phase, both it and `bur_gait` subscribe to
`/gait_command`. `use_gait_node:=false` exists for this, but nothing enforces it.

- **Input:** a running vehicle
- **Output:** either a runtime guard, or `use_gait_node` defaulted to `false`
  with the bench launch overriding it
- **Done when:** it is impossible to accidentally drive the servos from two
  places

### C-3 — Fix how pitch is applied
**DONE.** `pitch_cmd` is renamed `fin_angle_rad`, removed from `GaitParams`,
and no longer added to any chain joint angle. The controller publishes it as a
fin deflection; `bur_gait` ignores it, since fins are not part of the gait.

Remaining, moved to the TODO(port) in `gait_node.cpp`: routing the fin command
to an actual servo, which needs D-3 to settle fin count and placement first.

<details><summary>Original task, for context</summary>

**Blocked by:** D-1, D-3 · **Confirmed defect**

`gait.hpp` adds `pitch_cmd` as a uniform offset to every joint. A uniform offset
in the pitch plane is a body bend, not a pitch moment — and the smoke test
already showed it saturating the tail joints at their limit.

- **Input:** D-1's axis layout (settled: Z-only), D-3's mechanism (leading:
  fins)
- **Output:** `pitch_cmd` **removed from the chain joint angles entirely** — a
  Z-only chain cannot pitch, so adding it there only steals joint range from
  the wave. It becomes a fin deflection the MCU routes to the fin servo.
  `turn_bias` is then the only offset competing with wave amplitude, which
  should be budgeted against `joint_limit_rad` so a hard turn cannot saturate
  the tail.
- **Done when:** commanding full pitch and full turn together leaves usable
  wave amplitude at the tail

</details>

### C-4 — Filter undulation out of the IMU
**Blocked by:** M-7

- **Input:** M-7's noise spectra; the commanded tail-beat frequency, which the
  controller already knows exactly
- **Output:** a notch or low-pass stage ahead of the EKF, or justified process
  noise inflation in `ekf.yaml`
- **Done when:** `/odometry/filtered` stays stable while the body is actively
  swimming

### C-5 — Retune the EKF
**Blocked by:** C-4

Every covariance in `ekf.yaml` is inherited from a vehicle with completely
different dynamics.

- **Input:** tank test recordings; C-4's filtering
- **Output:** retuned `process_noise_covariance` and measurement variances
- **Done when:** depth and attitude estimates are stable and converge at a
  sensible rate

### C-6 — Teleop depth setpoint from odometry
**Small, self-contained, unblocked**

`joy_to_swim.py` resets its integrated depth setpoint to `0.0` on disarm, which
is only correct at the surface. It should read actual depth.

- **Input:** `/odometry/filtered`
- **Output:** setpoint initialised from measured depth on arm
- **Done when:** re-arming at depth does not command a surface run

### C-7 — Unit-test the gait function
**Small, self-contained, unblocked**

`bur_gait::jointAngles` is a pure function with no ROS dependency, which makes it
the cheapest thing in the stack to test properly — and the phase and envelope
maths is exactly where an off-by-one hides.

- **Input:** none
- **Output:** a `ament_cmake_gtest` target covering envelope monotonicity, phase
  lag direction, joint-limit clamping, zero-amplitude and wavelength guards
- **Done when:** the build fails if the wave maths regresses

---

### C-8 — Make the depth loop speed-aware
**Confirmed defect** · **Blocked by:** D-3 · `controller_node.cpp:171-175`

> **Priority raised again.** Briefly deprioritised on the grounds that level
> tasks need little depth authority. Fins make it worse, not better: fin lift
> scales as `v²`, so authority collapses at low speed while the loop carries on
> demanding depth changes it cannot produce. Whatever the mechanism, this loop
> has to know the vehicle's forward speed.

The depth loop converts depth error into a pitch setpoint, which is the right
cascade — but it does so with **no reference to forward speed**. On a body whose
only depth authority is pitching while swimming, vertical rate is proportional
to `surge x sin(pitch)`. At zero surge, pitch achieves nothing, so the loop
winds the integrator against an error it cannot affect and the vehicle sits
nose-down going nowhere.

Options depend on D-3:

- **chain-bend pitch:** gate or scale the depth loop by measured surge speed,
  and have `HOLD` mode refuse depth commands it cannot honour
- **dive planes:** same coupling, but the authority is a plane angle rather
  than a body bend
- **buoyancy engine or ballast:** the coupling disappears entirely, depth works
  at zero speed, and this task becomes "add a second actuator path"

- **Input:** D-3's mechanism; `state_.twist.twist.linear.x`; the plant table
  from M-3
- **Output:** a depth loop that cannot demand depth change the vehicle has no
  authority to produce, and does not wind up when it has none
- **Done when:** commanding a depth change at zero surge either refuses, or
  commands forward motion to achieve it — but does not silently pitch and stall

### C-9 — Repoint the EKF at the new sensor topology
**DECIDED: reconfigure `robot_localization`,** not a custom filter. The ported
`ekf.yaml` already fuses exactly the three sensor classes planned (depth, IMU
rates + attitude, visual odometry), so this is a remap and a retune rather than
a rewrite. A custom ESKF was considered — the organisation forks
[`ASME-BUR/eskf`](https://github.com/ASME-BUR/eskf) — and rejected as
semester-scale work for a problem a config change covers.

- **Input:** depth arriving over micro-ROS from the MCU rather than from a Pi
  node; ZED **Mini** odometry from the Jetson once P-1 lands
- **Output:** `odom0` pointed at the micro-ROS depth topic; `odom1` uncommented
  and pointed at the ZED Mini's `odom`; covariances retuned per C-5
- **Done when:** `/odometry/filtered` is stable on depth + IMU alone, and stays
  stable when the ZED is added

> Keep the two-`imu0`/`imu1` split from bur-onboard: one entry for rates and
> accelerations, one in `relative` mode for absolute attitude. That is
> deliberate, not redundant.


## A — Autonomy

### A-1 — Manager node
**Blocked by:** D-3, D-7 · **Blocks:** A-2, A-3

Deliberately not ported. The previous node emitted a goal **pose**, assuming the
vehicle could translate in any direction and hold station. Porting it before the
motion interface is settled would bake that assumption back in.

- **Input:** `/odometry/filtered`, `/vision` (`CVDetections`), a behaviour tree
  XML file
- **Output:** `bur_msgs/SwimCommand` on `/swim_command`; BehaviorTree.CPP v4
  tree ticked at 10 Hz
- **Done when:** `fish_tree.xml` loads, ticks, and drives the vehicle in the
  bench preview

### A-2 — Behaviour tree leaf nodes
**Blocked by:** A-1

`behavior_trees/fish_tree.xml` names the target leaf set; none of it exists.
There is deliberately **no `GoToPose`** — reaching a point means turning to face
it and then swimming, which is two behaviours, not one.

| Leaf | Input | Output |
|---|---|---|
| `HoldDepth` | target depth, tolerance, timeout | `SwimCommand.depth_m`; SUCCESS inside tolerance |
| `TurnToHeading` | heading, or a detection label | `SwimCommand.yaw_rate_rps`; SUCCESS when aligned |
| `SwimForward` | distance, speed | `SwimCommand.surge_mps`; SUCCESS on odometry delta |
| `SearchPattern` | sweep angle, timeout | alternating yaw rate; SUCCESS on detection |
| `Surface` | — | depth 0, zero surge; SUCCESS at the surface |
| `IsDetected` | label | condition on `/vision` |

- **Done when:** every node referenced by any XML in `behavior_trees/` is
  registered with the factory

### A-3 — Tree load test
**Blocked by:** A-2 · **Small, high value**

The previous `comp_tree.xml` could never load — it referenced four node types
nobody had registered, and nothing caught it.

- **Input:** every XML in `behavior_trees/`
- **Output:** a test that loads each one against the real factory
- **Done when:** the build fails if a tree references an unregistered node

### A-4 — Curvature-aware planner
**Blocked by:** A-2

The old `bur_planner` (on `bur-onboard`'s `waypoint-planner` branch) linearly
interpolated 20 waypoints between poses and pushed them sideways around
obstacles. It ignores a turning radius, so it is the wrong shape for this
vehicle. Not carried over.

- **Input:** current pose, goal pose, minimum turning radius, obstacles
- **Output:** a path the vehicle can physically follow — Dubins arcs are the
  cheap correct answer for something that cannot strafe
- **Done when:** a planned path never asks for a turn tighter than the vehicle
  can execute

---

## P — Perception

Nothing in this workspace. The ZED and YOLO live on the Jetson, and their code
lives in [`ASME-BUR/bur-vision-nano`](https://github.com/ASME-BUR/bur-vision-nano)
(`zed_inference.py`) and [`ASME-BUR/bur-vision`](https://github.com/ASME-BUR/bur-vision).
Read those before P-2; the export pipeline you need to rebuild is in there.

### P-1 — Jetson to JetPack 7.2
**Blocks:** P-2, P-3

JetPack 6.2.1 is Ubuntu 22.04 and has no Jazzy packages. JetPack 7.2 is Jetson
Linux 39.2 on Ubuntu 24.04 and covers Orin, so Jazzy installs natively. ZED SDK
5.5 ships a JetPack 7.2 installer listing Orin, and the ZED ROS 2 wrapper
supports Jazzy. The 6.2.1 firmware already meets the upgrade prerequisite.

**Install to a spare microSD and keep the 6.2.1 card.** A spare card turns an
irreversible reflash into a thirty-second swap.

- **Input:** a spare microSD (64 GB UHS-1+), a USB stick (16 GB+) for the ISO,
  a host machine
- **Output:** Jetson on JetPack 7.2 running Jazzy, ZED SDK 5.5 installed
- **Done when:** the ZED streams and `ros2 topic list` shows the wrapper's topics

> The risk here is not ROS. It is that JetPack 7.2 has no official NVIDIA
> PyTorch wheel, does not ship cuDNN/NCCL/cuSPARSELt, and jumps CUDA 12.6 to
> 13.2 — which invalidates every cached TensorRT engine. Spike it before
> committing.

### P-2 — Re-export the YOLO model
**Blocked by:** P-1

- **Input:** the trained weights; TensorRT on CUDA 13.2
- **Output:** a working engine file, and a documented export command
- **Done when:** inference runs at a usable frame rate on JetPack 7.2

### P-3 — Re-enable ZED odometry in the EKF
**Blocked by:** P-1

The `odom1` block in `ekf.yaml` is commented out and ready.

- **Input:** `zed/zed_node/odom` reaching the Pi over Jazzy-to-Jazzy DDS
- **Output:** `odom1` uncommented; `ROS_DOMAIN_ID` pinned; multicast confirmed
  across the tether
- **Done when:** x and y stop drifting in `/odometry/filtered`

### P-4 — Close the detection loop
**Blocked by:** A-2, P-1 · **Pre-existing gap**

In the previous stack, `vision_callback` stored detections in a member that
nothing read, and the gate target came from a static parameter. **The camera
could not influence behaviour at all.**

- **Input:** `/vision` (`CVDetections`)
- **Output:** detections updating target poses used by `TurnToHeading` and
  `IsDetected`
- **Done when:** moving the physical target changes what the vehicle does

---

## X — Infrastructure

### X-1 — Rewrite the topic viewer
**Small, unblocked, genuinely useful**

`bur-onboard` had a `topic_viewer.py` TUI printing live command, control effort
and thruster values. It earned its keep during pool tests. Not carried over
because every topic it watched is gone.

- **Input:** `/swim_command`, `/gait_command`, `/servo_chain_state`,
  `/odometry/filtered`
- **Output:** a one-screen live view in `bur_bringup`
- **Done when:** someone on the pool deck can see what the vehicle thinks it is
  doing without running four `ros2 topic echo` windows

### X-2 — udev rules for serial devices
**Small, unblocked** · **Blocked by:** D-4 for the final device list

`/dev/ttyACM*` numbering is not stable across reboots.

- **Input:** USB serial numbers of the MCU boards
- **Output:** a udev rule giving `/dev/bur_servo` and `/dev/bur_imu`;
  `micro_ros.launch.py` defaults updated
- **Done when:** device assignment survives a reboot and a replug

### X-3 — RViz config
**Small, unblocked**

`bench.launch.py` opens RViz with no configuration, so it shows nothing useful.

- **Input:** the URDF; `/joint_states`; TF
- **Output:** a `.rviz` file with RobotModel and TF enabled, installed and
  referenced by the launch file
- **Done when:** `bench.launch.py` opens on a visible, animating vehicle


### X-5 — Read last year's firmware
**Small, unblocked.** Do this before F-1, not after.

[`ASME-BUR/bur-arduino`](https://github.com/ASME-BUR/bur-arduino) contains
`arduino_imu/`, `arduino_merge/` and `arduino_thruster/` — which matches the
two-agent split in `micro_ros.launch.py` exactly (`ttyACM0` thrusters,
`ttyACM1` IMU), plus an apparent combined build.

- **Input:** that repository
- **Output:** a short written summary of what is reusable — serial setup, baud
  rate, micro-ROS initialisation, watchdog behaviour (if any), and which board
  it actually targeted
- **Done when:** F-1 and D-2 are informed by it rather than guessing

---

### X-6 — Delete the unused `Command.msg`
**Small, unblocked.** Was decision D-8 ("narrow it"), which is moot: a grep
shows **nothing in the workspace consumes `Command.msg`**. `bur_controller`
takes `SwimCommand`, and the old aggregate state+setpoint message has no
remaining consumer.

It still carries a sway field the vehicle physically cannot execute, so leaving
it in place is an invitation.

- **Input:** none
- **Output:** `Command.msg` removed from `msg/` and from
  `rosidl_generate_interfaces` in `bur_msgs/CMakeLists.txt`
- **Done when:** the workspace builds without it

> `ServoChainState`, `CVDetection` and `CVDetections` also have no consumers
> yet, but those are interface contracts for F-2, P-4 and X-1. Keep them.

### X-7 — Waterproof the servos
**URGENT. Blocks any wet test.** The SER0067 is **IP54**: dust-protected and
splash-resistant. The second digit is the one that matters, and 4 means
splashing water, not immersion. Immersion starts at IPx7.

DFRobot's own product title says "Waterproof", which is how this gets missed.
It is not submersion-rated, and four flooded 35 kg-cm servos is an expensive
first pool test.

Options, roughly in order of how much the team controls the outcome:

- **Dry hull with shaft seals.** The usual answer for fish robots: servos stay
  inside and only the output shafts cross the boundary, through rotary seals.
  Adds mechanical work and seal friction, but the actuators are genuinely dry.
- **Oil-fill or pot the servos.** Common in hobby submarine builds. Pressure
  compensates rather than resists. Voids the warranty.
- **Buy submersible servos.** IPx7 or higher, or purpose-built underwater
  servos. More money, far less integration risk.

Fin servos (D-3) need the same treatment and are often harder, sitting
further outboard where there is less hull to seal against.

- **Input:** the hull design; expected depth (a pool is shallow, which helps);
  fin count from D-3
- **Output:** a sealing approach, built and leak-tested on **one** joint before
  any servo goes near water
- **Done when:** a single sealed joint has been cycled underwater for longer
  than a mission with no ingress

> Do not skip the single-joint test. Finding this out with all four installed
> costs four servos instead of one.

## Appendix: inherited failure modes

Trimmed to the ones still live. Defects this tree has already fixed in code
(uninitialised PID state, a singular covariance on the Pi, a non-self-contained
header, undeclared transitive dependencies) are documented at the point they
were fixed rather than listed here.

1. **A behaviour tree referencing unregistered nodes loads as a silent
   failure.** `comp_tree.xml` could never run — four of its node types were
   never registered with the factory, and nothing caught it. This is the whole
   reason A-3 exists.
2. **A tree ticked before it is built.** The old manager started its tick timer
   in the constructor but built the tree afterwards in `main()`. Watch for this
   in A-1.
3. **Parameter name mismatches fail silently.** `params.yaml` declared
   `gate_target`; the node read `gate_position`. The YAML was ignored and a
   hardcoded default won, with no warning. Declare and read from one list.
4. **Losing the actuation node did not stop the actuators.** `shutdown()`
   appended to a vector it never cleared, and the destructor built a zero
   message it never published. F-3 is the equivalent on this vehicle, and it is
   the one task on this list that can damage hardware if skipped.
5. **The deadman was a joystick button**, so the controller published zeros
   forever with no gamepad attached — including during autonomous runs. This
   vehicle uses a stale-command timeout instead, which is why autonomy and
   teleop behave identically.
6. **Covariance is easy to get wrong in a way nothing notices.** The old Bar30
   node filled all 36 entries of its 6x6 with the same value, which is
   singular. It never bit because the EKF read only the z term. F-4
   reimplements this in firmware — use a diagonal.
7. **The same constant disagreed in three places.** `max_torque` was 4000.0 in
   the shipped config, 40.0 in the generator that wrote it, and 8.0 in the node
   default. One source of truth per number.

## Appendix: Jazzy migration

**Complete and verified.** The old `bur_controller` and `bur_msgs` build and run
on Jazzy unmodified, so the two things that looked like blockers are not: tf2's
`.h` headers are forwarding shims with no warning, and a package's
`CMAKE_CXX_STANDARD 14` is overridden because `rclcpp` exports `cxx_std_17`.

One live item: `tf2_geometry_msgs.h` *was* removed outright in Jazzy. This tree
already uses the `.hpp` spelling everywhere, but grep for the old name before
porting any further package in from `bur-onboard`.
