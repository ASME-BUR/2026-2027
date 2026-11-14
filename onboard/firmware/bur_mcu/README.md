# bur_mcu: Arduino Due firmware

The one MCU on the vehicle. It covers F-1 (gait), F-3 (watchdog) and F-5 (IMU)
in [`DEVELOPMENT.md`](../../DEVELOPMENT.md).

```
/gait_command (bur_msgs/GaitCommand, 10 Hz) ──> phase accumulator + gait.hpp ──> 4 chain servos, 50 Hz PWM
MTi-3 on Wire1 (100 Hz)                     ──> /imu (sensor_msgs/Imu, frame "imu")
chain status (10 Hz)                        ──> /servo_chain_state (bur_msgs/ServoChainState)
```

**It has never run on a Due.** It compiles for the Due, and its chain logic
passes host unit tests. Everything that needs hardware is still open; see
[Not verified yet](#not-verified-yet).

## Build

Use PlatformIO on Linux (Ubuntu 22.04/24.04 or WSL). `micro_ros_platformio`
does not build on native Windows.

```bash
pip install platformio                 # or the PlatformIO IDE for VS Code
cd onboard/firmware/bur_mcu
pio test -e native                     # host unit tests, no hardware (seconds)
pio run                                # firmware for the Due (first run builds micro-ROS: a few minutes)
pio run -t upload                      # flash over the Native USB port
```

- **Distro:** `board_microros_distro = jazzy` must match the micro-ROS agent
  on the vehicle computer. That computer is still Humble / 22.04. Either run a
  Jazzy agent (in the dev container), or set `humble` here and rebuild.
- **Message changes:** `scripts/sync_bur_msgs.py` copies
  `src/interfaces/bur_msgs` into `extra_packages/` before every build. When the
  messages change, it deletes the built micro-ROS library so it is rebuilt.
  Never edit `extra_packages/`.
- **Shared gait code:** `gait.hpp` is compiled straight from
  `src/gnc/bur_gait/include`. The firmware's gait shape (`n_joints`, envelope,
  joint limit, slew rate) is `bur_gait::ChainConfig`'s defaults, the same
  numbers as `gait.yaml` and the URDF. Change all of them together.

## Run against the Pi

```bash
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyACM0   # the Due's Native port
ros2 topic echo /servo_chain_state     # `sequence` counts received GaitCommands
ros2 topic hz /imu                     # ~100 Hz once the clock has synced
ros2 launch bur_bringup vehicle.launch.py use_gait_node:=false use_micro_ros:=false
```

Launch with `use_gait_node:=false`. With the bench gait node running too,
both would compute the wave (C-2).

## Wiring

| Signal | Due pin | Notes |
|---|---|---|
| Chain servo 0 (head) to 3 (tail) | D4, D5, D6, D7 | `kChainPins` in `include/config.h`. TODO(hw): confirm against the harness |
| MTi-3 SDA / SCL | SDA1 / SCL1 (`Wire1`) | No pull-ups on the Due for Wire1. MTi-#-DK newer than rev 2.3 has them on board; older ones need 2.7 kOhm to 3.3 V |
| MTi-3 DRDY | D3 | |
| MTi-3 power | 3.3 V, GND | 5 V forces the MTi into USB mode |
| micro-ROS | Native USB port | The Programming port prints debug text at 115200 |
| Status LED | D13 | On while the agent is connected |
| Bar30 (deferred, F-4) | SDA / SCL (`Wire`, D20/D21) | Its own bus, so its blocking read cannot stall the IMU |

**The IMU is on I2C (`Wire1`), not UART or SPI** (decided 2026-11-07). `Wire1`
is the Due's second I2C bus, so the IMU never shares a bus with the Bar30.
This also reuses last year's driver, which is known to work on the Due. A
100 Hz MTi packet takes about 1.5-2 ms to read at 400 kHz, and it is read
only when DRDY is high.

The Due is 3.3 V and its pins are not 5 V tolerant. Check that the SER0067
accepts a 3.3 V PWM signal; if it doesn't, add a level shifter.

## How it behaves

- **The servo tick runs every 20 ms, timed with `micros()`.** micro-ROS and
  the IMU run in the time left over. Every call that waits on the agent has a
  timeout of a few milliseconds. The tick does not depend on the link.
- **Command timeout (F-3).** If no `GaitCommand` arrives for 300 ms, the chain
  straightens to zero. This happens whatever the link state is: unplugged
  USB, a dead agent, or a crashed controller node with the agent still up.
  The return to zero is slew-limited, and the phase is kept, so swimming
  resumes where it stopped. ESTOP, COAST and unknown modes behave the same
  way. A command containing NaN or inf is rejected and treated as COAST.
- **Hardware watchdog.** If `loop()` stops running for 1 s (a hung USB write
  or a stuck I2C bus), the SAM3X resets. The servos come back at neutral.
- **At boot,** the servos are attached and set to neutral before anything
  slow happens. With no MTi present, the firmware runs without `/imu`.
- **`/imu` is published only after the micro-ROS clock has synced.** Without
  the sync, stamps would be time since boot and the EKF would discard them.
- **`ServoChainState`:** position, current and temperature stay empty,
  because the SER0067 gives no feedback over PWM (F-2). `fault_flags` stays 0
  until F-2 defines its bits.
- **`fin_angle_rad` is received and ignored** until D-3 settles the fins.

## Not verified yet

Each item needs a Due, and most need the vehicle.

- [ ] micro-ROS Jazzy actually connects on the Due. Its support for the Due
      comes from the community, and last year only Humble was proven.
- [ ] One gait tick plus servo writes fits easily inside 20 ms. Measure with
      a scope on a spare pin, or with `micros()` printed to the Programming
      port.
- [ ] Pulling the USB cable mid-wave straightens the chain within about 0.5 s
      (the F-3 done-when criterion).
- [ ] `kChainDirection` and `kChainTrimUs`: a positive `turn_bias` bends every
      joint to port, and zero is physically straight.
- [ ] SER0067 at 3.3 V PWM, and what it does with no pulses (during a
      watchdog reset).
- [ ] MTi detected on `Wire1`. `/imu` at about 100 Hz, and rotating to port
      gives positive yaw rate (F-5, TODO(verify) in `src/imu.cpp`).
- [ ] The `/imu` QoS matches robot_localization's subscription
      (`ros2 topic info -v /imu`). The firmware publishes best-effort.
- [ ] The IMU covariances against the MTi-3 datasheet (TODO(verify) in
      `src/imu.h`).
