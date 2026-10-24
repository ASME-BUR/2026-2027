# 2026-2027

ASME Bruin Underwater Robotics — electrical hardware and software for the
2026-2027 vehicle.

The vehicle swims: propulsion is a chain of servos driving a travelling wave
down the body, replacing the eight-thruster design used through 2025-2026.

## Software

The onboard ROS 2 **Jazzy** workspace lives in [`onboard/`](onboard/).

| | |
|---|---|
| [`onboard/README.md`](onboard/README.md) | Setup, build and run instructions, repository structure, hardware notes, troubleshooting. **Start here.** |
| [`onboard/DEVELOPMENT.md`](onboard/DEVELOPMENT.md) | Every outstanding task with its inputs and outputs, in dependency order. Read before picking up work. |

The workspace builds and the control chain is verified in simulation, but
nothing has run on hardware yet and every gain and dimension is a placeholder.

## Electrical hardware

Schematics and board design files are kept on their own branches rather than on
`main` — for example `leak_sense`.

## Related repositories

The organisation splits repositories by **which computer the code runs on**.
That is worth knowing before concluding something is missing:

| Repository | Runs on |
|---|---|
| [`bur-arduino`](https://github.com/ASME-BUR/bur-arduino) | microcontrollers — `arduino_imu`, `arduino_thruster`, `arduino_merge` |
| [`bur-vision-nano`](https://github.com/ASME-BUR/bur-vision-nano) | the perception box — ZED inference |
| [`bur-vision`](https://github.com/ASME-BUR/bur-vision) | perception |
| [`bur-interfaces`](https://github.com/ASME-BUR/bur-interfaces) | shared message definitions |
| [`bur-onboard`](https://github.com/ASME-BUR/bur-onboard) | the 2025-2026 thruster vehicle's ROS stack — what `onboard/` was ported from |
| [`bur-gazebo`](https://github.com/ASME-BUR/bur-gazebo) | simulation (superseded; we use MuJoCo) |
| [`bur-wiki`](https://github.com/ASME-BUR/bur-wiki) | documentation |
