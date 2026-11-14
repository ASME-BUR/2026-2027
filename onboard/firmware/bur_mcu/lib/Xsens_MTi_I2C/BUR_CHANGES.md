# Changes from upstream

Copied from `ASME-BUR/bur-arduino` (`arduino_imu/lib/Xsens_MTi_I2C`, commit
`131181e`), which is Xsens' MIT-licensed Arduino I2C example (v2.0.0, see
`LICENSE`). The 2026-2027 firmware changes are:

1. **All debug output goes to `MTI_LOG` (default `Serial`, the programming
   port), and only when `verbose` is true.** Upstream printed to `SerialUSB`
   unconditionally in `detect`, `configureOutputs`, `goToConfig` and
   `goToMeasurement`. `SerialUSB` is the micro-ROS transport, so those prints
   corrupted the link.
2. **`Xbus::parseNotification` updates `configState` regardless of
   `verbose`.** Upstream set the flag and broke out of the switch only inside
   `if (verbose)`, so with verbose off the cases fell through.
3. **`MTi::sendMessage` builds the frame in a local buffer.** Upstream wrote
   the checksum one byte past the end of the caller's array.
4. **`MTi::readMessages` is bounded** (`kMaxReadsPerCall`) and returns whether
   it parsed a measurement. Upstream looped `while (DRDY)`, which never
   returns if DRDY is stuck high.
5. **Pipe sizes reported by the MTi are clamped to the 256-byte buffers.**
6. The `examples/` directory was dropped.

The driver still uses `Wire1` (the Due's second I2C bus, SDA1/SCL1), and it
still needs the `ASME-BUR/ArduinoCore-sam` fork, whose only change raises the
Wire buffer from 32 to 256 bytes.
