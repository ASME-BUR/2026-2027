#ifndef BUR_I2C_BUS_H
#define BUR_I2C_BUS_H

// Drop-in replacement for the five pigpiod_if2 functions the Bar30 driver uses.
//
// Signatures and return conventions match pigpio exactly so that MS5837.cpp
// needs no changes beyond its #include. In particular:
//
//   * i2c_read_word_data returns the word LITTLE-endian, as pigpio's SMBus
//     wrapper does. MS5837.cpp relies on this and byte-swaps itself:
//         PROMresult[i] = ((temp & 0x00FF) << 8) | (temp >> 8);
//     Do not "fix" the endianness here without fixing that line too.
//   * i2c_write_byte sends ONE bare byte (a sensor command), not a register
//     write, and returns 0 on success.
//   * the `pi` argument is accepted and ignored. There is no daemon any more.
//
// TODO(port): there is no locking. Fine today because exactly one node opens
// the bus. If a second I2C device ever gets its own node, either share one
// process or add a mutex around the write-then-read pairs, because an
// interleaved transaction on a shared bus returns silent garbage.

extern "C" {

// Kept so carried-over code still compiles. Returns 0 and does nothing.
int pigpio_start(const char * addr, const char * port);
void pigpio_stop(int pi);

int i2c_open(int pi, unsigned i2c_bus, unsigned i2c_addr, unsigned i2c_flags);
int i2c_close(int pi, unsigned handle);

// Writes `reg`, then reads two bytes. Returns the word little-endian, or <0.
int i2c_read_word_data(int pi, unsigned handle, unsigned reg);

// Writes `reg`, then reads `count` bytes into buf. Returns count, or <0.
int i2c_read_i2c_block_data(int pi, unsigned handle, unsigned reg,
                            char * buf, unsigned count);

// Sends a single byte. Returns 0 on success, <0 on failure.
int i2c_write_byte(int pi, unsigned handle, unsigned bVal);

}  // extern "C"

#endif
