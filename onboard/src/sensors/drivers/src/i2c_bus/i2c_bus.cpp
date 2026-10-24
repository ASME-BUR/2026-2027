#include "i2c_bus.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

namespace
{

constexpr int kMaxHandles = 8;

struct Slot
{
    int fd = -1;
    bool used = false;
};

Slot g_slots[kMaxHandles];

Slot * slot(unsigned handle)
{
    if (handle >= static_cast<unsigned>(kMaxHandles)) { return nullptr; }
    if (!g_slots[handle].used) { return nullptr; }
    return &g_slots[handle];
}

}  // namespace

extern "C" {

int pigpio_start(const char * addr, const char * port)
{
    (void)addr;
    (void)port;
    // No daemon. Returning 0 keeps carried-over callers' `if (pi < 0)` checks
    // happy without pretending a connection was made.
    return 0;
}

void pigpio_stop(int pi)
{
    (void)pi;
    for (int i = 0; i < kMaxHandles; ++i) {
        if (g_slots[i].used && g_slots[i].fd >= 0) {
            ::close(g_slots[i].fd);
            g_slots[i].fd = -1;
            g_slots[i].used = false;
        }
    }
}

int i2c_open(int pi, unsigned i2c_bus, unsigned i2c_addr, unsigned i2c_flags)
{
    (void)pi;
    (void)i2c_flags;

    int handle = -1;
    for (int i = 0; i < kMaxHandles; ++i) {
        if (!g_slots[i].used) { handle = i; break; }
    }
    if (handle < 0) { return -1; }

    char path[32];
    std::snprintf(path, sizeof(path), "/dev/i2c-%u", i2c_bus);

    int fd = ::open(path, O_RDWR);
    if (fd < 0) { return -2; }

    if (::ioctl(fd, I2C_SLAVE, static_cast<unsigned long>(i2c_addr)) < 0) {
        ::close(fd);
        return -3;
    }

    g_slots[handle].fd = fd;
    g_slots[handle].used = true;
    return handle;
}

int i2c_close(int pi, unsigned handle)
{
    (void)pi;
    Slot * s = slot(handle);
    if (s == nullptr) { return -1; }
    ::close(s->fd);
    s->fd = -1;
    s->used = false;
    return 0;
}

int i2c_read_word_data(int pi, unsigned handle, unsigned reg)
{
    (void)pi;
    Slot * s = slot(handle);
    if (s == nullptr) { return -1; }

    unsigned char r = static_cast<unsigned char>(reg);
    if (::write(s->fd, &r, 1) != 1) { return -2; }

    unsigned char buf[2] = {0, 0};
    if (::read(s->fd, buf, 2) != 2) { return -3; }

    // Device sends MSB first; pigpio hands back a little-endian word, and
    // MS5837.cpp swaps it. Preserve that contract.
    return (static_cast<int>(buf[1]) << 8) | static_cast<int>(buf[0]);
}

int i2c_read_i2c_block_data(int pi, unsigned handle, unsigned reg,
                            char * buf, unsigned count)
{
    (void)pi;
    Slot * s = slot(handle);
    if (s == nullptr || buf == nullptr || count == 0) { return -1; }

    unsigned char r = static_cast<unsigned char>(reg);
    if (::write(s->fd, &r, 1) != 1) { return -2; }

    ssize_t n = ::read(s->fd, buf, count);
    if (n != static_cast<ssize_t>(count)) { return -3; }
    return static_cast<int>(n);
}

int i2c_write_byte(int pi, unsigned handle, unsigned bVal)
{
    (void)pi;
    Slot * s = slot(handle);
    if (s == nullptr) { return -1; }

    unsigned char b = static_cast<unsigned char>(bVal);
    if (::write(s->fd, &b, 1) != 1) { return -2; }
    return 0;
}

}  // extern "C"
