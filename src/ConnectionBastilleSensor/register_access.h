/**
 * @file register_access.cpp
 * @author Jon Szymaniak <jon.szymaniak@nuand.com>
 * @brief Raw register access helper routines for ConnectionBastilleSensor
 */

#ifndef CONNECTION_BASTILLE_SENSOR_REGISTER_ACCESS_H_
#define CONNECTION_BASTILLE_SENSOR_REGISTER_ACCESS_H_

#include <fcntl.h>
#include <unistd.h>
#include <cerrno>

#include "LMS7002M.h"
#include "nuand_regs.h"

#ifdef REG_DEBUG
#   include <cstdio>
#   define DBG_MSG(...) fprintf(stderr, "[BastilleSensor:RegisterDebug] " __VA_ARGS__)
#else
#   define DBG_MSG(...)
#endif

static inline int reg_read(int fd, uint16_t addr, uint16_t *data)
{
    ssize_t n = pread(fd, data, sizeof(data[0]), addr);

    if (n == sizeof(data[0])) {
        DBG_MSG("Read  [0x%04x] = 0x%04x\n", addr, *data);
        return 0;
    } else if (n < 0) {
        return errno;
    } else {
        return -EIO;
    }
}

// reg_read with exceptions thrown on error
static inline uint16_t reg_read_ex(int fd, uint16_t addr)
{
    uint16_t data;
    const int status = reg_read(fd, addr, &data);
    if (status != 0) {
        throw status;
    }

    return data;
}


static inline bool reg_write(int fd, uint16_t addr, uint16_t data)
{
    ssize_t n = pwrite(fd, &data, sizeof(data), addr);
    DBG_MSG("Write [0x%04x] = 0x%04x, result = %zd\n", addr, data, n);

    if (n == sizeof(data)) {
        return 0;
    } else if (n < 0) {
        return errno;
    } else {
        return -EIO;
    }
}

// reg_write with exceptions thrown on error
static inline void reg_write_ex(int fd, uint16_t addr, uint16_t data)
{
    const int status = reg_write(fd, addr, data);
    if (status != 0) {
        throw status;
    }
}

// Low-level channel select
static inline int select_channel(int fd, lime::LMS7002M::Channel ch)
{
    uint16_t regval;
    int status = reg_read(fd, lime::MAC.address, &regval);
    if (status != 0) {
        return status;
    }

    regval &= ~(0x3 << lime::MAC.lsb);
    regval |=  (ch  << lime::MAC.lsb);

    return reg_write(fd, lime::MAC.address, regval);
}

// Low-level channel select, throws exception on error
static inline void select_channel_ex(int fd, lime::LMS7002M::Channel ch)
{
    const int status = select_channel(fd, ch);
    if (status != 0) {
        throw status;
    }
}

/*
 * The IConnection interface operates on "ready to be shifted out the bus" word.
 *
 * However, our abstraction is (address, data) accesses, so we have to
 * unpack and repack every request handled by our IConnection.
 */

static inline void unpackSpiXfer(const uint32_t xfer, uint16_t &addr, uint16_t &data)
{
    addr = (xfer >> 16) & 0x7fff;   // Bit 15 is actually a read/write bit
    data = xfer & 0xffff;
}

static inline void packSpiXfer(uint32_t &xfer, uint16_t addr, uint16_t data)
{
    xfer = (addr << 16) | data;
}

#endif
