/**
 * @file register_access.cpp
 * @author Jon Szymaniak <jon.szymaniak@nuand.com>
 * @brief ConnectionBastilleSensor register access functionality
 */

#include "ConnectionBastilleSensor.h"
#include "register_access.h"

using namespace lime;

int ConnectionBastilleSensor::RegWrite(int fd, const uint32_t *xfer, size_t count)
{
    uint16_t addr, data;
    int status;

    for (size_t i = 0; i < count; i++) {
        unpackSpiXfer(xfer[i], addr, data);
        status = reg_write(fd, addr, data);
        if (status != 0) {
            ReportError(EIO, "SPI write %zd / %zd failed.\n", i + 1, count);
            return -1;
        }
    }

    return 0;
}

int ConnectionBastilleSensor::RegRead(int fd, const uint32_t *xferIn, uint32_t *xferOut, size_t count)
{
    uint16_t addr, data;
    int status;

    for (size_t i = 0; i < count; i++) {
        unpackSpiXfer(xferIn[i], addr, data);
        status = reg_read(fd, addr, &data);
        if (status != 0) {
            ReportError(EIO, "SPI write %zd / %zd failed.\n", i + 1, count);
        } else {
            packSpiXfer(xferOut[i], addr, data);
        }
    }

    return 0;
}
