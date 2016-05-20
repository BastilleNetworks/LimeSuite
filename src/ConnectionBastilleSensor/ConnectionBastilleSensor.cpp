/**
 * @file ConnectionBastilleSensor.h
 * @author Jon Szymaniak <jon.szymaniak@nuand.com>
 * @brief LMS7002M connections on Bastille Sensor via nlms7 kernel module
 */

#include <fstream>
#include <fcntl.h>
#include <unistd.h>

#include "ConnectionBastilleSensor.h"
#include "register_access.h"


// Version of this module
#define CONNECTION_BASTILLE_SENSOR_VERSION  "0.0.1"

using namespace lime;

ConnectionBastilleSensor::ConnectionBastilleSensor(const std::vector<std::string> &devices)
{
    for (size_t i = 0; i < devices.size(); i++) {
        const std::string device(devices.at(i));
        int fd = open(device.c_str(), O_RDWR);
        if (fd < 0) {
            const std::string msg = std::string("Failed to open ") + device;
            ReportError(errno, msg.c_str());
        } else {
            // Just use a simple 0-indexing for the "addresses"
            _reg_fd.insert(std::map<int, int>::value_type(i, fd));
        }
    }
}

ConnectionBastilleSensor::~ConnectionBastilleSensor()
{
    for (const auto &fd : _reg_fd) {
        close(fd.second);
    }
}


int ConnectionBastilleSensor::TransactSPI(const int addr, const uint32_t *writeData, uint32_t *readData, const size_t count)
{
    if (not this->IsOpen()) {
        ReportError(ENOTCONN, "Connection is not open");
        return -1;
    }

    auto device = _reg_fd.find(addr);
    if (device == _reg_fd.end()) {
        ReportError(ENODEV, "Connection does not have a file descriptor for addr=%d", addr);
        return -1;
    }

    const int fd = device->second;

    if (writeData != nullptr && readData == nullptr) {
        return RegWrite(fd, writeData, count);
    } else if (writeData != nullptr && readData != nullptr) {
        return RegRead(fd, writeData, readData, count);
    } else {
        ReportError(EINVAL, "Invalid SPI transaction provided");
        return -1;
    }
}

DeviceInfo ConnectionBastilleSensor::GetDeviceInfo()
{
    DeviceInfo d;
    int status = -1;

    d.deviceName = "Bastille Sensor";
    d.expansionName = "N/A";

    // All of the devices are using the same LMS7 memory-mapped interface,
    // so just use the currently selected device to query the revision
    // of this FPGA block
    if (this->IsOpen()) {
        uint16_t major, minor, patch;
        try {
            int fd = _reg_fd.at(0);
            major = reg_read_ex(fd, VER_MAJOR_ADDR);
            minor = reg_read_ex(fd, VER_MINOR_ADDR);
            patch = reg_read_ex(fd, VER_PATCH_ADDR);

            d.firmwareVersion = std::to_string(major) + "." +
                                std::to_string(minor) + "." +
                                std::to_string(patch);

            status = 0;

        } catch (int err) {
            ReportError(err, "Failed to retrieve MM-IF version");
            status = err;
        }
    }

    if (status != 0) {
        d.firmwareVersion = "Unknown (Requires connection)";
    }

    std::ifstream hw_rev_file;
    std::string hw_rev;

    try {
        hw_rev_file.open("/proc/device-tree/version", std::ifstream::in);
        if (hw_rev_file.is_open()) {
            std::getline(hw_rev_file, hw_rev);
            hw_rev_file.close();
        }
    } catch (...) {
        ReportError(EIO, "Failed to read HW version file");
    }

    if (hw_rev.size() == 0) {
        d.hardwareVersion = "Unknown";
    } else {
        d.hardwareVersion = "Rev. " + hw_rev;
    }

    // Use the protocol field to version this module
    d.protocolVersion = CONNECTION_BASTILLE_SENSOR_VERSION;

    // TODO: Retrieve serial number and shoehorn it into 32-bits?
    d.boardSerialNumber = 0;

    // Devices not on the platform
    d.addrSi5351  = -1;
    d.addrADF4002 = -1;

    if (this->IsOpen()) {
        for (auto handle : _reg_fd) {
            const int addr = handle.first;
            d.addrsLMS7002M.push_back(addr);
        }
    }

    return d;
}

int ConnectionBastilleSensor::DeviceReset()
{
    int status, ret = 0;

    if (not this->IsOpen()) {
        ReportError(ENOTCONN, "Connection is not open");
        return -1;
    }

    for (auto handle : _reg_fd) {
        status = this->DeviceReset(handle.second);
        ret = (ret == 0) ? status : ret;    // Track first error code
    }

    return ret;
}

int ConnectionBastilleSensor::DeviceReset(int fd)
{
    int status;

    // Strobe HW reset line
    try {
        uint16_t regval = reg_read_ex(fd, GPOUT_CTRL_ADDR);
        reg_write_ex(fd, GPOUT_CTRL_ADDR, regval & ~GPOUT_CTRL_LMS7_RESETN);
        usleep(1000);
        reg_write_ex(fd, GPOUT_CTRL_ADDR, regval | GPOUT_CTRL_LMS7_RESETN);
    } catch (int error) {
        ReportError(error, "Failed to reset device.");
        return error;
    }

    // Perform platform-specific initializations
    status = this->InitLimeLight(fd);
    if (status != 0) {
        return status;
    }

    status = this->InitLDO(fd);
    if (status != 0) {
        return status;
    }

    status = this->InitXBUF(fd);
    if (status != 0) {
        return status;
    }

    return 0;
}
