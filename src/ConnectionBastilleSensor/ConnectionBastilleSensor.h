/**
 * @file ConnectionBastilleSensor.h
 * @author Jon Szymaniak <jon.szymaniak@nuand.com>
 * @brief LMS7002M connections on Bastille Sensor via nlms7 kernel module
 */

#ifndef CONNECTION_BASTILLE_SENSOR_H_
#define CONNECTION_BASTILLE_SENSOR_H_

#include <cstdint>
#include <vector>
#include <string>
#include <map>

#include <ConnectionRegistry.h>
#include <IConnection.h>
#include <ErrorReporting.h>

namespace lime {

class ConnectionBastilleSensor : public virtual IConnection
{
public:
    ConnectionBastilleSensor(const std::vector<std::string> &devices);
    ~ConnectionBastilleSensor();

    int TransactSPI(const int addr, const uint32_t *writeData, uint32_t *readData, const size_t size);

    DeviceInfo GetDeviceInfo();

    // Reset and re-initialize all LMS7002M devices handled by this connection
    // This should be called before attempting to use the connection.
    int DeviceReset();

    // Reset and reinitialize the specified device
    int DeviceReset(int addr);

    // Our connection is considered "open" if we have any open FDs
    bool IsOpen() { return !_reg_fd.empty(); }

    // This platform has a fixed 40 MHz reference
    double GetReferenceClockRate() { return 40e6; }
    double GetTxReferenceClockRate() { return GetReferenceClockRate(); }

private:

    /*
     * Per-device Initializations
     */

    // Initialize Lime interface LML2 drive strength and mode (TRXIQ_TX)
    int InitLimeLight(int fd);

    // Configure the use of internal LDOs
    int InitLDO(int fd);

    // Configure devices to use a single XBUF for SXR and SXT
    int InitXBUF(int fd);

    /*
     * Register Access for use in the above inits
     */

    // Perform the register writes described by the array of transfers
    int RegWrite(int fd, const uint32_t *xfer, size_t count);

    // Perform the register reads decibed by input transfers
    int RegRead(int fd, const uint32_t *xferIn, uint32_t *xferOut, size_t count);

    // IConnection "address" -> file handle for register access
    std::map<int, int> _reg_fd;
};

class ConnectionBastilleSensorEntry : public ConnectionRegistryEntry
{
public:
    ConnectionBastilleSensorEntry(void);

    std::vector<ConnectionHandle> enumerate(const ConnectionHandle &hint);

    IConnection *make(const ConnectionHandle &handle);
};

#endif // CONNECTION_BASTILLE_SENSOR_H_

}
