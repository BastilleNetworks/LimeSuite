/**
 * @file ConnectionBastilleSensorEntry.cpp
 * @author Jon Szymaniak <jon.szymaniak@nuand.com>
 * @brief Create ConnectionBastilleSensor IConnections
 */

#include <iostream>
#include <sys/stat.h>

#include "ConnectionBastilleSensor.h"

using namespace lime;

// The other ConnectionRegistry entries all appear to statically register,
// with plans to perform this dynamically with shared libraries.  We'll follow
// suit until the preferred dynamic loading procedures have been established.
void __loadConnectionBastilleSensorEntry(void)
{
    static ConnectionBastilleSensorEntry BastilleSensorEntry;
}

ConnectionBastilleSensorEntry::ConnectionBastilleSensorEntry(void) :
    ConnectionRegistryEntry("Bastille Sensor")
{
    return;
}

// Enumerate the attached LMS7002M entries on the system by simply checking
// for the presence of the character driver nodes.
//
// TODO: Use the hint to limit the search space or change basedir
std::vector<ConnectionHandle> ConnectionBastilleSensorEntry::enumerate(const ConnectionHandle &hint)
{
    std::vector<ConnectionHandle> handles;
    const std::string basedir("/dev/lms7-");
    std::string addrs;

    for (int i = 0; i < 256; i++) {
        int status;
        struct stat stat_buf;
        const std::string device(basedir + std::to_string(i));

        status = stat(device.c_str(), &stat_buf);
        if (status == 0 && S_ISCHR(stat_buf.st_mode)) {
            addrs += basedir + std::to_string(i) + ":";
        }
    }

    if (not addrs.empty()) {
        ConnectionHandle h;
        h.media = "AXI";            // Memory-mapped access via AXI Bus
        h.name = "BASTILLESENSOR";
        h.addr = addrs;
        handles.push_back(h);
    }

    return handles;
}

std::vector<std::string> devstr_to_vector(const std::string devstr)
{
    std::vector<std::string> devs;
    size_t pos = 0, npos;

    do {
        npos = devstr.find(':', pos);

        if (npos != std::string::npos) {
            std::string dev(devstr.substr(pos, npos - pos));
            if (not dev.empty()) {
                devs.push_back(dev);
            }
            pos = npos + 1;
        }
    } while (npos != std::string::npos);

    // Cover last entry without trailing colon
    if (pos < devstr.size()) {
        std::string dev(devstr.substr(pos, devstr.size() - pos));
        if (not dev.empty()) {
            devs.push_back(dev);
        }
    }

    return devs;
}

IConnection *ConnectionBastilleSensorEntry::make(const ConnectionHandle &handle)
{
    return new ConnectionBastilleSensor(devstr_to_vector(handle.addr));
}
