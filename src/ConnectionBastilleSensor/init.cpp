/**
 * @file init.cpp
 * @author Jon Szymaniak <jon.szymaniak@nuand.com>
 * @brief Platform-specific initializations
 */

#include "ConnectionBastilleSensor.h"
#include "ErrorReporting.h"
#include "LMS7002M_parameters.h"
#include "register_access.h"

using namespace lime;

int ConnectionBastilleSensor::InitLimeLight(int fd)
{
    int status = 0;
    try {
        uint16_t regval;

        // Set the drive strength of DIQ2 to 8 mA
        regval = reg_read_ex(fd, lime::DIQ2_DS.address);
        regval |= (1 << lime::DIQ2_DS.lsb);
        reg_write_ex(fd, lime::DIQ2_DS.address, regval);

        // Set LML2 to TRQXIQ_TX (RF -> BB) mode
        regval = reg_read_ex(fd, lime::LML2_MODE.address);
        regval &= ~(1 << lime::LML2_MODE.lsb);
        reg_write_ex(fd, lime::LML2_MODE.address, regval);

        // MCLK2 <- RxTSPCLKA, MCLK1 <- TxTSPCLKA, disable RX/TX clock dividers
        regval = reg_read_ex(fd, lime::MCLK1SRC.address);
        regval &= ~((0x3 << lime::MCLK1SRC.lsb) | (0x3 << lime::MCLK2SRC.lsb));
        regval |=  (0x2 << lime::MCLK1SRC.lsb);
        regval |=  (0x3 << lime::MCLK2SRC.lsb);
        regval &= ~(1 << lime::RXDIVEN.lsb);
        regval &= ~(1 << lime::TXDIVEN.lsb);
        reg_write_ex(fd, lime::MCLK1SRC.address, regval);

        // Ensure FIFOs are enabled. 1=enable, 0=reset. (Beware of doc typo!)
        regval = reg_read_ex(fd, lime::SRST_RXFIFO.address);
        regval |= (1 << lime::SRST_RXFIFO.lsb);
        regval |= (1 << lime::SRST_TXFIFO.lsb);
        reg_write_ex(fd, lime::SRST_RXFIFO.address, regval);

    } catch (int error) {
        ReportError(error, "Failed to initialize LimeLight interface");
        status = error;
    }

    return status;
}

int ConnectionBastilleSensor::InitLDO(int fd)
{
    int status = 0;
    try {
        uint16_t regval;

        // Disable SX LDO bypass on both SXR and SXT
        select_channel_ex(fd, lime::LMS7002M::ChSXR);
        regval = reg_read_ex(fd, lime::BYPLDO_VCO.address);
        regval &= ~(1 << lime::BYPLDO_VCO.lsb);
        reg_write_ex(fd, lime::BYPLDO_VCO.address, regval);

        select_channel_ex(fd, lime::LMS7002M::ChSXT);
        regval = reg_read_ex(fd, lime::BYPLDO_VCO.address);
        regval &= ~(1 << lime::BYPLDO_VCO.lsb);
        reg_write_ex(fd, lime::BYPLDO_VCO.address, regval);

        select_channel_ex(fd, lime::LMS7002M::ChAB);

        // Enable all LDOs
        reg_write_ex(fd, lime::EN_LDO_DIG.address, 0xffff);
        reg_write_ex(fd, lime::EN_LOADIMP_LDO_TLOB.address, 0x03ff);

        // Write RDIV values recommended in "LMS7002M power supply connection"
        reg_write_ex(fd, lime::RDIV_VCOSXR.address, 0x658c);
        reg_write_ex(fd, lime::RDIV_MXRFE.address,  0x8c8c);

        // Write external LDO enable (from FPGA)
        regval = reg_read_ex(fd, GPOUT_CTRL_ADDR);
        reg_write_ex(fd, GPOUT_CTRL_ADDR, regval | GPOUT_CTRL_CORE_LDO_ENABLE);


    } catch (int error) {
        ReportError(error, "Failed to initialize LDOs");
        status = error;
    }
    return status;
}

int ConnectionBastilleSensor::InitXBUF(int fd)
{
    int status = 0;
    try {
        uint16_t regval = 0;

        // Disable biasing digital contro and bypass. Power up individual XBUFs
        regval |= (1 << lime::EN_G_XBUF.lsb);

        /* Enable the 2nd output of the TX XBUF. This is used as an alternate
         * input to the RX XBUF
         *
         * IMPORTANT NOTE: This is bit is documented incorrectly in programming
         *                 manual v2.25.0
         */
        regval |= (1 << lime::EN_OUT2_XBUF_TX.lsb);

        // Source XCLK_RX from XCLK_TX_OUT2 instead of xoscin_rx_pad */
        regval |= (1 << lime::EN_TBUFIN_XBUF_RX.lsb);

        reg_write_ex(fd, lime::EN_TBUFIN_XBUF_RX.address, regval);

    } catch (int error) {
        status = error;
    }

    return status;
}
