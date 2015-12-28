/**
    @file IConnection.h
    @author Lime Microsystems
    @brief Interface class for connection types
*/

#ifndef ICONNECTION_H
#define ICONNECTION_H

#include <string>
#include <vector>
//#include <mutex>
#include <cstring> //memset
#include <functional>

enum OperationStatus
{
    SUCCESS = 0,
    FAILED,
    UNSUPPORTED,
    DISCONNECTED,
};

using namespace std;

/*!
 * Information about a particular RFIC on an IConnection.
 * RFICInfo associates streaming channels and SPI slaves.
 */
struct RFICInfo
{

    RFICInfo(void);

    /*!
     * The SPI index number used to access the lime RFIC.
     * This index will be used in the spi access functions.
     */
    int spiIndexRFIC;

    /*!
     * The SPI index number used to access the Si5351
     * found on some development boards. -1 when not present.
     */
    int spiIndexSi5351;

    /*!
     * The channel number used in the read stream API.
     * Set to -1 when RX streaming not available.
     */
    int rxChannel;

    /*!
     * The channel number used in the write stream API.
     * Set to -1 when TX streaming not available.
     */
    int txChannel;
};

/*!
 * Information about the set of available hardware on a device.
 * This includes available ICs, streamers, and version info.
 */
struct DeviceInfo
{
    DeviceInfo(void);

    //! The displayable name for the device
    std::string deviceName;

    /*! The displayable name for the expansion card
     * Ex: if the RFIC is on a daughter-card
     */
    std::string expansionName;

    //! The firmware version as a string
    std::string firmwareVersion;

    //! The hardware version as a string
    std::string hardwareVersion;

    //! The protocol version as a string
    std::string protocolVersion;
};

/*!
 * The Stream metadata structure is used with the streaming API to exchange
 * extra data associated with the stream such as timestamps and burst info.
 */
struct StreamMetadata
{
    StreamMetadata(void);

    /*!
     * The timestamp in clock units
     * Set to -1 when the timestamp is not applicable.
     */
    long long timestamp;

    /*!
     * True to indicate the end of a stream buffer.
     * When false, subsequent calls continue the stream.
     */
    bool endOfBurst;
};

/*!
 * IConnection is the interface class for a device with 1 or more Lime RFICs.
 * The LMS7002M driver class calls into IConnection to interface with the hardware
 * to implement high level functions on top of low-level SPI and GPIO.
 * Device developers will implement a custom IConnection for their hardware
 * as an abstraction for streaming and low-level SPI and configuration access.
 */
class IConnection
{
public:

    //! IConnection constructor
    IConnection(void);

    //! IConnection destructor
    virtual ~IConnection(void);

    /*!
     * Is this connection open?
     * The constructor should attempt to connect but may fail,
     * or the connection may go down at a later time.
     * @return true when the connection is available
     */
    virtual bool IsOpen(void);

    /*!
     * Get information about a device
     * for displaying helpful information
     * or for making device-specific decisions.
     */
    virtual DeviceInfo GetDeviceInfo(void);

    /*!
     * RFIC enumeration API.
     * @return a list of RFICInfos
     */
    virtual std::vector<RFICInfo> ListRFICs(void);

    /*!
     * Perform reset sequence on the device.
     * Typically this will reset the RFIC using a GPIO,
     * and possibly other ICs located on the device.
     */
    virtual OperationStatus DeviceReset(void);

   /*!
    * @brief Bulk SPI write/read transaction.
    *
    * The transactSPI function is capable of bulk writes and bulk reads
    * of SPI registers in an arbitrary IC (up to 32-bits per transaction).
    *
    * The readData parameter may be NULL to indicate a write-only operation,
    * the underlying implementation may be able to optimize out the readback.
    *
    * @param index the SPI device index
    * @param writeData SPI bits to write out
    * @param [out] readData stores readback data
    * @param size the number of SPI transactions
    * @return the transaction success state
    */
    virtual OperationStatus TransactSPI(const int index, const uint32_t *writeData, uint32_t *readData, const size_t size);

    /*!
     * Called by the LMS7002M driver after potential band-selection changes.
     * Implementations may have additional external bands to switch via GPIO.
     * @param trfBand the SEL_BAND2_TRF config bits
     * @param rfeBand the SEL_PATH_RFE config bits
     */
    virtual void UpdateExternalBandSelect(const int trfBand, const int rfeBand);

    /*!
     * Query the frequency of the reference clock.
     * Some implementations have a fixed reference,
     * some have a programmable synthesizer like Si5351C.
     * @return the reference clock rate in Hz
     */
    virtual double GetReferenceClockRate(void);

    /*!
     * Set the programmable reference clock rate.
     * Some implementations use the programmable Si5351C.
     * @param rate the clock rate in Hz
     */
    virtual void SetReferenceClockRate(const double rate);

    /*!
     * The RX stream control call configures a channel to
     * stream at a particular time, requests burst,
     * or to start or stop continuous streaming.
     *
     * - Use the metadata's optional timestamp to control stream time
     * - Use the metadata's end of burst to request stream bursts
     * - Without end of burst, the burstSize affects continuous streaming
     *
     * @param streamID the RX stream index number
     * @param burstSize the burst size when metadata has end of burst
     * @param metadata time and burst options
     * @return true for success, otherwise false
     */
    virtual bool RxStreamControl(const int streamID, const size_t burstSize, const StreamMetadata &metadata);

    /*!
     * Read blocking data from the stream into the specified buffer.
     *
     * @param streamID the RX stream index number
     * @param buffs an array of buffers pointers
     * @param length the number of bytes in the buffer
     * @param timeout_ms the timeout in milliseconds
     * @param [out] metadata optional stream metadata
     * @return the number of bytes read or error code
     */
    virtual int ReadStream(const int streamID, void * const *buffs, const size_t length, const long timeout_ms, StreamMetadata &metadata);

    /*!
     * Write blocking data into the stream from the specified buffer.
     *
     * - The metadata timestamp corresponds to the start of the buffer.
     * - The end of burst only applies when all bytes have been written.
     *
     * @param streamID the TX stream stream number
     * @param buffs an array of buffers pointers
     * @param length the number of bytes in the buffer
     * @param timeout_ms the timeout in milliseconds
     * @param metadata optional stream metadata
     * @return the number of bytes written or error code
     */
    virtual int WriteStream(const int streamID, const void * const *buffs, const size_t length, const long timeout_ms, const StreamMetadata &metadata);



    /** @brief Uploads program to selected device
        @param buffer binary program data
        @param length buffer length
        @param programmingMode to RAM, to FLASH, to EEPROM, etc..
        @param index target device number
        @return the operation success state

        Can be used to program MCU, FPGA, write external on board memory.
        This could be a quite long operation, need callback to get progress or terminate early
    */
    virtual OperationStatus ProgramWrite(const char *buffer, const size_t length, const int programmingMode, const int index);

    /**	@brief Reads current program from selected device
        @param destination buffer for binary program data
        @param length buffer length to read
        @param index target device number
        @return the operation success state
    */
    virtual OperationStatus ProgramRead(char *buffer, const size_t length, const int index);

    /**	@brief Writes GPIO values to device
    @param source buffer for GPIO values LSB first, each bit sets GPIO state
    @param bufLength buffer length
    @return the operation success state
    */
    virtual OperationStatus GPIOWrite(const uint8_t *buffer, const size_t bufLength);

    /**	@brief Reads GPIO values from device
    @param destination buffer for GPIO values LSB first, each bit represent GPIO state
    @param bufLength buffer length to read
    @return the operation success state
    */
    virtual OperationStatus GPIORead(uint8_t *buffer, const size_t bufLength);

    /***********************************************************************
     * !!! Below is the old IConnection Streaming API
     * It remains here to enable compiling until its replaced
     **********************************************************************/

    int ReadStream(char *buffer, int length, unsigned int timeout_ms)
    {
        /*int handle = activeControlPort->BeginDataReading(buffer, length);
        activeControlPort->WaitForReading(handle, timeout_ms);
            long received = length;
            activeControlPort->FinishDataReading(buffer, received, handle);
            return received;
        */
        long len = length;
        int status = this->ReadDataBlocking(buffer, len, 0);
        return len;
    }

	virtual int BeginDataReading(char *buffer, long length){ return -1; };
	virtual int WaitForReading(int contextHandle, unsigned int timeout_ms){ return 0;};
	virtual int FinishDataReading(char *buffer, long &length, int contextHandle){ return 0;}
	virtual void AbortReading(){};
    virtual int ReadDataBlocking(char *buffer, long &length, int timeout_ms){ return 0; }

	virtual int BeginDataSending(const char *buffer, long length){ return -1; };
	virtual int WaitForSending(int contextHandle, unsigned int timeout_ms){ return 0;};
	virtual int FinishDataSending(const char *buffer, long &length, int contextHandle){ return 0;}
	virtual void AbortSending(){};

    /** @brief Sets callback function which gets called each time data is sent or received
    */
    void SetDataLogCallback(std::function<void(bool, const unsigned char*, const unsigned int)> callback);

protected:
    //unsigned char* PreparePacket(const GenericPacket &pkt, int &length, const eLMS_PROTOCOL protocol);
    //int ParsePacket(GenericPacket &pkt, const unsigned char* buffer, const int length, const eLMS_PROTOCOL protocol);
    //eConnectionType m_connectionType;
    std::function<void(bool, const unsigned char*, const unsigned int)> callback_logData;
    bool mSystemBigEndian;
    //std::mutex mControlPortLock;
};

#endif
