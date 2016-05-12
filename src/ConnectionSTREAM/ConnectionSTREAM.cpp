/**
    @file ConnectionSTREAM.cpp
    @author Lime Microsystems
    @brief Implementation of STREAM board connection.
*/

#include "ConnectionSTREAM.h"
#include "ErrorReporting.h"
#include <cstring>
#include <iostream>
#include "Si5351C.h"
#include "fifo.h"
#include "FPGA_common.h"

#include <thread>
#include <chrono>

using namespace std;

#define USB_TIMEOUT 1000

#define HW_LDIGIRED L"DigiRed"
#define HW_LDIGIGREEN L"DigiGreen"
#define HW_LSTREAMER L"Stream"

#define HW_DIGIRED "DigiRed"
#define HW_DIGIGREEN "DigiGreen"
#define HW_STREAMER "Stream"

#define CTR_W_REQCODE 0xC1
#define CTR_W_VALUE 0x0000
#define CTR_W_INDEX 0x0000

#define CTR_R_REQCODE 0xC0
#define CTR_R_VALUE 0x0000
#define CTR_R_INDEX 0x0000

using namespace lime;

/**	@brief Initializes port type and object necessary to communicate to usb device.
*/
ConnectionSTREAM::ConnectionSTREAM(void *arg, const unsigned index, const int vid, const int pid) :
    mRxService(this), mTxService(this)
{
    m_hardwareName = "";
    isConnected = false;
#ifndef __unix__
    USBDevicePrimary = (CCyUSBDevice *)arg;
    OutCtrEndPt = NULL;
    InCtrEndPt = NULL;
	InCtrlEndPt3 = NULL;
	OutCtrlEndPt3 = NULL;
#else
    dev_handle = 0;
    devs = 0;
    ctx = (libusb_context *)arg;
#endif
    if (this->Open(index, vid, pid) != 0)
        std::cerr << GetLastErrorMessage() << std::endl;

    //must configure synthesizer before using LimeSDR
    DeviceInfo info = this->GetDeviceInfo();
    if (info.deviceName == GetDeviceName(LMS_DEV_LIMESDR))
    {
        std::shared_ptr<Si5351C> si5351module(new Si5351C());
        si5351module->Initialize(this);
        si5351module->SetPLL(0, 25000000, 0);
        si5351module->SetPLL(1, 25000000, 0);
        si5351module->SetClock(0, 27000000, true, false);
        si5351module->SetClock(1, 27000000, true, false);
        for (int i = 2; i < 8; ++i)
            si5351module->SetClock(i, 27000000, false, false);
        Si5351C::Status status = si5351module->ConfigureClocks();
        if (status != Si5351C::SUCCESS)
        {
            std::cerr << "Warning: Failed to configure Si5351C" << std::endl;
            return;
        }
        status = si5351module->UploadConfiguration();
        if (status != Si5351C::SUCCESS)
            std::cerr << "Warning: Failed to upload Si5351C configuration" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); //some settle time
    }
}

/**	@brief Closes connection to chip and deallocates used memory.
*/
ConnectionSTREAM::~ConnectionSTREAM()
{
    mTxService.destroy();
    mRxService.destroy();
    Close();
}

/**	@brief Tries to open connected USB device and find communication endpoints.
	@return Returns 0-Success, other-EndPoints not found or device didn't connect.
*/
int ConnectionSTREAM::Open(const unsigned index, const int vid, const int pid)
{
#ifndef __unix__
	wstring m_hardwareDesc = L"";
	if( index < USBDevicePrimary->DeviceCount())
	{
		if(USBDevicePrimary->Open(index))
		{
            m_hardwareDesc = USBDevicePrimary->Product;
            unsigned int pos;
            //determine connected board type
            pos = m_hardwareDesc.find(HW_LDIGIRED, 0);
            if( pos != wstring::npos )
                m_hardwareName = HW_DIGIRED;
            else if (m_hardwareDesc.find(HW_LSTREAMER, 0) != wstring::npos)
                m_hardwareName = HW_STREAMER;
            else
				m_hardwareName = HW_STREAMER;


			if (InCtrlEndPt3)
			{
				delete InCtrlEndPt3;
				InCtrlEndPt3 = NULL;
			}
			InCtrlEndPt3 = new CCyControlEndPoint(*USBDevicePrimary->ControlEndPt);

			if (OutCtrlEndPt3)
			{
				delete OutCtrlEndPt3;
				OutCtrlEndPt3 = NULL;
			}
			OutCtrlEndPt3 = new CCyControlEndPoint(*USBDevicePrimary->ControlEndPt);

			InCtrlEndPt3->ReqCode = CTR_R_REQCODE;
			InCtrlEndPt3->Value = CTR_R_VALUE;
			InCtrlEndPt3->Index = CTR_R_INDEX;

			OutCtrlEndPt3->ReqCode = CTR_W_REQCODE;
			OutCtrlEndPt3->Value = CTR_W_VALUE;
			OutCtrlEndPt3->Index = CTR_W_INDEX;

			for (int i=0; i<USBDevicePrimary->EndPointCount(); i++)
				if(USBDevicePrimary->EndPoints[i]->Address == 0x01)
				{
					OutEndPt = USBDevicePrimary->EndPoints[i];
					long len = OutEndPt->MaxPktSize * 64;
					OutEndPt->SetXferSize(len);
					break;
				}
			for (int i=0; i<USBDevicePrimary->EndPointCount(); i++)
				if(USBDevicePrimary->EndPoints[i]->Address == 0x81)
				{
					InEndPt = USBDevicePrimary->EndPoints[i];
					long len = InEndPt->MaxPktSize * 64;
					InEndPt->SetXferSize(len);
					break;
				}
			isConnected = true;
			return 0;
		} //successfully opened device
	} //if has devices
    return ReportError("No matching devices found");
#else

    if( vid == 1204)
    {
        if(pid == 34323)
        {
            m_hardwareName = HW_DIGIGREEN;
        }
        else if(pid == 241)
        {
            m_hardwareName = HW_DIGIRED;
        }
    }

    dev_handle = libusb_open_device_with_vid_pid(ctx, vid, pid);

    if(dev_handle == nullptr)
        return ReportError("libusb_open failed");
    if(libusb_kernel_driver_active(dev_handle, 0) == 1)   //find out if kernel driver is attached
    {
        printf("Kernel Driver Active\n");
        if(libusb_detach_kernel_driver(dev_handle, 0) == 0) //detach it
            printf("Kernel Driver Detached!\n");
    }
    int r = libusb_claim_interface(dev_handle, 0); //claim interface 0 (the first) of device
    if(r < 0)
    {
        printf("Cannot Claim Interface\n");
        return ReportError("Cannot claim interface - %s", libusb_strerror(libusb_error(r)));
    }
    isConnected = true;
    return 0;
#endif
}

/**	@brief Closes communication to device.
*/
void ConnectionSTREAM::Close()
{
    #ifndef __unix__
	USBDevicePrimary->Close();
	InEndPt = NULL;
	OutEndPt = NULL;
	if (InCtrlEndPt3)
	{
		delete InCtrlEndPt3;
		InCtrlEndPt3 = NULL;
	}
	if (OutCtrlEndPt3)
	{
		delete OutCtrlEndPt3;
		OutCtrlEndPt3 = NULL;
	}
    #else
    if(dev_handle != 0)
    {
        libusb_release_interface(dev_handle, 0);
        libusb_close(dev_handle);
        dev_handle = 0;
    }
    #endif
    isConnected = false;
}

/**	@brief Returns connection status
	@return 1-connection open, 0-connection closed.
*/
bool ConnectionSTREAM::IsOpen()
{
    #ifndef __unix__
    return USBDevicePrimary->IsOpen() && isConnected;
    #else
    return isConnected;
    #endif
}

/**	@brief Sends given data buffer to chip through USB port.
	@param buffer data buffer, must not be longer than 64 bytes.
	@param length given buffer size.
    @param timeout_ms timeout limit for operation in milliseconds
	@return number of bytes sent.
*/
int ConnectionSTREAM::Write(const unsigned char *buffer, const int length, int timeout_ms)
{
    std::lock_guard<std::mutex> lock(mExtraUsbMutex);
    long len = length;
    if(IsOpen())
    {
		unsigned char* wbuffer = new unsigned char[length];
		memcpy(wbuffer, buffer, length);
        if(m_hardwareName == HW_DIGIRED || m_hardwareName == HW_STREAMER)
        {
            #ifndef __unix__
            if(OutCtrlEndPt3)
                OutCtrlEndPt3->Write(wbuffer, len);
            else
                len = 0;
            #else
                len = libusb_control_transfer(dev_handle, LIBUSB_REQUEST_TYPE_VENDOR,CTR_W_REQCODE ,CTR_W_VALUE, CTR_W_INDEX, wbuffer, length, USB_TIMEOUT);
            #endif
        }
        else
        {
            #ifndef __unix__
            if(OutCtrEndPt)
                OutCtrEndPt->XferData(wbuffer, len);
            else
                len = 0;
            #else
                int actual = 0;
                libusb_bulk_transfer(dev_handle, 0x01, wbuffer, len, &actual, USB_TIMEOUT);
                len = actual;
            #endif
        }
		delete wbuffer;
    }
    else
        return 0;
    return len;
}

/**	@brief Reads data coming from the chip through USB port.
	@param buffer pointer to array where received data will be copied, array must be
	big enough to fit received data.
	@param length number of bytes to read from chip.
    @param timeout_ms timeout limit for operation in milliseconds
	@return number of bytes received.
*/
int ConnectionSTREAM::Read(unsigned char *buffer, const int length, int timeout_ms)
{
    std::lock_guard<std::mutex> lock(mExtraUsbMutex);
    long len = length;
    if(IsOpen())
    {
        if(m_hardwareName == HW_DIGIRED || m_hardwareName == HW_STREAMER)
        {
            #ifndef __unix__
            if(InCtrlEndPt3)
                InCtrlEndPt3->Read(buffer, len);
            else
                len = 0;
            #else
            len = libusb_control_transfer(dev_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN ,CTR_R_REQCODE ,CTR_R_VALUE, CTR_R_INDEX, buffer, len, USB_TIMEOUT);
            #endif
        }
        else
        {
            #ifndef __unix__
            if(InCtrEndPt)
                InCtrEndPt->XferData(buffer, len);
            else
                len = 0;
            #else
                int actual = 0;
                libusb_bulk_transfer(dev_handle, 0x81, buffer, len, &actual, USB_TIMEOUT);
                len = actual;
            #endif
        }
    }
    return len;
}

#ifdef __unix__
/**	@brief Function for handling libusb callbacks
*/
void callback_libusbtransfer(libusb_transfer *trans)
{
	USBTransferContext *context = reinterpret_cast<USBTransferContext*>(trans->user_data);
	switch(trans->status)
	{
    case LIBUSB_TRANSFER_CANCELLED:
        //printf("Transfer %i canceled\n", context->id);
        context->bytesXfered = trans->actual_length;
        context->done.store(true);
        //context->used = false;
        //context->reset();
        break;
    case LIBUSB_TRANSFER_COMPLETED:
        //if(trans->actual_length == context->bytesExpected)
		{
			context->bytesXfered = trans->actual_length;
			context->done.store(true);
		}
        break;
    case LIBUSB_TRANSFER_ERROR:
        printf("TRANSFER ERRRO\n");
        context->bytesXfered = trans->actual_length;
        context->done.store(true);
        //context->used = false;
        break;
    case LIBUSB_TRANSFER_TIMED_OUT:
        //printf("transfer timed out %i\n", context->id);
        context->bytesXfered = trans->actual_length;
        context->done.store(true);
        //context->used = false;

        break;
    case LIBUSB_TRANSFER_OVERFLOW:
        printf("transfer overflow\n");

        break;
    case LIBUSB_TRANSFER_STALL:
        printf("transfer stalled\n");
        break;
    case LIBUSB_TRANSFER_NO_DEVICE:
        printf("transfer no device\n");

        break;
	}
	context->transferLock.unlock();
	context->cv.notify_one();
}
#endif

/**
	@brief Starts asynchronous data reading from board
	@param *buffer buffer where to store received data
	@param length number of bytes to read
	@return handle of transfer context
*/
int ConnectionSTREAM::BeginDataReading(char *buffer, long length)
{
    int i = 0;
	bool contextFound = false;
	//find not used context
    for(i = 0; i<USB_MAX_CONTEXTS; i++)
    {
        if(!contexts[i].used)
        {
            contextFound = true;
            break;
        }
    }
    if(!contextFound)
    {
        printf("No contexts left for reading data\n");
        return -1;
    }
    contexts[i].used = true;
    #ifndef __unix__
    if(InEndPt)
        contexts[i].context = InEndPt->BeginDataXfer((unsigned char*)buffer, length, contexts[i].inOvLap);
	return i;
    #else
    unsigned int Timeout = 500;
    libusb_transfer *tr = contexts[i].transfer;
	libusb_fill_bulk_transfer(tr, dev_handle, 0x81, (unsigned char*)buffer, length, callback_libusbtransfer, &contexts[i], Timeout);
	contexts[i].done = false;
	contexts[i].bytesXfered = 0;
	contexts[i].bytesExpected = length;
	int status = libusb_submit_transfer(tr);
    if(status != 0)
    {
        printf("ERROR BEGIN DATA READING %s\n", libusb_error_name(status));
        contexts[i].used = false;
        return -1;
    }
    else
        contexts[i].transferLock.lock();
    #endif
    return i;
}

/**
	@brief Waits for asynchronous data reception
	@param contextHandle handle of which context data to wait
	@param timeout_ms number of miliseconds to wait
	@return 1-data received, 0-data not received
*/
int ConnectionSTREAM::WaitForReading(int contextHandle, unsigned int timeout_ms)
{
    if(contextHandle >= 0 && contexts[contextHandle].used == true)
    {
    int status = 0;
    #ifndef __unix__
	if(InEndPt)
        status = InEndPt->WaitForXfer(contexts[contextHandle].inOvLap, timeout_ms);
	return status;
    #else
    auto t1 = chrono::high_resolution_clock::now();
    auto t2 = chrono::high_resolution_clock::now();

    std::unique_lock<std::mutex> lck(contexts[contextHandle].transferLock);
    while(contexts[contextHandle].done.load() == false && std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() < timeout_ms)
    {
        //blocking not to waste CPU
        contexts[contextHandle].cv.wait(lck);
        t2 = chrono::high_resolution_clock::now();
    }
	return contexts[contextHandle].done.load() == true;
    #endif
    }
    else
        return 0;
}

/**
	@brief Finishes asynchronous data reading from board
	@param buffer array where to store received data
	@param length number of bytes to read, function changes this value to number of bytes actually received
	@param contextHandle handle of which context to finish
	@return false failure, true number of bytes received
*/
int ConnectionSTREAM::FinishDataReading(char *buffer, long &length, int contextHandle)
{
    if(contextHandle >= 0 && contexts[contextHandle].used == true)
    {
    #ifndef __unix__
    int status = 0;
    if(InEndPt)
        status = InEndPt->FinishDataXfer((unsigned char*)buffer, length, contexts[contextHandle].inOvLap, contexts[contextHandle].context);
    contexts[contextHandle].used = false;
    contexts[contextHandle].reset();
    return length;
    #else
	length = contexts[contextHandle].bytesXfered;
	contexts[contextHandle].used = false;
	contexts[contextHandle].reset();
	return length;
    #endif
    }
    else
        return 0;
}

/**
	@brief Aborts reading operations
*/
void ConnectionSTREAM::AbortReading()
{
#ifndef __unix__
	InEndPt->Abort();
#else
    for(int i=0; i<USB_MAX_CONTEXTS; ++i)
    {
        if(contexts[i].used)
            libusb_cancel_transfer( contexts[i].transfer );
    }
#endif
}

/**
	@brief Starts asynchronous data Sending to board
	@param *buffer buffer to send
	@param length number of bytes to send
	@return handle of transfer context
*/
int ConnectionSTREAM::BeginDataSending(const char *buffer, long length)
{
    int i = 0;
	//find not used context
	bool contextFound = false;
    for(i = 0; i<USB_MAX_CONTEXTS; i++)
    {
        if(!contextsToSend[i].used)
        {
            contextFound = true;
            break;
        }
    }
    if(!contextFound)
        return -1;
    contextsToSend[i].used = true;
    #ifndef __unix__
    if(OutEndPt)
        contextsToSend[i].context = OutEndPt->BeginDataXfer((unsigned char*)buffer, length, contextsToSend[i].inOvLap);
	return i;
    #else
    unsigned int Timeout = 500;
    libusb_transfer *tr = contextsToSend[i].transfer;
	libusb_fill_bulk_transfer(tr, dev_handle, 0x1, (unsigned char*)buffer, length, callback_libusbtransfer, &contextsToSend[i], Timeout);
	contextsToSend[i].done = false;
	contextsToSend[i].bytesXfered = 0;
	contextsToSend[i].bytesExpected = length;
    int status = libusb_submit_transfer(tr);
    if(status != 0)
    {
        printf("ERROR BEGIN DATA SENDING %s\n", libusb_error_name(status));
        contextsToSend[i].used = false;
        return -1;
    }
    else
        contextsToSend[i].transferLock.lock();
    #endif
    return i;
}

/**
	@brief Waits for asynchronous data sending
	@param contextHandle handle of which context data to wait
	@param timeout_ms number of miliseconds to wait
	@return 1-data received, 0-data not received
*/
int ConnectionSTREAM::WaitForSending(int contextHandle, unsigned int timeout_ms)
{
    if( contextsToSend[contextHandle].used == true )
    {
    #ifndef __unix__
	int status = 0;
	if(OutEndPt)
        status = OutEndPt->WaitForXfer(contextsToSend[contextHandle].inOvLap, timeout_ms);
	return status;
    #else
    auto t1 = chrono::high_resolution_clock::now();
    auto t2 = chrono::high_resolution_clock::now();
    std::unique_lock<std::mutex> lck(contextsToSend[contextHandle].transferLock);
    while(contextsToSend[contextHandle].done.load() == false && std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() < timeout_ms)
    {
        //blocking not to waste CPU
        contextsToSend[contextHandle].cv.wait(lck);
        t2 = chrono::high_resolution_clock::now();
    }
	return contextsToSend[contextHandle].done == true;
    #endif
    }
    else
        return 0;
}

/**
	@brief Finishes asynchronous data sending to board
	@param buffer array where to store received data
	@param length number of bytes to read, function changes this value to number of bytes acctually received
	@param contextHandle handle of which context to finish
	@return false failure, true number of bytes sent
*/
int ConnectionSTREAM::FinishDataSending(const char *buffer, long &length, int contextHandle)
{
    if( contextsToSend[contextHandle].used == true)
    {
    #ifndef __unix__
	if(OutEndPt)
        OutEndPt->FinishDataXfer((unsigned char*)buffer, length, contextsToSend[contextHandle].inOvLap, contextsToSend[contextHandle].context);
    contextsToSend[contextHandle].used = false;
    contextsToSend[contextHandle].reset();
    return length;
    #else
	length = contextsToSend[contextHandle].bytesXfered;
	contextsToSend[contextHandle].used = false;
    contextsToSend[contextHandle].reset();
	return length;
    #endif
    }
    else
        return 0;
}

/**
	@brief Aborts sending operations
*/
void ConnectionSTREAM::AbortSending()
{
#ifndef __unix__
	OutEndPt->Abort();
#else
    for (int i = 0; i<USB_MAX_CONTEXTS; ++i)
    {
        if(contextsToSend[i].used)
            libusb_cancel_transfer(contextsToSend[i].transfer);
    }
#endif
}

static void ResetUSBFIFO(LMS64CProtocol* port)
{
// TODO : USB FIFO reset command for IConnection
    if (port == nullptr) return;
    LMS64CProtocol::GenericPacket ctrPkt;
    ctrPkt.cmd = CMD_USB_FIFO_RST;
    ctrPkt.outBuffer.push_back(0x01);
    port->TransferPacket(ctrPkt);
    ctrPkt.outBuffer[0] = 0x00;
    port->TransferPacket(ctrPkt);
}

ConnectionSTREAM::StreamService::StreamService(ConnectionSTREAM* serPort)
{
    active = false;
    port = serPort;
    FIFO = nullptr;
}
ConnectionSTREAM::StreamService::~StreamService()
{
    destroy();
}

int ConnectionSTREAM::StreamService::setup(const StreamConfig &config)
{
#ifndef NDEBUG
    printf("%s Service setup\n", (config.isTx ? "Tx" : "Rx"));
    printf("channels count %i : [", (int)config.channels.size());
    for(int i=0; i<config.channels.size(); ++i)
        printf("%i ", (int)config.channels[i]);
    printf("]\n");
    string outputFormat;
    if(config.format == StreamConfig::StreamDataFormat::STREAM_12_BIT_IN_16)
        outputFormat = "STREAM_12_BIT_IN_16";
    else if(config.format == StreamConfig::StreamDataFormat::STREAM_COMPLEX_FLOAT32)
        outputFormat = "STREAM_COMPLEX_FLOAT32";
    else if(config.format == StreamConfig::StreamDataFormat::STREAM_12_BIT_COMPRESSED)
        outputFormat = "STREAM_12_BIT_COMPRESSED";
    else
        outputFormat = "undefined";
    printf("Output format: %s\n", outputFormat.c_str());
    string linkFormat;
    if(config.linkFormat == StreamConfig::StreamDataFormat::STREAM_12_BIT_IN_16)
        linkFormat = "STREAM_12_BIT_IN_16";
    else if(config.linkFormat == StreamConfig::StreamDataFormat::STREAM_COMPLEX_FLOAT32)
        linkFormat = "STREAM_COMPLEX_FLOAT32";
    else if(config.linkFormat == StreamConfig::StreamDataFormat::STREAM_12_BIT_COMPRESSED)
        linkFormat = "STREAM_12_BIT_COMPRESSED";
    else
        linkFormat = "undefined";
    printf("Link format: %s\n", linkFormat.c_str());
#endif
    mActiveStreamConfig = config;
    if(FIFO)
        delete FIFO;
    FIFO = new LMS_SamplesFIFO(65536, 2);
    mActiveStreamConfig = config;
    fpga::InitializeStreaming(port, config);
    //USB FIFO reset
    ResetUSBFIFO(port);
}

int ConnectionSTREAM::StreamService::start()
{
    //create threads
    mTerminate.store(0);
    ThreadData threadArgs;
    threadArgs.dataPort = port;
    threadArgs.terminate = &mTerminate;
    threadArgs.config.channels = mActiveStreamConfig.channels;
    threadArgs.FIFO = FIFO;
    active = true;
    fpga::StartStreaming(port);
    mThread = std::thread(ReceivePackets, threadArgs);
}

int ConnectionSTREAM::StreamService::stop()
{
    if(!active)
        return 0;
    mTerminate.store(1);
    mThread.join();
    active = false;
    fpga::StopStreaming(port);
}

int ConnectionSTREAM::StreamService::destroy()
{
    if(isRunning())
        stop();
    if(FIFO)
        delete FIFO;
    FIFO = nullptr;
}

/** @brief Function dedicated for receiving data samples from board
    @param rxFIFO FIFO to store received data
    @param terminate periodically pooled flag to terminate thread
    @param dataRate_Bps (optional) if not NULL periodically returns data rate in bytes per second
*/
void ConnectionSTREAM::StreamService::ReceivePackets(const ThreadData &args)
{
    auto dataPort = args.dataPort;
    auto rxFIFO = args.FIFO;
    auto terminate = args.terminate;
    auto dataRate_Bps = args.dataRate_Bps;
    //auto report = args.report;
    //auto getCmd = args.getCmd;

    //at this point Rx must be enabled in FPGA
    unsigned long rxDroppedSamples = 0;
    const int channelsCount = args.config.channels.size();
    uint32_t samplesCollected = 0;
    auto t1 = chrono::high_resolution_clock::now();
    auto t2 = chrono::high_resolution_clock::now();

    const int bufferSize = 65536;
    const int buffersCount = 16; // must be power of 2
    const int buffersCountMask = buffersCount - 1;
    int handles[buffersCount];
    memset(handles, 0, sizeof(int)*buffersCount);
    vector<char>buffers;
    try
    {
        buffers.resize(buffersCount*bufferSize, 0);
    }
    catch (const std::bad_alloc &ex)
    {
        printf("Error allocating Rx buffers, not enough memory\n");
        return;
    }

    //temporary buffer to store samples for batch insertion to FIFO
    PacketFrame tempPacket;
    tempPacket.Initialize(channelsCount);

    for (int i = 0; i<buffersCount; ++i)
        handles[i] = dataPort->BeginDataReading(&buffers[i*bufferSize], bufferSize);

    int bi = 0;
    unsigned long totalBytesReceived = 0; //for data rate calculation
    int m_bufferFailures = 0;
    int16_t sample;
    uint32_t samplesReceived = 0;

    bool currentRxCmdValid = false;
    size_t ignoreTxLateCount = 0;

    while (terminate->load() == false)
    {
        if (ignoreTxLateCount != 0) ignoreTxLateCount--;
        if (dataPort->WaitForReading(handles[bi], 1000) == false)
            ++m_bufferFailures;

        long bytesToRead = bufferSize;
        long bytesReceived = dataPort->FinishDataReading(&buffers[bi*bufferSize], bytesToRead, handles[bi]);
        if (bytesReceived > 0)
        {
            if (bytesReceived != bufferSize) //data should come in full sized packets
                ++m_bufferFailures;

            totalBytesReceived += bytesReceived;
            for (int pktIndex = 0; pktIndex < bytesReceived / sizeof(PacketLTE); ++pktIndex)
            {
                PacketLTE* pkt = (PacketLTE*)&buffers[bi*bufferSize];
                tempPacket.first = 0;
                tempPacket.timestamp = pkt[pktIndex].counter;

                int statusFlags = 0;
                uint8_t* pktStart = (uint8_t*)pkt[pktIndex].data;
                const int stepSize = channelsCount * 3;
                size_t numPktBytes = sizeof(pkt->data);

                auto byte0 = pkt[pktIndex].reserved[0];
                if ((byte0 & (1 << 3)) != 0 and ignoreTxLateCount == 0)
                {
                    uint16_t reg9;
                    dataPort->ReadRegister(0x0009, reg9);
                    dataPort->WriteRegister(0x0009, reg9 | (1 << 1));
                    dataPort->WriteRegister(0x0009, reg9 & ~(1 << 1));
//                    if (report)
//                        report(STATUS_FLAG_TX_LATE, pkt[pktIndex].counter);
                    ignoreTxLateCount = 16;
                }

//                if (report)
//                    report(STATUS_FLAG_TIME_UP, pkt[pktIndex].counter);

                /*if (getCmd)
                {
                    auto numPacketSamps = numPktBytes/stepSize;

                    //no valid command, try to pop a new command
                    if (not currentRxCmdValid) currentRxCmdValid = getCmd(currentRxCmd);

                    //command is valid, and this is an infinite read
                    //so we always replace the command if available
                    //the currentRxCmdValid variable remains true
                    else if (!currentRxCmd.finiteRead) getCmd(currentRxCmd);

                    //no valid command, just continue to the next pkt
                    if (not currentRxCmdValid) continue;

                    //handle timestamp logic
                    if (currentRxCmd.waitForTimestamp)
                    {
                        //the specified timestamp is late
                        //TODO report late condition to the rx fifo
                        //we are done with this current command
                        if (currentRxCmd.timestamp < tempPacket.timestamp)
                        {
                            std::cout << "L" << std::flush;
                            //until we can report late, treat it as asap:
                            currentRxCmd.waitForTimestamp = false;
                            //and put this one back in....
                            //currentRxCmdValid = false;
                            if (report) report(STATUS_FLAG_RX_LATE, currentRxCmd.timestamp);
                            continue;
                        }

                        //the specified timestamp is in a future packet
                        //continue onto the next packet
                        if (currentRxCmd.timestamp >= tempPacket.timestamp+numPacketSamps)
                        {
                            continue;
                        }

                        //the specified timestamp is within this packet
                        //adjust pktStart and numPktBytes for the offset
                        currentRxCmd.waitForTimestamp = false;
                        auto offsetSamps = currentRxCmd.timestamp-tempPacket.timestamp;
                        pktStart += offsetSamps*stepSize;
                        numPktBytes -= offsetSamps*stepSize;
                        numPacketSamps -= offsetSamps;
                        tempPacket.timestamp += offsetSamps;
                    }

                    //total adjustments for finite read
                    if (currentRxCmd.finiteRead)
                    {
                        //the current command has been depleted
                        //adjust numPktBytes to the requested end
                        if (currentRxCmd.numSamps <= numPacketSamps)
                        {
                            auto extraSamps = numPacketSamps-currentRxCmd.numSamps;
                            numPktBytes -= extraSamps*stepSize;
                            currentRxCmdValid = false;
                            statusFlags |= STATUS_FLAG_RX_END;
                        }
                        else currentRxCmd.numSamps -= numPacketSamps;
                    }
                }*/

                for (uint16_t b = 0; b < numPktBytes; b += stepSize)
                {
                    for (int ch = 0; ch < channelsCount; ++ch)
                    {
                        //I sample
                        sample = (pktStart[b + 1 + 3 * ch] & 0x0F) << 8;
                        sample |= (pktStart[b + 3 * ch] & 0xFF);
                        sample = sample << 4;
                        sample = sample >> 4;
                        tempPacket.samples[ch][samplesCollected].i = sample;

                        //Q sample
                        sample = pktStart[b + 2 + 3 * ch] << 4;
                        sample |= (pktStart[b + 1 + 3 * ch] >> 4) & 0x0F;
                        sample = sample << 4;
                        sample = sample >> 4;
                        tempPacket.samples[ch][samplesCollected].q = sample;
                    }
                    ++samplesCollected;
                    ++samplesReceived;
                }
                tempPacket.last = samplesCollected;

                uint32_t samplesPushed = rxFIFO->push_samples((const complex16_t**)tempPacket.samples, samplesCollected, channelsCount, tempPacket.timestamp, 10, statusFlags);
                if (samplesPushed != samplesCollected)
                {
                    rxDroppedSamples += samplesCollected - samplesPushed;
//                    if (report)
//                        report(STATUS_FLAG_RX_DROP, pkt[pktIndex].counter);
                }
                samplesCollected = 0;
            }
        }
        else
        {
            ++m_bufferFailures;
        }
        t2 = chrono::high_resolution_clock::now();
        auto timePeriod = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        if (timePeriod >= 1000)
        {
            //total number of bytes sent per second
            double dataRate = 1000.0*totalBytesReceived / timePeriod;
            //total number of samples from all channels per second
            float samplingRate = 1000.0*samplesReceived / timePeriod;
            samplesReceived = 0;
            t1 = t2;
            totalBytesReceived = 0;
            if (dataRate_Bps)
                dataRate_Bps->store((long)dataRate);
            m_bufferFailures = 0;

#ifndef NDEBUG
            printf("Rx rate: %.3f MB/s Fs: %.3f MHz | dropped samples: %lu\n", dataRate / 1000000.0, samplingRate / 1000000.0, rxDroppedSamples);
#endif
            rxDroppedSamples = 0;
        }

        // Re-submit this request to keep the queue full
        memset(&buffers[bi*bufferSize], 0, bufferSize);
        handles[bi] = dataPort->BeginDataReading(&buffers[bi*bufferSize], bufferSize);
        bi = (bi + 1) & buffersCountMask;
    }
    dataPort->AbortReading();
    for (int j = 0; j<buffersCount; j++)
    {
        long bytesToRead = bufferSize;
        dataPort->WaitForReading(handles[j], 1000);
        dataPort->FinishDataReading(&buffers[j*bufferSize], bytesToRead, handles[j]);
    }
}
