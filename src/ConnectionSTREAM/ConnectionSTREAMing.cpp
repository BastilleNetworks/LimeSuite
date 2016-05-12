/**
    @file ConnectionSTREAMing.cpp
    @author Lime Microsystems
    @brief Implementation of STREAM board connection (streaming API)
*/

#include "ConnectionSTREAM.h"
#include "fifo.h"
#include <LMS7002M.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <complex>
#include <ciso646>
#include <FPGA_common.h>
#include "ErrorReporting.h"

using namespace lime;

#define LTE_CHAN_COUNT 2
#define STREAM_MTU (PacketFrame::maxSamplesInPacket/(LTE_CHAN_COUNT))

/***********************************************************************
 * Streaming API implementation
 **********************************************************************/

std::string ConnectionSTREAM::SetupStream(size_t &streamID, const StreamConfig &config)
{
    streamID = ~0;

    //API format check
    bool convertFloat = false;
    if (config.format == StreamConfig::STREAM_COMPLEX_FLOAT32) convertFloat = true;
    else if (config.format == StreamConfig::STREAM_12_BIT_IN_16) convertFloat = false;
    else return "ConnectionSTREAM::setupStream() only complex floats or int16";

    //check channel config
    //provide a default channel 0 if none specified
    std::vector<size_t> channels(config.channels);
    if (channels.empty()) channels.push_back(0);
    if (channels.size() > 2) return "SetupStream() 2 channels max";
    if (channels.front() > 1) return "SetupStream() channels must be [0, 1]";
    if (channels.back() > 1) return "SetupStream() channels must be [0, 1]";
    bool pos0isA = channels.front() == 0;
    bool pos1isA = channels.back() == 0;

    //determine sample positions based on channels
//    auto s3 = LMS7002M::BI
//    auto s2 = LMS7002M::BQ
//    auto s1 = LMS7002M::AI;
//    auto s0 = LMS7002M::AQ;

    //configure LML based on channel config
    LMS7002M rfic;
    rfic.SetConnection(this);
    rfic.Modify_SPI_Reg_bits(LML1_MODE, 0, true);
    rfic.Modify_SPI_Reg_bits(LML2_MODE, 0, true);
//    if (config.isTx) rfic.ConfigureLML_BB2RF(s0, s1, s2, s3);
//    else             rfic.ConfigureLML_RF2BB(s0, s1, s2, s3);

    if(config.isTx)
    {
        if(mTxService.isRunning())
            return "Tx Stream is already active, stop it before changing config";
        mTxService.setup(config);
        streamID = size_t(&mTxService);
    }
    else
    {
        if(mRxService.isRunning())
            return "Rx Stream is already active, stop it before changing config";
        mRxService.setup(config);
        streamID = size_t(&mRxService);
    }

    return ""; //success
}

void ConnectionSTREAM::CloseStream(const size_t streamID)
{
    auto service = (StreamService*)streamID;
    service->stop();
    service->destroy();
}

size_t ConnectionSTREAM::GetStreamSize(const size_t streamID)
{
    auto service = (StreamService*)streamID;
    return (PacketFrame::maxSamplesInPacket/(service->mActiveStreamConfig.channels.size()));
}

bool ConnectionSTREAM::ControlStream(const size_t streamID, const bool enable, const size_t burstSize, const StreamMetadata &metadata)
{
    auto service = (StreamService*)streamID;
    if(enable && service->isRunning() == false)
        service->start();
    if(!enable && service->isRunning() == true)
        service->stop();
}

int ConnectionSTREAM::ReadStream(const size_t streamID, void * const *buffs, const size_t length, const long timeout_ms, StreamMetadata &metadata)
{
    auto service = (StreamService*)streamID;
    if(service == nullptr)
        return 0;

    const int channelsCnt = service->mActiveStreamConfig.channels.size();
    complex16_t** buffers = new complex16_t*[channelsCnt];
    for(int i=0; i<channelsCnt; ++i)
        buffers[i] = new complex16_t[length*2];

    if(service->FIFO == nullptr)
        return 0;
    uint32_t samplesPopped = service->FIFO->pop_samples((complex16_t**)buffers, length, channelsCnt, &metadata.timestamp, timeout_ms, nullptr);
    if(service->mActiveStreamConfig.format == StreamConfig::STREAM_12_BIT_IN_16)
    {
        complex16_t **destBuffer = (complex16_t**)buffs;
        for(int ch=0; ch<channelsCnt; ++ch)
        {
            for(int i=0; i<samplesPopped; ++i)
            {
                destBuffer[ch][i] = buffers[ch][i];
            }
            delete buffers[ch];
        }
    }
    else if(service->mActiveStreamConfig.format == StreamConfig::STREAM_COMPLEX_FLOAT32)
    {
        std::complex<float> **destBuffer = (std::complex<float>**)buffs;
        for(int ch=0; ch<channelsCnt; ++ch)
        {
            for(int i=0; i<samplesPopped; ++i)
            {
                destBuffer[ch][i] = std::complex<float>(buffers[ch][i].i, buffers[ch][i].q);
            }
            delete buffers[ch];
        }
    }
    delete buffers;
    //printf("Reading stream %i\n", samplesPopped);
    return samplesPopped;
}

int ConnectionSTREAM::WriteStream(const size_t streamID, const void * const *buffs, const size_t length, const long timeout_ms, const StreamMetadata &metadata)
{
    auto service = (StreamService*)streamID;
    if(service == nullptr)
        return 0;

    const int channelsCnt = service->mActiveStreamConfig.channels.size();
    if(service->FIFO == nullptr)
        return 0;
    uint32_t samplesPushed = 0;
    if(service->mActiveStreamConfig.format == StreamConfig::STREAM_12_BIT_IN_16)
    {
        complex16_t** buffers = new complex16_t*[channelsCnt];
        for(int i=0; i<channelsCnt; ++i)
            buffers[i] = (complex16_t*)buffs[i];
        samplesPushed = service->FIFO->push_samples((const complex16_t**)buffers, length, channelsCnt, metadata.timestamp, timeout_ms, 0);
        delete buffers;
    }
    else if(service->mActiveStreamConfig.format == StreamConfig::STREAM_COMPLEX_FLOAT32)
    {
        std::complex<float>** src = (std::complex<float>**)buffs;
        complex16_t** buffers = new complex16_t*[channelsCnt];
        for(int i=0; i<channelsCnt; ++i)
        {
            buffers[i] = new complex16_t[length*2];
            for(int j=0; j<length; ++j)
            {
                buffers[i][j].i = src[i][j].real();
                buffers[i][j].q = src[i][j].imag();
            }
        }
        samplesPushed = service->FIFO->push_samples((const complex16_t**)buffers, length, channelsCnt, metadata.timestamp, timeout_ms, 0);
        for(int i=0; i<channelsCnt; ++i)
            delete []buffers[i];
        delete []buffers;
    }
    return samplesPushed;
}

int ConnectionSTREAM::ReadStreamStatus(const size_t streamID, const long timeout_ms, StreamMetadata &metadata)
{

}

/** @brief Configures FPGA PLLs to LimeLight interface frequency
*/
void ConnectionSTREAM::UpdateExternalDataRate(const size_t channel, const double txRate_Hz, const double rxRate_Hz)
{
#ifdef LMS_VERBOSE_OUTPUT
    std::cout << "ConnectionSTREAM::ConfigureFPGA_PLL(tx=" << txRate_Hz/1e6 << "MHz, rx=" << rxRate_Hz/1e6 << "MHz)" << std::endl;
#endif
    const float txInterfaceClk = 2 * txRate_Hz;
    const float rxInterfaceClk = 2 * rxRate_Hz;
    if(txInterfaceClk >= 5e6)
    {
        lime::fpga::FPGA_PLL_clock clocks[2];
        clocks[0].bypass = false;
        clocks[0].index = 0;
        clocks[0].outFrequency = txInterfaceClk;
        clocks[0].phaseShift_deg = 0;
        clocks[1].bypass = false;
        clocks[1].index = 1;
        clocks[1].outFrequency = txInterfaceClk;
        clocks[1].phaseShift_deg = 90;
        lime::fpga::SetPllFrequency(this, 0, txInterfaceClk, clocks, 2);
    }
    else
        lime::fpga::SetDirectClocking(this, 0, txInterfaceClk, 90);

    if(rxInterfaceClk >= 5e6)
    {
        lime::fpga::FPGA_PLL_clock clocks[2];
        clocks[0].bypass = false;
        clocks[0].index = 0;
        clocks[0].outFrequency = rxInterfaceClk;
        clocks[0].phaseShift_deg = 0;
        clocks[1].bypass = false;
        clocks[1].index = 1;
        clocks[1].outFrequency = rxInterfaceClk;
        clocks[1].phaseShift_deg = 90;
        lime::fpga::SetPllFrequency(this, 1, rxInterfaceClk, clocks, 2);
    }
    else
        lime::fpga::SetDirectClocking(this, 1, rxInterfaceClk, 90);
}

void ConnectionSTREAM::EnterSelfCalibration(const size_t channel)
{
    //if (mStreamService) mStreamService->stop();
}

void ConnectionSTREAM::ExitSelfCalibration(const size_t channel)
{
    //if (mStreamService) mStreamService->start();
}

uint64_t ConnectionSTREAM::GetHardwareTimestamp(void)
{
    //return mStreamService->mLastRxTimestamp + mStreamService->mTimestampOffset;
}

void ConnectionSTREAM::SetHardwareTimestamp(const uint64_t now)
{
    //mStreamService->mTimestampOffset = int64_t(now)-int64_t(mStreamService->mLastRxTimestamp);
}

double ConnectionSTREAM::GetHardwareTimestampRate(void)
{
    //return mStreamService->mHwCounterRate;
}
