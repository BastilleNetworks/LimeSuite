#include "gtest/gtest.h"
#include "streamingFixture.h"
#include "IConnection.h"
#include "LMS7002M.h"
#include "math.h"
#include <thread>
#include <chrono>
#include "dataTypes.h"
using namespace std;
using namespace lime;

TEST_F (StreamingFixture, RxChannelSwapping)
{
    for(int repeatCount=0; repeatCount<1; ++repeatCount)
    {
        LMS7002M lmsControl;
        lmsControl.SetConnection(serPort, 0);
        lmsControl.ResetChip();
        //load initial settings to get samples
        lmsControl.UploadAll();
        lmsControl.SetActiveChannel(LMS7002M::ChA);
        lmsControl.Modify_SPI_Reg_bits(EN_ADCCLKH_CLKGN, 0);
        lmsControl.Modify_SPI_Reg_bits(CLKH_OV_CLKL_CGEN, 2);
        lmsControl.SetFrequencySX(LMS7002M::Tx, 1e6);
        lmsControl.SetFrequencySX(LMS7002M::Rx, 1e6);
        lmsControl.Modify_SPI_Reg_bits(LML1_MODE, 0);
        lmsControl.Modify_SPI_Reg_bits(LML2_MODE, 0);
        lmsControl.Modify_SPI_Reg_bits(PD_RX_AFE2, 0);

        lmsControl.SetActiveChannel(LMS7002M::ChAB);
        lmsControl.Modify_SPI_Reg_bits(INSEL_RXTSP, 1);
        lmsControl.Modify_SPI_Reg_bits(GFIR1_BYP_RXTSP, 1);
        lmsControl.Modify_SPI_Reg_bits(GFIR2_BYP_RXTSP, 1);
        lmsControl.Modify_SPI_Reg_bits(GFIR3_BYP_RXTSP, 1);
        lmsControl.Modify_SPI_Reg_bits(AGC_BYP_RXTSP, 1);
        lmsControl.Modify_SPI_Reg_bits(CMIX_BYP_RXTSP, 1);

        lmsControl.SetActiveChannel(LMS7002M::ChA);
        lmsControl.Modify_SPI_Reg_bits(TSGFCW_RXTSP, 1);
        lmsControl.Modify_SPI_Reg_bits(TSGFC_RXTSP, 1);
        lmsControl.SetActiveChannel(LMS7002M::ChB);
        lmsControl.Modify_SPI_Reg_bits(TSGFCW_RXTSP, 1);
        lmsControl.Modify_SPI_Reg_bits(TSGFC_RXTSP, 0);
        lmsControl.SetActiveChannel(LMS7002M::ChA);

        int ch = lmsControl.GetActiveChannelIndex();
        float cgenFreq = 30.72e6 * 2;
        lmsControl.SetInterfaceFrequency(cgenFreq, 0, 0);
        auto txRate = lmsControl.GetSampleRate(LMS7002M::Tx);
        auto rxRate = lmsControl.GetSampleRate(LMS7002M::Rx);
        serPort->UpdateExternalDataRate(ch, txRate, rxRate);

        //setup streaming
        size_t streamId;
        StreamConfig config;
        config.channels.push_back(0);
        config.channels.push_back(1);
        config.isTx = false;
        config.format = StreamConfig::StreamDataFormat::STREAM_12_BIT_IN_16;
        config.linkFormat = StreamConfig::StreamDataFormat::STREAM_12_BIT_COMPRESSED;
        serPort->SetHardwareTimestamp(0);
        //create streaming channel
        serPort->SetupStream(streamId, config);
        ASSERT_NE(streamId, size_t(~0));

        StreamMetadata metadata;
        metadata.hasTimestamp = false;
        metadata.endOfBurst = false;
        auto ok = serPort->ControlStream(streamId, true, 680, metadata);

        const int chCount = 2;
        int16_t** buffers = new int16_t*[chCount];
        const int streamsize = serPort->GetStreamSize(streamId);
        for(int i=0; i<chCount; ++i)
            buffers[i] = new int16_t[streamsize*2];

        //Both channels are set to receive test signal
        //A channel full scale, B channel -6 dB
        //A channel amplitude should always be bigger than channel B
        //otherwise it would show channels data swapping
        //this_thread::sleep_for(chrono::seconds(3));
        bool samplesValid = true;
        bool channelsSwapped = false;
        int sampleIndex = 0;
        int samplesRead = serPort->ReadStream(streamId, (void * const *)buffers, streamsize, 1000, metadata);

        for(int cnt=0; cnt<1; ++cnt)
        {
            int samplesRead = serPort->ReadStream(streamId, (void * const *)buffers, streamsize, 1000, metadata);
            ASSERT_EQ(streamsize, samplesRead);

            for(int i=0; i < samplesRead; i+=2)
            {
                //compare amplitudes from each channel, should be A > B
                float ampA = pow(buffers[0][i], 2) + pow(buffers[0][i+1], 2);
                float ampB = pow(buffers[1][i], 2) + pow(buffers[1][i+1], 2);
                //make sure both channels are receiving valid samples
                if(ampA == 0 || ampB == 0)
                {
                    samplesValid = false;
                    printf("AmpA : %g \t AmpB: %g, @ %i\n", ampA, ampB, sampleIndex);
                    break;
                }
                if(ampA < ampB)
                {
                    channelsSwapped = true;
                    printf("AmpA : %g \t AmpB: %g @ %i\n", ampA, ampB, sampleIndex);
                    printf("AmpA : %i %i \t AmpB: %i %i\n", buffers[0][i], buffers[0][i+1], buffers[1][i], buffers[1][i+1]);
                    break;
                }
                ++sampleIndex;
            }
        }
        EXPECT_EQ(true, samplesValid);
        EXPECT_EQ(false, channelsSwapped);

        //this_thread::sleep_for(chrono::milliseconds(800));
        ok = serPort->ControlStream(streamId, false, 680, metadata);
        serPort->CloseStream(streamId);
        for(int i=0; i<chCount; ++i)
            delete buffers[i];
        delete buffers;
    }
}

TEST_F (StreamingFixture, transmitSamples)
{
    LMS7002M lmsControl;
    lmsControl.SetConnection(serPort, 0);
    lmsControl.ResetChip();
    //load initial settings to get samples
    lmsControl.UploadAll();
    lmsControl.SetActiveChannel(LMS7002M::ChA);
    lmsControl.Modify_SPI_Reg_bits(EN_ADCCLKH_CLKGN, 0);
    lmsControl.Modify_SPI_Reg_bits(CLKH_OV_CLKL_CGEN, 2);
    lmsControl.SetFrequencySX(LMS7002M::Tx, 1e6);
    lmsControl.SetFrequencySX(LMS7002M::Rx, 1e6);
    lmsControl.Modify_SPI_Reg_bits(LML1_MODE, 0);
    lmsControl.Modify_SPI_Reg_bits(LML2_MODE, 0);
    lmsControl.Modify_SPI_Reg_bits(PD_RX_AFE2, 0);

    lmsControl.SetActiveChannel(LMS7002M::ChAB);
    lmsControl.Modify_SPI_Reg_bits(GFIR1_BYP_TXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(GFIR2_BYP_TXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(GFIR3_BYP_TXTSP, 1);

    lmsControl.SetActiveChannel(LMS7002M::ChA);

    int ch = lmsControl.GetActiveChannelIndex();
    float cgenFreq = 30.72e6 * 2;
    lmsControl.SetInterfaceFrequency(cgenFreq, 0, 0);
    auto txRate = lmsControl.GetSampleRate(LMS7002M::Tx);
    auto rxRate = lmsControl.GetSampleRate(LMS7002M::Rx);
    serPort->UpdateExternalDataRate(ch, txRate, rxRate);

    //setup streaming
    size_t streamId;
    StreamConfig config;
    config.channels.push_back(0);
    config.channels.push_back(1);
    config.isTx = true;
    config.format = StreamConfig::STREAM_12_BIT_IN_16;
    config.linkFormat = StreamConfig::StreamDataFormat::STREAM_12_BIT_COMPRESSED;
    serPort->SetHardwareTimestamp(0);
    //create streaming channel
    serPort->SetupStream(streamId, config);
    ASSERT_NE(streamId, size_t(~0));

    StreamMetadata metadata;
    metadata.hasTimestamp = false;
    metadata.endOfBurst = false;
    auto ok = serPort->ControlStream(streamId, true, 680, metadata);

    const int chCount = 2;
    int16_t** buffers = new int16_t*[chCount];
    const int streamsize = serPort->GetStreamSize(streamId)*20;
    for(int i=0; i<chCount; ++i)
        buffers[i] = new int16_t[streamsize*2];

    for(int i=0; i<3; ++i)
    {
        int samplesWrite = serPort->WriteStream(streamId, (void * const *)buffers, streamsize, 1000, metadata);
        EXPECT_EQ(streamsize, samplesWrite);
    }
    ok = serPort->ControlStream(streamId, false, 680, metadata);
    serPort->CloseStream(streamId);
    for(int i=0; i<chCount; ++i)
        delete buffers[i];
    delete buffers;
}

TEST_F (StreamingFixture, Rx2TxLoop)
{
    LMS7002M lmsControl;
    lmsControl.SetConnection(serPort, 0);
    lmsControl.ResetChip();
    //load initial settings to get samples
    lmsControl.UploadAll();
    lmsControl.SetActiveChannel(LMS7002M::ChA);
    lmsControl.Modify_SPI_Reg_bits(EN_ADCCLKH_CLKGN, 0);
    lmsControl.Modify_SPI_Reg_bits(CLKH_OV_CLKL_CGEN, 2);
    lmsControl.SetFrequencySX(LMS7002M::Tx, 1e6);
    lmsControl.SetFrequencySX(LMS7002M::Rx, 1e6);
    lmsControl.Modify_SPI_Reg_bits(LML1_MODE, 0);
    lmsControl.Modify_SPI_Reg_bits(LML2_MODE, 0);
    lmsControl.Modify_SPI_Reg_bits(PD_RX_AFE2, 0);

    lmsControl.SetActiveChannel(LMS7002M::ChAB);
    lmsControl.Modify_SPI_Reg_bits(INSEL_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(GFIR1_BYP_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(GFIR2_BYP_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(GFIR3_BYP_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(AGC_BYP_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(CMIX_BYP_RXTSP, 1);

    lmsControl.SetActiveChannel(LMS7002M::ChA);
    lmsControl.Modify_SPI_Reg_bits(TSGFCW_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(TSGFC_RXTSP, 1);
    lmsControl.SetActiveChannel(LMS7002M::ChB);
    lmsControl.Modify_SPI_Reg_bits(TSGFCW_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(TSGFC_RXTSP, 0);
    lmsControl.SetActiveChannel(LMS7002M::ChA);

    int ch = lmsControl.GetActiveChannelIndex();
    float cgenFreq = 30.72e6 * 2;
    lmsControl.SetInterfaceFrequency(cgenFreq, 0, 0);
    auto txRate = lmsControl.GetSampleRate(LMS7002M::Tx);
    auto rxRate = lmsControl.GetSampleRate(LMS7002M::Rx);
    serPort->UpdateExternalDataRate(ch, txRate, rxRate);

    //setup streaming
    size_t rxstreamId;
    size_t txstreamId;
    StreamConfig config;
    config.channels.push_back(0);
    config.channels.push_back(1);
    config.isTx = false;
    config.format = StreamConfig::StreamDataFormat::STREAM_12_BIT_IN_16;
    config.linkFormat = StreamConfig::StreamDataFormat::STREAM_12_BIT_COMPRESSED;
    config.bufferLength = 65536;
    serPort->SetHardwareTimestamp(0);
    //create streaming channel
    serPort->SetupStream(rxstreamId, config);
    ASSERT_NE(rxstreamId, size_t(~0));

    serPort->SetupStream(txstreamId, config);
    ASSERT_NE(txstreamId, size_t(~0));

    StreamMetadata metadata;
    metadata.hasTimestamp = false;
    metadata.endOfBurst = false;
    auto ok = serPort->ControlStream(rxstreamId, true, 680, metadata);
    ok = serPort->ControlStream(txstreamId, true, 680, metadata);

    const int chCount = 2;
    int16_t** buffers = new int16_t*[chCount];
    const int streamsize = serPort->GetStreamSize(rxstreamId);
    for(int i=0; i<chCount; ++i)
        buffers[i] = new int16_t[streamsize*2];

    for(int cnt=0; cnt<10; ++cnt)
    {
        int samplesRead = serPort->ReadStream(rxstreamId, (void * const *)buffers, streamsize, 1000, metadata);
        ASSERT_EQ(streamsize, samplesRead);
        int samplesWrite = serPort->WriteStream(txstreamId, (void * const *)buffers, streamsize, 1000, metadata);
        ASSERT_EQ(streamsize, samplesWrite);
    }

    ok = serPort->ControlStream(rxstreamId, false, 680, metadata);
    ok = serPort->ControlStream(txstreamId, false, 680, metadata);
    serPort->CloseStream(rxstreamId);
    serPort->CloseStream(txstreamId);
    for(int i=0; i<chCount; ++i)
        delete buffers[i];
    delete buffers;
}


TEST_F (StreamingFixture, RxBursts)
{
    LMS7002M lmsControl;
    lmsControl.SetConnection(serPort, 0);
    lmsControl.ResetChip();
    //load initial settings to get samples
    lmsControl.UploadAll();
    lmsControl.SetActiveChannel(LMS7002M::ChA);
    lmsControl.Modify_SPI_Reg_bits(EN_ADCCLKH_CLKGN, 0);
    lmsControl.Modify_SPI_Reg_bits(CLKH_OV_CLKL_CGEN, 2);
    lmsControl.SetFrequencySX(LMS7002M::Tx, 1e6);
    lmsControl.SetFrequencySX(LMS7002M::Rx, 1e6);
    lmsControl.Modify_SPI_Reg_bits(LML1_MODE, 0);
    lmsControl.Modify_SPI_Reg_bits(LML2_MODE, 0);
    lmsControl.Modify_SPI_Reg_bits(PD_RX_AFE2, 0);

    lmsControl.SetActiveChannel(LMS7002M::ChAB);
    lmsControl.Modify_SPI_Reg_bits(INSEL_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(GFIR1_BYP_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(GFIR2_BYP_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(GFIR3_BYP_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(AGC_BYP_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(CMIX_BYP_RXTSP, 1);

    lmsControl.SetActiveChannel(LMS7002M::ChA);
    lmsControl.Modify_SPI_Reg_bits(TSGFCW_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(TSGFC_RXTSP, 1);
    lmsControl.SetActiveChannel(LMS7002M::ChB);
    lmsControl.Modify_SPI_Reg_bits(TSGFCW_RXTSP, 1);
    lmsControl.Modify_SPI_Reg_bits(TSGFC_RXTSP, 0);
    lmsControl.SetActiveChannel(LMS7002M::ChA);

    int ch = lmsControl.GetActiveChannelIndex();
    float cgenFreq = 30.72e6 * 2;
    lmsControl.SetInterfaceFrequency(cgenFreq, 0, 0);
    auto txRate = lmsControl.GetSampleRate(LMS7002M::Tx);
    auto rxRate = lmsControl.GetSampleRate(LMS7002M::Rx);
    serPort->UpdateExternalDataRate(ch, txRate, rxRate);

    //setup streaming
    size_t streamId;
    StreamConfig config;
    config.channels.push_back(0);
    config.channels.push_back(1);
    config.isTx = false;
    config.format = StreamConfig::StreamDataFormat::STREAM_12_BIT_IN_16;
    config.linkFormat = StreamConfig::StreamDataFormat::STREAM_12_BIT_COMPRESSED;
    serPort->SetHardwareTimestamp(0);
    //create streaming channel
    serPort->SetupStream(streamId, config);
    ASSERT_NE(streamId, size_t(~0));

    StreamMetadata metadata;
    metadata.hasTimestamp = false;
    metadata.endOfBurst = true;
    const int burstLength = 680*10;
    auto ok = serPort->ControlStream(streamId, true, burstLength, metadata);

    const int chCount = 2;
    int16_t** buffers = new int16_t*[chCount];
    for(int i=0; i<chCount; ++i)
        buffers[i] = new int16_t[burstLength*2];

    bool samplesValid = true;
    bool channelsSwapped = false;

    //first burst read should succeed
    int samplesRead = serPort->ReadStream(streamId, (void * const *)buffers, burstLength, 200, metadata);
    ASSERT_EQ(burstLength, samplesRead);

    //confirm samples
    for(int cnt=0; cnt<1; ++cnt)
    {
        for(int i=0; i < samplesRead; i+=2)
        {
            //compare amplitudes from each channel, should be A > B
            float ampA = pow(buffers[0][i], 2) + pow(buffers[0][i+1], 2);
            float ampB = pow(buffers[1][i], 2) + pow(buffers[1][i+1], 2);
            //make sure both channels are receiving valid samples
            if(ampA == 0 || ampB == 0)
            {
                samplesValid = false;
                printf("AmpA : %g \t AmpB: %g\n", ampA, ampB);
                break;
            }
            if(ampA < ampB)
            {
                channelsSwapped = true;
                printf("AmpA : %g \t AmpB: %g\n", ampA, ampB);
                printf("AmpA : %i %i \t AmpB: %i %i\n", buffers[0][i], buffers[0][i+1], buffers[1][i], buffers[1][i+1]);
                break;
            }
        }
    }
    EXPECT_EQ(true, samplesValid);
    EXPECT_EQ(false, channelsSwapped);

    //second read attempt should fail and return 0 samples
    samplesRead = serPort->ReadStream(streamId, (void * const *)buffers, burstLength, 200, metadata);
    ASSERT_EQ(0, samplesRead);

    ok = serPort->ControlStream(streamId, true, burstLength, metadata);

    //third read should succeed again
    samplesRead = serPort->ReadStream(streamId, (void * const *)buffers, burstLength, 200, metadata);
    ASSERT_EQ(burstLength, samplesRead);

    //confirm samples
    samplesValid = true;
    channelsSwapped = false;
    for(int cnt=0; cnt<1; ++cnt)
    {
        for(int i=0; i < samplesRead; i+=2)
        {
            //compare amplitudes from each channel, should be A > B
            float ampA = pow(buffers[0][i], 2) + pow(buffers[0][i+1], 2);
            float ampB = pow(buffers[1][i], 2) + pow(buffers[1][i+1], 2);
            //make sure both channels are receiving valid samples
            if(ampA == 0 || ampB == 0)
            {
                samplesValid = false;
                printf("AmpA : %g \t AmpB: %g, \n", ampA, ampB);
                break;
            }
            if(ampA < ampB)
            {
                channelsSwapped = true;
                printf("AmpA : %g \t AmpB: %g \n", ampA, ampB);
                printf("AmpA : %i %i \t AmpB: %i %i\n", buffers[0][i], buffers[0][i+1], buffers[1][i], buffers[1][i+1]);
                break;
            }
        }
    }
    EXPECT_EQ(true, samplesValid);
    EXPECT_EQ(false, channelsSwapped);

    ok = serPort->ControlStream(streamId, false, 680, metadata);
    serPort->CloseStream(streamId);
    for(int i=0; i<chCount; ++i)
        delete buffers[i];
    delete buffers;
}
