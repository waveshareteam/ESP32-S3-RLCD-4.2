#pragma once

#include "codec_board.h"
#include "codec_init.h"
#include "i2c_bsp.h"

class CodecPort
{
private:
    esp_codec_dev_handle_t playback = NULL;
    esp_codec_dev_handle_t record = NULL;
    I2cMasterBus& i2cbus_;
    i2c_master_dev_handle_t I2c_DevEs8311;
    i2c_master_dev_handle_t I2c_DevEs7210;
    const uint8_t Es8311Address = 0x18;
    const uint8_t Es7210Address = 0x40;

    static void CodecPort_MusicTask(void *arg);
    static void CodecPort_EchoTask(void *arg);
public:
    CodecPort(I2cMasterBus& i2cbus,const char *strName);
    ~CodecPort();
    
    void Codec_SetCodecReg(const char * str,uint8_t reg,uint8_t data);
    uint8_t Codec_GetCodecReg(const char *str, uint8_t reg);

    void CodecPort_SetSpeakerVol(int vol);
    void CodecPort_SetMicGain(float db_value);

    void CodecPort_CloseSpeaker(void);
    void CodecPort_CloseMic(void);

    int CodecPort_PlayWrite(void *ptr,int ptr_len);
    int CodecPort_EchoRead(void *ptr,int ptr_len);

    void CodecPort_CreateMusicTask(void);
    void CodecPort_CreateEchoTask(void);

    void CodecPort_SetInfo(const char *strName,int open_en,int sample_rate,int channel,int bits_per_sample);

    uint8_t *CodecPort_GetPcmData(uint32_t *len);

};

