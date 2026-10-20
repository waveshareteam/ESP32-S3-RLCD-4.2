#pragma once

#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>

class CustomSDPort
{
private:
    const char *TAG = "SDPort";
    const char *SdName_;
    int is_SdcardInitOK = 0;
    sdmmc_card_t *sdcard_host = NULL;
public:
    CustomSDPort(const char *SdName,int clk = 38,int cmd = 21,int d0 = 39,int width = 1);
    ~CustomSDPort();

    int SDPort_GetStatus() {return is_SdcardInitOK;}
    int SDPort_WriteFile(const char *path, const void *data, size_t data_len);
    int SDPort_ReadFile(const char *path, uint8_t *buffer, size_t *outLen);
    int SDPort_ReadOffset(const char *path, void *buffer, size_t len, size_t offset);
    int SDPort_WriteOffset(const char *path, const void *data, size_t len, bool append);
};