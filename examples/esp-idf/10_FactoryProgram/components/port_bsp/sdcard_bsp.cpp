#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include "sdcard_bsp.h"

CustomSDPort::CustomSDPort(const char *SdName,int clk,int cmd,int d0,int width) :
SdName_(SdName)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed           = false;
    mount_config.max_files                        = 5;
    mount_config.allocation_unit_size             = 16 * 1024 * 3;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width               = width;
    slot_config.clk                 = (gpio_num_t)clk;
    slot_config.cmd                 = (gpio_num_t)cmd;
    slot_config.d0                  = (gpio_num_t)d0;

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_vfs_fat_sdmmc_mount(SdName_, &host, &slot_config, &mount_config, &sdcard_host));

    if (sdcard_host != NULL) {
        sdmmc_card_print_info(stdout, sdcard_host);
        is_SdcardInitOK = 1;
    } else {
        is_SdcardInitOK = 0;
    }
}

CustomSDPort::~CustomSDPort() {

}

int CustomSDPort::SDPort_WriteFile(const char *path, const void *data, size_t data_len) {
    if (sdcard_host == NULL) {
        ESP_LOGE(TAG, "SD card not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sdmmc_get_status(sdcard_host) != ESP_OK) {
        ESP_LOGE(TAG, "SD card not ready");
        return ESP_FAIL;
    }

    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    size_t written = fwrite(data, 1, data_len, f);
    fclose(f);

    if (written != data_len) {
        ESP_LOGE(TAG, "Write failed (%zu/%zu bytes)", written, data_len);
        return ESP_FAIL;
    }
    return ESP_OK;
}

int CustomSDPort::SDPort_ReadFile(const char *path, uint8_t *buffer, size_t *outLen) {
    if (sdcard_host == NULL) {
        ESP_LOGE(TAG, "SD card not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sdmmc_get_status(sdcard_host) != ESP_OK) {
        ESP_LOGE(TAG, "SD card not ready");
        return ESP_FAIL;
    }

    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    if (file_size <= 0) {
        ESP_LOGE(TAG, "Invalid file size");
        fclose(f);
        return ESP_FAIL;
    }
    fseek(f, 0, SEEK_SET);

    size_t bytes_read = fread(buffer, 1, file_size, f);
    fclose(f);

    if (outLen) *outLen = bytes_read;
    return (bytes_read > 0) ? ESP_OK : ESP_FAIL;
}

int CustomSDPort::SDPort_ReadOffset(const char *path, void *buffer, size_t len, size_t offset) {
    if (sdcard_host == NULL) {
        ESP_LOGE(TAG, "SD card not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sdmmc_get_status(sdcard_host) != ESP_OK) {
        ESP_LOGE(TAG, "SD card not ready");
        return ESP_FAIL;
    }

    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, offset, SEEK_SET);
    size_t bytes_read = fread(buffer, 1, len, f);
    fclose(f);
    return bytes_read;
}

int CustomSDPort::SDPort_WriteOffset(const char *path, const void *data, size_t len, bool append) {
    if (sdcard_host == NULL) {
        ESP_LOGE(TAG, "SD card not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sdmmc_get_status(sdcard_host) != ESP_OK) {
        ESP_LOGE(TAG, "SD card not ready");
        return ESP_FAIL;
    }

    const char *mode = append ? "ab" : "wb";
    FILE *f = fopen(path, mode);
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    size_t bytes_written = fwrite(data, 1, len, f);
    fclose(f);

    if (!append && len == 0) {
        ESP_LOGI(TAG, "File cleared: %s", path);
        return ESP_OK;
    }
    return bytes_written;
}