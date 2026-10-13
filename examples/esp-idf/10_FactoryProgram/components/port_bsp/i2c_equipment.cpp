#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include "i2c_equipment.h"
#include "i2c_bsp.h"
#include "SensorPCF85063.hpp"

Shtc3Port::Shtc3Port(I2cMasterBus& i2cbus) :
i2cbus_(i2cbus) {
    i2c_master_bus_handle_t I2cMasterBus = i2cbus_.Get_I2cBusHandle();
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = Shtc3Address;
    dev_cfg.scl_speed_hz    = 400000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(I2cMasterBus, &dev_cfg, &I2c_DevShtc3));

    Shtc3_Wakeup();
    Shtc3_SoftReset();
    vTaskDelay(pdMS_TO_TICKS(20)); //20MS
    Shtc3_GetId();
    ESP_LOGI(TAG, "ID:%04x", shtc3_id);
}

Shtc3Port::~Shtc3Port() {
}

etError Shtc3Port::Shtc3_GetId() {
    uint8_t senBuf[2]  = {(READ_ID >> 8), (READ_ID & 0xff)};
    uint8_t readBuf[3] = {0, 0, 0};
    int     err        = i2cbus_.i2c_master_write_read_dev(I2c_DevShtc3, senBuf, 2, readBuf, 3);
    etError error      = (err == ESP_OK) ? NO_ERROR : ACK_ERROR;
    if (error != NO_ERROR) {
        ESP_LOGE("shtc3", "GetId WRITE Failure");
        return error;
    }
    error = Shtc3_CheckCrc(readBuf, 2, readBuf[2]);
    if (error != NO_ERROR) {
        ESP_LOGE("shtc3", "GetId CRC Failure");
        return error;
    }
    shtc3_id = ((readBuf[0] << 8) | readBuf[1]);
    return error;
}

uint16_t Shtc3Port::Shtc3_GetShtc3Id() {
    return shtc3_id;
}

// wake up the sensor from sleep mode
etError Shtc3Port::Shtc3_Wakeup() {
    uint8_t senBuf[2] = {(WAKEUP >> 8), (WAKEUP & 0xff)};
    int     err       = i2cbus_.i2c_write_buff(I2c_DevShtc3, -1, senBuf, 2);
    etError error     = (err == ESP_OK) ? NO_ERROR : ACK_ERROR;
    //esp_rom_delay_us(100); //100us
    vTaskDelay(pdMS_TO_TICKS(50)); //50MS
    if (error != NO_ERROR)
        ESP_LOGE("shtc3", "Wakeup Failure");
    return error;
}

etError Shtc3Port::Shtc3_SoftReset() {
    uint8_t senBuf[2] = {(SOFT_RESET >> 8), (SOFT_RESET & 0xff)};
    int     err       = i2cbus_.i2c_write_buff(I2c_DevShtc3, -1, senBuf, 2);
    etError error     = (err == ESP_OK) ? NO_ERROR : ACK_ERROR;
    if (error != NO_ERROR)
        ESP_LOGE("shtc3", "SoftReset Failure");
    return error;
}

etError Shtc3Port::Shtc3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum) {
    uint8_t bit;        // bit mask
    uint8_t crc = 0xFF; // calculated checksum
    uint8_t byteCtr;    // byte counter

    // calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for (bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }

    // verify checksum
    if (crc != checksum) {
        return CHECKSUM_ERROR;
    } else {
        return NO_ERROR;
    }
}

float Shtc3Port::Shtc3_CalcTemperature(uint16_t rawValue) {
    // calculate temperature [°C]
    // T = -45 + 175 * rawValue / 2^16
    return 175 * (float) rawValue / 65536.0f - 45.0f - SHTC3_PETP_VOL;
}

float Shtc3Port::Shtc3_CalcHumidity(uint16_t rawValue) {
    // calculate relative humidity [%RH]
    // RH = rawValue / 2^16 * 100
    return 100 * (float) rawValue / 65536.0f;
}

etError Shtc3Port::Shtc3_GetTempAndHumiPolling(float *temp, float *humi) {
    int      err = 0;
    etError  error;        // error code
    uint16_t rawValueTemp; // temperature raw value from sensor
    uint16_t rawValueHumi; // humidity raw value from sensor
    uint8_t  bytes[6] = {0};
    ;
    uint8_t senBuf[2] = {(MEAS_T_RH_POLLING >> 8), (MEAS_T_RH_POLLING & 0xff)};
    err               = i2cbus_.i2c_write_buff(I2c_DevShtc3, -1, senBuf, 2);
    error             = (err == ESP_OK) ? NO_ERROR : ACK_ERROR;
    if (error != NO_ERROR) {
        ESP_LOGE("shtc3", "GetTempAndHumi WRITE Failure");
        return error;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    // if no error, read temperature and humidity raw values
    err   = i2cbus_.i2c_read_buff(I2c_DevShtc3, -1, bytes, 6);
    error = (err == ESP_OK) ? NO_ERROR : ACK_ERROR;
    if (error != NO_ERROR) {
        ESP_LOGE("shtc3", "GetTempAndHumi READ Failure");
        return error;
    }
    error = Shtc3_CheckCrc(bytes, 2, bytes[2]);
    if (error != NO_ERROR) {
        ESP_LOGE("shtc3", "GetTempAndHumi TempCRC Failure");
        return error;
    }
    error = Shtc3_CheckCrc(&bytes[3], 2, bytes[5]);
    if (error != NO_ERROR) {
        ESP_LOGE("shtc3", "GetTempAndHumi humidityCRC Failure");
        return error;
    }
    // if no error, calculate temperature in °C and humidity in %RH
    rawValueTemp = (bytes[0] << 8) | bytes[1];
    rawValueHumi = (bytes[3] << 8) | bytes[4];
    *temp        = Shtc3_CalcTemperature(rawValueTemp);
    *humi        = Shtc3_CalcHumidity(rawValueHumi);
    return error;
}

etError Shtc3Port::Shtc3_Sleep() {
    uint8_t senBuf[2] = {(SLEEP >> 8), (SLEEP & 0xff)};
    int     err       = i2cbus_.i2c_write_buff(I2c_DevShtc3, -1, senBuf, 2);
    etError error     = (err == ESP_OK) ? NO_ERROR : ACK_ERROR;
    if (error != NO_ERROR)
        ESP_LOGE("shtc3", "Sleep Failure");
    return error;
}

uint8_t Shtc3Port::Shtc3_ReadTempHumi(float *t,float *h) {
    etError      error;
    Shtc3_Wakeup();
    error = Shtc3_GetTempAndHumiPolling(t, h);
    if (error != NO_ERROR) {
        ESP_LOGW("shtc3", "error:%d", error);
    } else {
        return 0;
    }
    Shtc3_Sleep();
    return 1;
}

static i2c_master_dev_handle_t I2cRTCdev = NULL;
static uint8_t                 I2cRTCAddress;
static I2cMasterBus           *I2cbus_   = NULL;
SensorPCF85063 rtc;

static bool I2cDevCallback(uint8_t address, uint8_t reg, uint8_t *buf, size_t len, bool writeReg, bool isWrite) {
    int                     ret;
    i2c_master_dev_handle_t dev_handle = NULL;
    dev_handle = I2cRTCdev;
    if (isWrite) {
        if (writeReg) {
            ret = I2cbus_->i2c_write_buff(dev_handle, reg, buf, len);
        } else {
            ret = I2cbus_->i2c_write_buff(dev_handle, -1, buf, len);
        }
    } else {
        if (writeReg) {
            ret = I2cbus_->i2c_read_buff(dev_handle, reg, buf, len);
        } else {
            ret = I2cbus_->i2c_read_buff(dev_handle, -1, buf, len);
        }
    }
    return (ret == ESP_OK) ? true : false;
}

void Rtc_Setup(I2cMasterBus *i2cbus,uint8_t dev_addr) {
    if (I2cbus_ == NULL) {
        I2cbus_ = i2cbus;
    }
    if (I2cRTCdev == NULL) {
        i2c_master_bus_handle_t BusHandle = i2cbus->Get_I2cBusHandle();
        i2c_device_config_t     dev_cfg   = {};
        dev_cfg.dev_addr_length           = I2C_ADDR_BIT_LEN_7;
        dev_cfg.scl_speed_hz              = 300000;
        dev_cfg.device_address            = dev_addr;
        ESP_ERROR_CHECK(i2c_master_bus_add_device(BusHandle, &dev_cfg, &I2cRTCdev));
        I2cRTCAddress = dev_addr;
    }
    if (rtc.begin(I2cDevCallback)) {
        ESP_LOGI("rtc", "InitWill");
    } else {
        ESP_LOGE("rtc", "InitFailure");
    }
}

void Rtc_SetTime(uint16_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t second) {
    rtc.setDateTime(year, month, day, hour, minute, second);
}

void Rtc_GetTime(rtcTimeStruct_t *time) {
    RTC_DateTime  datetime = rtc.getDateTime();
    time->year              = datetime.getYear();
    time->month             = datetime.getMonth();
    time->day               = datetime.getDay();
    time->hour              = datetime.getHour();
    time->minute            = datetime.getMinute();
    time->second            = datetime.getSecond();
    time->week              = datetime.getWeek();
}