#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include "i2c_bsp.h"

static uint32_t i2c_data_pdMS_TICKS = 0;
static uint32_t i2c_done_pdMS_TICKS = 0;

I2cMasterBus::I2cMasterBus(int scl_pin,int sda_pin,int i2c_port) {
    i2c_data_pdMS_TICKS = pdMS_TO_TICKS(5000);
    i2c_done_pdMS_TICKS = pdMS_TO_TICKS(1000);

    i2c_master_bus_config_t i2c_bus_config      = {};
    i2c_bus_config.clk_source                   = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.i2c_port                     = (i2c_port_t)i2c_port;
    i2c_bus_config.scl_io_num                   = (gpio_num_t)scl_pin;
    i2c_bus_config.sda_io_num                   = (gpio_num_t)sda_pin;
    i2c_bus_config.glitch_ignore_cnt            = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &user_i2c_handle));
}

I2cMasterBus::~I2cMasterBus() {

}

int I2cMasterBus::i2c_write_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len) {
    int  ret;
    uint8_t *pbuf = NULL;
    ret           = i2c_master_bus_wait_all_done(user_i2c_handle, i2c_done_pdMS_TICKS);
    if (ret != ESP_OK)
        return ret;
    if (reg == -1) {
        ret = i2c_master_transmit(dev_handle, buf, len, i2c_data_pdMS_TICKS);
    } else {
        pbuf    = (uint8_t *) malloc(len + 1);
        pbuf[0] = reg;
        for (uint8_t i = 0; i < len; i++) {
            pbuf[i + 1] = buf[i];
        }
        ret = i2c_master_transmit(dev_handle, pbuf, len + 1, i2c_data_pdMS_TICKS);
        free(pbuf);
        pbuf = NULL;
    }
    return ret;
}

int I2cMasterBus::i2c_master_write_read_dev(i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen) {
    int ret;
    ret = i2c_master_bus_wait_all_done(user_i2c_handle, i2c_done_pdMS_TICKS);
    if (ret != ESP_OK)
        return ret;
    ret = i2c_master_transmit_receive(dev_handle, writeBuf, writeLen, readBuf, readLen, i2c_data_pdMS_TICKS);
    return ret;
}

int I2cMasterBus::i2c_read_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len) {
    int ret;
    uint8_t addr = 0;
    ret          = i2c_master_bus_wait_all_done(user_i2c_handle, i2c_done_pdMS_TICKS);
    if (ret != ESP_OK)
        return ret;
    if (reg == -1) {
        ret = i2c_master_receive(dev_handle, buf, len, i2c_data_pdMS_TICKS);
    } else {
        addr = (uint8_t) reg;
        ret  = i2c_master_transmit_receive(dev_handle, &addr, 1, buf, len, i2c_data_pdMS_TICKS);
    }
    return ret;
}

i2c_master_bus_handle_t I2cMasterBus::Get_I2cBusHandle() {
    return user_i2c_handle;
}