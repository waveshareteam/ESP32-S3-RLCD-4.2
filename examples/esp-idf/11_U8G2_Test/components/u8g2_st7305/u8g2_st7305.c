#include "u8g2_st7305.h"

#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ST7305_SPI_CLOCK_HZ 24000000
#define ST7305_TILE_WIDTH 38
#define ST7305_TILE_HEIGHT 50
#define ST7305_FULL_BUFFER_HEIGHT ST7305_TILE_HEIGHT
#define ST7305_BUFFER_ROW_BYTES (ST7305_TILE_WIDTH * 8)

static const char *TAG = "u8g2_st7305";

static const u8x8_display_info_t st7305_display_info = {
    /* chip_enable_level = */ 0,
    /* chip_disable_level = */ 1,
    /* post_chip_enable_wait_ns = */ 0,
    /* pre_chip_disable_wait_ns = */ 0,
    /* reset_pulse_width_ms = */ 20,
    /* post_reset_wait_ms = */ 50,
    /* sda_setup_time_ns = */ 0,
    /* sck_pulse_width_ns = */ 0,
    /* sck_clock_hz = */ ST7305_SPI_CLOCK_HZ,
    /* spi_mode = */ 0,
    /* i2c_bus_clock_100kHz = */ 4,
    /* data_setup_time_ns = */ 0,
    /* write_pulse_width_ns = */ 0,
    /* tile_width = */ ST7305_TILE_WIDTH,
    /* tile_height = */ ST7305_TILE_HEIGHT,
    /* default_x_offset = */ 0,
    /* flip_mode_x_offset = */ 0,
    /* pixel_width = */ 300,
    /* pixel_height = */ 400,
};

static esp_err_t st7305_spi_write(u8g2_st7305_t *dev, const uint8_t *data, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }

    spi_transaction_t tx = {
        .length = len * 8,
        .tx_buffer = data,
    };
    return spi_device_polling_transmit(dev->spi, &tx);
}

static esp_err_t st7305_write_cmd(u8g2_st7305_t *dev, uint8_t cmd)
{
    gpio_set_level(dev->dc_io, 0);
    gpio_set_level(dev->cs_io, 0);
    esp_err_t ret = st7305_spi_write(dev, &cmd, 1);
    gpio_set_level(dev->cs_io, 1);
    return ret;
}

static esp_err_t st7305_write_data(u8g2_st7305_t *dev, const uint8_t *data, size_t len)
{
    gpio_set_level(dev->dc_io, 1);
    gpio_set_level(dev->cs_io, 0);
    esp_err_t ret = st7305_spi_write(dev, data, len);
    gpio_set_level(dev->cs_io, 1);
    return ret;
}

static esp_err_t st7305_write_cmd_data(u8g2_st7305_t *dev, uint8_t cmd, const uint8_t *data, size_t len)
{
    gpio_set_level(dev->dc_io, 0);
    gpio_set_level(dev->cs_io, 0);
    esp_err_t ret = st7305_spi_write(dev, &cmd, 1);
    if (ret == ESP_OK && len > 0) {
        gpio_set_level(dev->dc_io, 1);
        ret = st7305_spi_write(dev, data, len);
    }
    gpio_set_level(dev->cs_io, 1);
    return ret;
}

static void st7305_reset(u8g2_st7305_t *dev)
{
    if (dev->reset_io < 0) {
        return;
    }

    gpio_set_level(dev->reset_io, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(dev->reset_io, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(dev->reset_io, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void st7305_full_init(u8g2_st7305_t *dev)
{
    st7305_reset(dev);

    const uint8_t d6[] = {0x17, 0x02};
    const uint8_t d1[] = {0x01};
    const uint8_t c0[] = {0x11, 0x04};
    const uint8_t c1[] = {0x69, 0x69, 0x69, 0x69};
    const uint8_t c2[] = {0x19, 0x19, 0x19, 0x19};
    const uint8_t c4[] = {0x4B, 0x4B, 0x4B, 0x4B};
    const uint8_t d8[] = {0x80, 0xE9};
    const uint8_t b2[] = {0x02};
    const uint8_t b3[] = {0xE5, 0xF6, 0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45};
    const uint8_t b4[] = {0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45};
    const uint8_t g_timing[] = {0x32, 0x03, 0x1F};
    const uint8_t b7[] = {0x13};
    const uint8_t b0[] = {0x64};
    const uint8_t c9[] = {0x00};
    const uint8_t m36[] = {0x48};
    const uint8_t m3a[] = {0x11};
    const uint8_t b9[] = {0x20};
    const uint8_t b8[] = {0x29};
    const uint8_t win_a[] = {0x12, 0x2A};
    const uint8_t win_b[] = {0x00, 0xC7};
    const uint8_t m35[] = {0x00};
    const uint8_t d0[] = {0xFF};

    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xD6, d6, sizeof(d6)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xD1, d1, sizeof(d1)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xC0, c0, sizeof(c0)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xC1, c1, sizeof(c1)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xC2, c2, sizeof(c2)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xC4, c4, sizeof(c4)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xC5, c2, sizeof(c2)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xD8, d8, sizeof(d8)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xB2, b2, sizeof(b2)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xB3, b3, sizeof(b3)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xB4, b4, sizeof(b4)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0x62, g_timing, sizeof(g_timing)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xB7, b7, sizeof(b7)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xB0, b0, sizeof(b0)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd(dev, 0x11));
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xC9, c9, sizeof(c9)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0x36, m36, sizeof(m36)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0x3A, m3a, sizeof(m3a)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xB9, b9, sizeof(b9)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xB8, b8, sizeof(b8)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd(dev, 0x21));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0x2A, win_a, sizeof(win_a)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0x2B, win_b, sizeof(win_b)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0x35, m35, sizeof(m35)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0xD0, d0, sizeof(d0)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd(dev, 0x38));
    ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd(dev, 0x29));
}

static uint8_t u8g2_st7305_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    u8g2_st7305_t *dev = (u8g2_st7305_t *)u8x8_GetUserPtr(u8x8);

    switch (msg) {
    case U8X8_MSG_BYTE_INIT:
        return 1;
    case U8X8_MSG_BYTE_START_TRANSFER:
        gpio_set_level(dev->cs_io, 0);
        return 1;
    case U8X8_MSG_BYTE_SEND:
        return st7305_spi_write(dev, (const uint8_t *)arg_ptr, arg_int) == ESP_OK;
    case U8X8_MSG_BYTE_END_TRANSFER:
        gpio_set_level(dev->cs_io, 1);
        return 1;
    case U8X8_MSG_BYTE_SET_DC:
        gpio_set_level(dev->dc_io, arg_int ? 1 : 0);
        return 1;
    default:
        return 0;
    }
}

static uint8_t u8g2_st7305_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)arg_ptr;
    u8g2_st7305_t *dev = (u8g2_st7305_t *)u8x8_GetUserPtr(u8x8);

    switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        return 1;
    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int));
        return 1;
    case U8X8_MSG_DELAY_10MICRO:
        esp_rom_delay_us((uint32_t)arg_int * 10U);
        return 1;
    case U8X8_MSG_DELAY_100NANO:
        esp_rom_delay_us(1);
        return 1;
    case U8X8_MSG_GPIO_CS:
        gpio_set_level(dev->cs_io, arg_int ? 1 : 0);
        return 1;
    case U8X8_MSG_GPIO_DC:
        gpio_set_level(dev->dc_io, arg_int ? 1 : 0);
        return 1;
    case U8X8_MSG_GPIO_RESET:
        if (dev->reset_io >= 0) {
            gpio_set_level(dev->reset_io, arg_int ? 1 : 0);
        }
        return 1;
    default:
        return 1;
    }
}

static uint8_t u8g2_st7305_display_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    u8g2_st7305_t *dev = (u8g2_st7305_t *)u8x8_GetUserPtr(u8x8);

    switch (msg) {
    case U8X8_MSG_DISPLAY_SETUP_MEMORY:
        u8x8_d_helper_display_setup_memory(u8x8, &st7305_display_info);
        return 1;
    case U8X8_MSG_DISPLAY_INIT:
        st7305_full_init(dev);
        return 1;
    case U8X8_MSG_DISPLAY_SET_POWER_SAVE:
        ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd(dev, arg_int == 0 ? 0x29 : 0x28));
        return 1;
    case U8X8_MSG_DISPLAY_DRAW_TILE: {
        u8x8_tile_t *tile = (u8x8_tile_t *)arg_ptr;
        uint8_t cnt = tile->cnt;
        uint8_t y_pos = tile->y_pos;
        uint8_t x_pos = tile->x_pos;

        int first_col = x_pos * 8;
        int last_col = (x_pos + cnt) * 8 - 1;
        if (last_col >= 300) {
            last_col = 299;
        }

        int addr_start = 0x12 + first_col / 12;
        int addr_end = 0x12 + last_col / 12;
        int send_start = (addr_start - 0x12) * 3;
        int send_cnt = (addr_end - addr_start + 1) * 3;

        int addr_first_col = (addr_start - 0x12) * 12;
        int addr_last_col = (addr_end - 0x12) * 12 + 11;
        if (addr_last_col >= 300) {
            addr_last_col = 299;
        }

        uint8_t *row_base = tile->tile_ptr - ((uint16_t)x_pos * 8U);
        uint8_t col_bounds[] = {(uint8_t)(0x3C - addr_end), (uint8_t)(0x3C - addr_start)};
        uint8_t row_bounds[] = {(uint8_t)(y_pos * 4), (uint8_t)(y_pos * 4 + 3)};

        static const uint8_t st_lut[4][4] = {
            {0x00, 0x80, 0x40, 0xC0},
            {0x00, 0x20, 0x10, 0x30},
            {0x00, 0x08, 0x04, 0x0C},
            {0x00, 0x02, 0x01, 0x03},
        };

        uint8_t all_rows[300] = {0};
        for (int sr = 0; sr < 4; sr++) {
            int shift = sr * 2;
            int base_off = sr * send_cnt;
            int idx = base_off + (addr_first_col >> 2) - send_start;

            for (int col = addr_first_col; col <= addr_last_col; col += 4, idx++) {
                all_rows[idx] = st_lut[0][(row_base[col] >> shift) & 3]
                              | st_lut[1][(row_base[col + 1] >> shift) & 3]
                              | st_lut[2][(row_base[col + 2] >> shift) & 3]
                              | st_lut[3][(row_base[col + 3] >> shift) & 3];
            }
        }

        ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0x2A, col_bounds, sizeof(col_bounds)));
        ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0x2B, row_bounds, sizeof(row_bounds)));
        ESP_ERROR_CHECK_WITHOUT_ABORT(st7305_write_cmd_data(dev, 0x2C, all_rows, (size_t)send_cnt * 4U));
        return 1;
    }
    default:
        return 0;
    }
}

u8g2_st7305_config_t u8g2_st7305_default_config(void)
{
    u8g2_st7305_config_t config = {
        .mosi_io = GPIO_NUM_12,
        .sclk_io = GPIO_NUM_11,
        .dc_io = GPIO_NUM_5,
        .cs_io = GPIO_NUM_40,
        .reset_io = GPIO_NUM_41,
        .spi_host = SPI3_HOST,
        .clock_hz = ST7305_SPI_CLOCK_HZ,
        .tile_buf_height = U8G2_ST7305_TILE_BUF_FULL,
        .rotation = U8G2_R1,
        .prefer_psram = true,
    };
    return config;
}

esp_err_t u8g2_st7305_init(u8g2_st7305_t *dev, const u8g2_st7305_config_t *config)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");

    memset(dev, 0, sizeof(*dev));
    dev->spi_host = config->spi_host;
    dev->dc_io = config->dc_io;
    dev->cs_io = config->cs_io;
    dev->reset_io = config->reset_io;

    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << config->dc_io) | (1ULL << config->cs_io),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (config->reset_io >= 0) {
        gpio_conf.pin_bit_mask |= (1ULL << config->reset_io);
    }
    ESP_RETURN_ON_ERROR(gpio_config(&gpio_conf), TAG, "gpio_config failed");
    gpio_set_level(config->cs_io, 1);
    gpio_set_level(config->dc_io, 1);
    if (config->reset_io >= 0) {
        gpio_set_level(config->reset_io, 1);
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = config->mosi_io,
        .miso_io_num = -1,
        .sclk_io_num = config->sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    esp_err_t ret = spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already initialized, reusing host %d", config->spi_host);
        ret = ESP_OK;
        dev->owns_spi_bus = false;
    } else {
        ESP_RETURN_ON_ERROR(ret, TAG, "spi_bus_initialize failed");
        dev->owns_spi_bus = true;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->clock_hz > 0 ? config->clock_hz : ST7305_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ret = spi_bus_add_device(config->spi_host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        if (dev->owns_spi_bus) {
            spi_bus_free(config->spi_host);
        }
        return ret;
    }

    dev->tile_buf_height = config->tile_buf_height == U8G2_ST7305_TILE_BUF_FULL
                               ? ST7305_FULL_BUFFER_HEIGHT
                               : config->tile_buf_height;
    if (dev->tile_buf_height == 0 || dev->tile_buf_height > ST7305_TILE_HEIGHT) {
        dev->tile_buf_height = ST7305_FULL_BUFFER_HEIGHT;
    }

    dev->buffer_size = ST7305_BUFFER_ROW_BYTES * dev->tile_buf_height;
    uint32_t caps = MALLOC_CAP_8BIT;
    if (config->prefer_psram) {
        caps |= MALLOC_CAP_SPIRAM;
    }
    dev->buffer = (uint8_t *)heap_caps_malloc(dev->buffer_size, caps);
    if (dev->buffer == NULL && config->prefer_psram) {
        dev->buffer = (uint8_t *)heap_caps_malloc(dev->buffer_size, MALLOC_CAP_8BIT);
    }
    if (dev->buffer == NULL) {
        u8g2_st7305_deinit(dev);
        return ESP_ERR_NO_MEM;
    }
    memset(dev->buffer, 0, dev->buffer_size);

    const u8g2_cb_t *rotation = config->rotation != NULL ? config->rotation : U8G2_R0;
    u8g2_SetupDisplay(&dev->u8g2, u8g2_st7305_display_cb, u8x8_cad_empty,
                      u8g2_st7305_byte_cb, u8g2_st7305_gpio_and_delay_cb);
    u8g2_SetUserPtr(&dev->u8g2, dev);
    u8g2_SetupBuffer(&dev->u8g2, dev->buffer, dev->tile_buf_height,
                     u8g2_ll_hvline_vertical_top_lsb, rotation);
    u8g2_InitDisplay(&dev->u8g2);
    u8g2_SetPowerSave(&dev->u8g2, 0);

    return ESP_OK;
}

void u8g2_st7305_deinit(u8g2_st7305_t *dev)
{
    if (dev == NULL) {
        return;
    }

    if (dev->spi != NULL) {
        spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
    }
    if (dev->owns_spi_bus) {
        spi_bus_free(dev->spi_host);
        dev->owns_spi_bus = false;
    }
    if (dev->buffer != NULL) {
        heap_caps_free(dev->buffer);
        dev->buffer = NULL;
    }
}
