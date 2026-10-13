
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

#include "display_bsp.h"
#include "lvgl_bsp.h"
#include "user_app.h"
#include "user_config.h"
#include "u8g2_st7305.h"

#if !APP_USE_U8G2_DEMO
DisplayPort RlcdPort(12,11,5,40,41,LCD_WIDTH,LCD_HEIGHT);

static void Lvgl_FlushCallback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map)
{
  	uint16_t *buffer = (uint16_t *)color_map;
  	for(int y = area->y1; y <= area->y2; y++) 
  	{
  	 	for(int x = area->x1; x <= area->x2; x++) 
  	 	{
  	 	   	uint8_t color = (*buffer < 0x7fff) ? ColorBlack : ColorWhite;
  	 	   	RlcdPort.RLCD_SetPixel(x, y, color);
  	 	   	buffer++;
  	 	}
  	}
  	RlcdPort.RLCD_Display();
	lv_disp_flush_ready(drv);
}
#endif

#if APP_USE_U8G2_DEMO
static u8g2_st7305_t g_u8g2_lcd;
static const char *TAG = "u8g2_counter";
static constexpr uint32_t kWdtFeedFrameInterval = 16;

static void U8g2_DrawCenteredStr(u8g2_t *u8g2, int y, const char *text)
{
	int text_width = (int)u8g2_GetStrWidth(u8g2, text);
	int x = (LCD_WIDTH - text_width) / 2;
	if (x < 0) {
		x = 0;
	}
	u8g2_DrawStr(u8g2, x, y, text);
}

static void U8g2_RunCounterDemo(void)
{
	u8g2_st7305_config_t config = u8g2_st7305_default_config();
	config.mosi_io = RLCD_MOSI_PIN;
	config.sclk_io = RLCD_SCK_PIN;
	config.dc_io = RLCD_DC_PIN;
	config.cs_io = RLCD_CS_PIN;
	config.reset_io = RLCD_RST_PIN;
	config.rotation = U8G2_R1;
	config.tile_buf_height = U8G2_ST7305_TILE_BUF_FULL;

	ESP_ERROR_CHECK(u8g2_st7305_init(&g_u8g2_lcd, &config));

	u8g2_t *u8g2 = u8g2_st7305_get_u8g2(&g_u8g2_lcd);

	uint32_t counter = 0;
	uint32_t frames = 0;
	uint32_t last_report_frames = 0;
	int64_t last_report_us = esp_timer_get_time();
	uint32_t fps_x100 = 0;
	uint32_t frame_us = 0;
	uint32_t flush_us = 0;

	while (true) {
		char text[80];
		const int64_t frame_start_us = esp_timer_get_time();

		u8g2_ClearBuffer(u8g2);
		u8g2_SetDrawColor(u8g2, 1);

		snprintf(text, sizeof(text), "%lu", (unsigned long)counter);
		u8g2_SetFont(u8g2, u8g2_font_logisoso50_tn);
		int number_height = u8g2_GetAscent(u8g2) - u8g2_GetDescent(u8g2);
		int number_y = ((LCD_HEIGHT - number_height) / 2) + u8g2_GetAscent(u8g2);
		U8g2_DrawCenteredStr(u8g2, number_y, text);

		u8g2_DrawFrame(u8g2, 10, 10, 380, 280);
		u8g2_DrawHLine(u8g2, 18, 260, 364);

		u8g2_SetFont(u8g2, u8g2_font_6x13_tf);
		U8g2_DrawCenteredStr(u8g2, number_y + 26, "ST7305 refresh counter");
		snprintf(text, sizeof(text), "FPS:%lu.%02lu  frame:%lums  flush:%lums",
				 (unsigned long)(fps_x100 / 100),
				 (unsigned long)(fps_x100 % 100),
				 (unsigned long)(frame_us / 1000),
				 (unsigned long)(flush_us / 1000));
		u8g2_DrawStr(u8g2, 20, 282, text);

		const int64_t flush_start_us = esp_timer_get_time();
		u8g2_SendBuffer(u8g2);
		const int64_t now_us = esp_timer_get_time();

		flush_us = (uint32_t)(now_us - flush_start_us);
		frame_us = (uint32_t)(now_us - frame_start_us);
		counter++;
		frames++;

		if (now_us - last_report_us >= 1000000) {
			uint32_t delta_frames = frames - last_report_frames;
			fps_x100 = (uint32_t)((uint64_t)delta_frames * 100000000ULL / (uint64_t)(now_us - last_report_us));
			ESP_LOGI(TAG, "counter=%lu fps=%lu.%02lu frame=%lu us flush=%lu us",
					 (unsigned long)counter,
					 (unsigned long)(fps_x100 / 100),
					 (unsigned long)(fps_x100 % 100),
					 (unsigned long)frame_us,
					 (unsigned long)flush_us);
			last_report_us = now_us;
			last_report_frames = frames;
		}

		if ((frames % kWdtFeedFrameInterval) == 0) {
			vTaskDelay(1);
		} else {
			taskYIELD();
		}
	}
}

static void U8g2_CounterTask(void *arg)
{
	(void)arg;
	U8g2_RunCounterDemo();
	vTaskDelete(NULL);
}
#endif

extern "C" void app_main(void)
{
#if APP_USE_U8G2_DEMO
	BaseType_t ok = xTaskCreatePinnedToCore(U8g2_CounterTask, "u8g2_counter", 8192, NULL, 4, NULL, 1);
	configASSERT(ok == pdPASS);
#else
	UserApp_AppInit();
	RlcdPort.RLCD_Init();
	Lvgl_PortInit(400,300,Lvgl_FlushCallback);
	if(Lvgl_lock(-1)) {
		UserApp_UiInit();
  	  	Lvgl_unlock();
  	}
	UserApp_TaskInit();
#endif
}
