
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_timer.h>
#include <esp_log.h>

#include "display_bsp.h"
#include "lvgl_bsp.h"
#include "user_app.h"

DisplayPort RlcdPort(12,11,5,40,41,400,300);

static void Lvgl_FlushCallback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
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

extern "C" void app_main(void)
{
	UserApp_AppInit();
	RlcdPort.RLCD_Init();
	Lvgl_PortInit(400,300,Lvgl_FlushCallback);
	if(Lvgl_lock(-1)) {
		UserApp_UiInit();
  	  	Lvgl_unlock();
  	}
	UserApp_TaskInit();
}
