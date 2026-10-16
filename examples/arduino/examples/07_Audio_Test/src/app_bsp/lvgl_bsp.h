#pragma once

#include "lvgl.h"

#define LVGL_TICK_PERIOD_MS    5
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 50

typedef void (*DispFlushCb)(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p);

void Lvgl_PortInit(int width, int height,DispFlushCb flush_cb);
bool Lvgl_lock(int timeout_ms);
void Lvgl_unlock(void);