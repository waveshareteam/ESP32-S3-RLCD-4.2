#include <stdio.h>
#include <freertos/freeRTOS.h>
#include <esp_log.h>

#include "user_app.h"
#include "gui_guider.h"

static lv_ui init_ui;

void Lvgl_LoopTask(void *arg) {
    for(;;) {
        lv_obj_clear_flag(init_ui.screen_img_1,LV_OBJ_FLAG_HIDDEN); 
        lv_obj_add_flag(init_ui.screen_img_2, LV_OBJ_FLAG_HIDDEN);
        vTaskDelay(pdMS_TO_TICKS(1500));
        lv_obj_clear_flag(init_ui.screen_img_2,LV_OBJ_FLAG_HIDDEN); 
        lv_obj_add_flag(init_ui.screen_img_1, LV_OBJ_FLAG_HIDDEN);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}


void UserApp_AppInit() {
    
}

void UserApp_UiInit() {
    setup_ui(&init_ui);
}

void UserApp_TaskInit() {
    xTaskCreatePinnedToCore(Lvgl_LoopTask, "Lvgl_LoopTask", 4 * 1024, NULL, 2, NULL,1);
}
