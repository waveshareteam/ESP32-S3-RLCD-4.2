#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include "user_app.h"
#include "i2c_equipment.h"
#include "i2c_bsp.h"

I2cMasterBus I2cbus(14,13,0);

void Rtc_LoopTask(void *arg) {
    for(;;) {
        rtcTimeStruct_t rtcData;
        Rtc_GetTime(&rtcData);
        ESP_LOGW("Rtc-Example","%d/%d/%d %02d:%02d:%02d",\
        rtcData.year,rtcData.month,rtcData.day,rtcData.hour,rtcData.minute,\
        rtcData.second);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void UserApp_AppInit(void) {
    printf("rtc-example run \n");
    Rtc_Setup(&I2cbus,0x51);
    Rtc_SetTime(2025,9,9,20,15,30);
    xTaskCreate(Rtc_LoopTask, "Rtc_LoopTask", 3000, NULL , 2, NULL);
}