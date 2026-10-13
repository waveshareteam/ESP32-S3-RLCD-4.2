#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include "user_app.h"
#include "i2c_equipment.h"
#include "i2c_bsp.h"

I2cMasterBus I2cbus(14,13,0);
Shtc3Port *shtc3port = NULL;


void Shtc3_LoopTask(void *arg) {
    for(;;) {
        float rh,temp;
        shtc3port->Shtc3_ReadTempHumi(&temp,&rh);
        ESP_LOGW("Shtc3-Example","RH:%.2f%%,Temp:%.2f°",rh,temp);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void UserApp_AppInit(void) {
    printf("shtc3-example run \n");
    shtc3port = new Shtc3Port(I2cbus);
    xTaskCreate(Shtc3_LoopTask, "Shtc3_LoopTask", 3000, NULL , 2, NULL);
}