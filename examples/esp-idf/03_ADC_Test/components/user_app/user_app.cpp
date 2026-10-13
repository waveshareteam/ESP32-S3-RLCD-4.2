#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include "adc_bsp.h"

void Adc_LoopTask(void *arg) {
    for(;;) {
        int data;
        float vol = Adc_GetBatteryVoltage(&data);
  	  	ESP_LOGW("Adc-Example","Adc Value:%d,Batt Voltage:%f\n",data,vol);
  	  	vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void UserApp_AppInit() {
    printf("adc-example run \n");
    Adc_PortInit();
    xTaskCreate(Adc_LoopTask, "Adc_LoopTask", 3000, NULL , 2, NULL);
}