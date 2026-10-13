#include <stdio.h>
#include <freertos/freeRTOS.h>
#include <esp_log.h>

#include "adc_bsp.h"

void Adc_LoopTask(void *arg) {
  for (;;) {
    int data;
    float vol = Adc_GetBatteryVoltage(&data);
    Serial.printf("adc value:%d,batt voltage:%.2fV \n",data, vol);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void UserApp_AppInit() {
  Serial.printf("adc-example run \n");
  Adc_PortInit();
  xTaskCreate(Adc_LoopTask, "Adc_LoopTask", 3000, NULL, 2, NULL);
}
