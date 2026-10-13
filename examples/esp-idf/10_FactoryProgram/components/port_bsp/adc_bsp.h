#pragma once

#include <esp_adc/adc_oneshot.h>

void Adc_PortInit();
float Adc_GetBatteryVoltage();
uint8_t Adc_GetBatteryLevel();