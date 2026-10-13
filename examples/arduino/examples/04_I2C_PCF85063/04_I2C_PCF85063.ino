#include "i2c_equipment.h"
#include "i2c_bsp.h"

I2cMasterBus I2cbus(14, 13, 0);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("rtc-example run \n");
  Rtc_Setup(&I2cbus, 0x51);
  Rtc_SetTime(2025, 9, 9, 20, 15, 30);
}

void loop() {
  rtcTimeStruct_t rtcData;
  Rtc_GetTime(&rtcData);
  Serial.printf("%d/%d/%d %02d:%02d:%02d \n",
                rtcData.year, rtcData.month, rtcData.day, rtcData.hour, rtcData.minute,
                rtcData.second);
  delay(1000);
}
