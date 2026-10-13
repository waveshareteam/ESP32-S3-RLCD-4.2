#include "i2c_equipment.h"
#include "i2c_bsp.h"

I2cMasterBus I2cbus(14,13,0);
Shtc3Port *shtc3port = NULL;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("shtc3-example run \n");
  shtc3port = new Shtc3Port(I2cbus);
}

void loop() {
  float rh,temp;
  shtc3port->Shtc3_ReadTempHumi(&temp,&rh);
  Serial.printf("RH:%.2f%%,Temp:%.2f° \n",rh,temp);
  delay(1000);
}
