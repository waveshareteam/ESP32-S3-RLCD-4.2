#include "sdcard_bsp.h"

#define sdcard_write_Test

CustomSDPort *sdcardPort = NULL;

void setup()
{
  Serial.begin(115200);
  delay(2000);
  sdcardPort = new CustomSDPort("/sdcard");
}

uint32_t value = 1;
char sdcard_read[45] = {""};
char sdcard_write[45] = {""};

void loop()
{
#ifdef sdcard_write_Test
  snprintf(sdcard_write,45,"sdcard_writeTest : %ld \n",value);
  sdcardPort->SDPort_WriteFile("/sdcard/writeTest.txt",sdcard_write,strlen(sdcard_write));
  vTaskDelay(pdMS_TO_TICKS(500));
  sdcardPort->SDPort_ReadFile("/sdcard/writeTest.txt",(uint8_t *)sdcard_read,NULL);
  Serial.printf("read data:%s\n",sdcard_read);
  vTaskDelay(pdMS_TO_TICKS(500));
  value++;
#endif
}
