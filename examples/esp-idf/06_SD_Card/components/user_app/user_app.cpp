#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include "user_app.h"
#include "sdcard_bsp.h"

#define sdcard_write_Test


CustomSDPort *sdcardPort = NULL;


void Fatfs_LoopTask(void *arg)
{
  	uint32_t value = 1;
  	char test[45] = {""};
  	char rtest[45] = {""};
  	for(;;)
  	{
#ifdef sdcard_write_Test
  		snprintf(test,45,"sdcard_writeTest : %ld\n",value);
  		sdcardPort->SDPort_WriteFile("/sdcard/writeTest.txt",test,strlen(test));
  		vTaskDelay(pdMS_TO_TICKS(500));
  		sdcardPort->SDPort_ReadFile("/sdcard/writeTest.txt",(uint8_t *)rtest,NULL);
  		printf("rtest:%s\n",rtest);
  		vTaskDelay(pdMS_TO_TICKS(500));
  		value++;
#else
  		vTaskDelay(pdMS_TO_TICKS(500));  
#endif
  	}
}

void UserApp_AppInit() {
    sdcardPort = new CustomSDPort("/sdcard");
    xTaskCreatePinnedToCore(Fatfs_LoopTask, "Fatfs_LoopTask", 5 * 1024, NULL , 2, NULL,0); //sd card test
}