
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>

#include "user_app.h"

extern "C" void app_main(void)
{
	UserApp_AppInit();
}
