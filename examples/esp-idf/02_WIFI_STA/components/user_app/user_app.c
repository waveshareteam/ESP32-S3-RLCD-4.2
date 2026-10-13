#include <stdio.h>
#include "user_app.h"

#include "esp_wifi_bsp.h"

void user_top_init(void)
{
    printf("wifi-example run \n");
    espwifi_Init();
}




