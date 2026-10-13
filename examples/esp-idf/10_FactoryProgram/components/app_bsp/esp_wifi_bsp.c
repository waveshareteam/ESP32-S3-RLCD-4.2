#include <stdio.h>
#include "esp_wifi_bsp.h"
#include "esp_wifi.h"  
#include "esp_event.h" 
#include "nvs_flash.h" 
#include "esp_log.h"


#include "string.h" 

EventGroupHandle_t wifi_even_ = NULL;

esp_bsp_t user_esp_bsp;

static esp_netif_t *net = NULL;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void example_scan_wifi_task(void *arg);
void espwifi_init(void)
{
    memset(&user_esp_bsp,0,sizeof(esp_bsp_t));
    wifi_even_ = xEventGroupCreate();
    nvs_flash_init();                           // Initialize default NVS storage
    esp_netif_init();                           // Initialize TCP/IP stack
    esp_event_loop_create_default();            // Create default event loop
    net = esp_netif_create_default_wifi_sta();  // Add TCP/IP stack to the default event loop
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Default configuration
    esp_wifi_init(&cfg);                                 // Initialize WiFi
    esp_event_handler_instance_t Instance_WIFI_IP;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &Instance_WIFI_IP);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &Instance_WIFI_IP);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "PDCN",
            .password = "1234567890",
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);               // Set mode to STA
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); // Configure WiFi
    esp_wifi_start();                               // Start WiFi
    xTaskCreatePinnedToCore(example_scan_wifi_task, "example_scan_wifi_task", 3000, NULL, 2, NULL,0);   
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        xEventGroupSetBits(wifi_even_,0x01);
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip[25];
        uint32_t pxip = event->ip_info.ip.addr;
        sprintf(ip, "%d.%d.%d.%d", (uint8_t)(pxip), (uint8_t)(pxip >> 8), (uint8_t)(pxip >> 16), (uint8_t)(pxip >> 24));
        ESP_LOGI("wifiSta","%s",ip);
    }
    else if(event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGD("wifiSta","Disconnected");
    }
}

void espwifi_deinit(void)
{
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_netif_destroy_default_wifi(net);
    esp_event_loop_delete_default();
    //esp_netif_deinit();
    //nvs_flash_deinit();
}



static void example_scan_wifi_task(void *arg)
{
    uint16_t rec = 0;
    EventBits_t even = xEventGroupWaitBits(wifi_even_,0x01,pdTRUE,pdTRUE,pdMS_TO_TICKS(15000));
    if( even & 0x01 ) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_scan_start(NULL,true));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_scan_get_ap_num(&rec));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_clear_ap_list());
    }
    user_esp_bsp.apNum = rec;
    xEventGroupSetBits(wifi_even_,0x02);
    vTaskDelete(NULL);
}

