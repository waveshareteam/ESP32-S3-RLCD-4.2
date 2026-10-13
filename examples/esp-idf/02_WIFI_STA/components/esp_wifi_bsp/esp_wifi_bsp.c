#include "esp_wifi_bsp.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <stdio.h>

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

void espwifi_Init(void) {
    nvs_flash_init();                                    // Initialize default NVS storage
    esp_netif_init();                                    // Initialize TCP/IP stack
    esp_event_loop_create_default();                     // Create default event loop
    esp_netif_create_default_wifi_sta();                 // Attach TCP/IP stack to default event loop
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Default config
    esp_wifi_init(&cfg);                                 // Initialize Wi-Fi
    esp_event_handler_instance_t Instance_WIFI_IP;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &Instance_WIFI_IP);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &Instance_WIFI_IP);
    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid     = "K2P",
                .password = "1234567890",
            },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);               // Set mode to STA
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); // Configure Wi-Fi
    esp_wifi_start();                               // Start Wi-Fi
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        char               ip[25];
        uint32_t           pxip = event->ip_info.ip.addr;
        sprintf(ip, "%d.%d.%d.%d", (uint8_t) (pxip), (uint8_t) (pxip >> 8), (uint8_t) (pxip >> 16), (uint8_t) (pxip >> 24));
        printf("IP: %s\n", ip);
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("disconnected\n");
    }
}
