#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_netif.h"

const char *ssid = "ESP32_AP";
const char *password = "12345678";

void setup() {
    Serial.begin(115200);
    
    // Set the ESP32 to AP mode
    WiFi.softAP(ssid, password);
    Serial.println("WiFi AP Started");
    
    // Print the IP address of AP
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
}

void loop() {
    delay(5000); // Check the connected devices every 5 seconds
    printConnectedDevices();
}

void printConnectedDevices() {
    wifi_sta_list_t stationList;
    esp_netif_ip_info_t ip_info;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    // Obtain the list of connected devices
    esp_wifi_ap_get_sta_list(&stationList);
    
    Serial.print("Connected Devices: ");
    Serial.println(stationList.num);

    if (netif == nullptr) {
        Serial.println("Failed to get AP interface!");
        return;
    }

    for (int i = 0; i < stationList.num; i++) {
        Serial.printf("Device %d MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", i + 1,
                      stationList.sta[i].mac[0], stationList.sta[i].mac[1], stationList.sta[i].mac[2],
                      stationList.sta[i].mac[3], stationList.sta[i].mac[4], stationList.sta[i].mac[5]);
        
        // Obtain the IP address of the device (ESP32 cannot directly obtain the IP address of the client when acting as an AP)
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            Serial.printf("Device %d IP: %s\n", i + 1, ip4addr_ntoa((const ip4_addr_t*)&ip_info.ip));
        }
    }
}
