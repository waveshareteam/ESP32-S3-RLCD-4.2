#include <WiFi.h>

const char* ssid = "K2P";          // Change to your WiFi name
const char* password = "1234567890";  // Change to your WiFi password
void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // Set to STA mode
  WiFi.begin(ssid, password);
  
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("IP Address:");
  Serial.println(WiFi.localIP().toString());
  //Serial.printf("IP Address: %s\n",WiFi.localIP().toString().c_str());
}
void loop()
{
  // Your code
}