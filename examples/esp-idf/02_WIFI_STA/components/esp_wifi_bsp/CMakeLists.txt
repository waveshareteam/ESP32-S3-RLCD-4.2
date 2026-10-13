idf_component_register(
  SRCS "esp_wifi_bsp.c"

  PRIV_REQUIRES
  esp_event
  nvs_flash
  REQUIRES
  esp_wifi
  INCLUDE_DIRS "./")
