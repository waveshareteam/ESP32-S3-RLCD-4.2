idf_component_register(
    SRCS 
    "adc_bsp.cpp"
    PRIV_REQUIRES 
    driver
    esp_adc
    INCLUDE_DIRS 
    "./")
