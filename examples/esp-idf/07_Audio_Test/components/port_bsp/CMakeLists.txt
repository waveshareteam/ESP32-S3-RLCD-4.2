idf_component_register(
    SRCS 
    "i2c_bsp.cpp" 
    "display_bsp.cpp" 
    "./src/multi_button/multi_button.c" 
    "button_bsp.c"
    "./codec_bsp.cpp"
    PRIV_REQUIRES 
    driver
    esp_timer
    codec_board 
    REQUIRES
    esp_driver_sdmmc
    fatfs
    INCLUDE_DIRS 
    "./" 
    "./src/multi_button"
    EMBED_FILES "./pcm/canon.pcm")
