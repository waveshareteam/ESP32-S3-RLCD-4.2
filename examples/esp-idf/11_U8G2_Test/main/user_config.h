#ifndef USER_CONFIG_H
#define USER_CONFIG_H


/*lcd init*/
#define LCD_WIDTH      400    //需要注意的是,竖屏和横屏显示的时候,分辨率不一样的
#define LCD_HEIGHT     300    //需要注意的是,竖屏和横屏显示的时候,分辨率不一样的

#define RLCD_DC_PIN    GPIO_NUM_5  
#define RLCD_CS_PIN    GPIO_NUM_40
#define RLCD_SCK_PIN   GPIO_NUM_11
#define RLCD_MOSI_PIN  GPIO_NUM_12
#define RLCD_RST_PIN   GPIO_NUM_41
#define RLCD_TE_PIN    GPIO_NUM_6

/* 1: run the U8g2 ST7305 demo, 0: run the original LVGL demo */
#define APP_USE_U8G2_DEMO 1

/*i2c*/
#define ESP32_I2C_SDA_PIN   GPIO_NUM_13
#define ESP32_I2C_SCL_PIN   GPIO_NUM_14

#endif
