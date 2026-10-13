# ST7305 U8g2 计数器

[English](README.md)

适用于搭载 ST7305 300x400 RLCD 屏幕的 ESP32-S3 开发板的 Arduino 示例。

## 环境要求

- Arduino IDE 或 `arduino-cli`
- ESP32 Arduino core
- U8g2 库

## 默认引脚

- SCK：GPIO 11
- MOSI：GPIO 12
- DC：GPIO 5
- CS：GPIO 40
- RST：GPIO 41

## 运行效果

- 屏幕中央显示帧计数
- 数字下方显示 `ST7305 refresh counter`
- 底部显示 FPS、帧时间和刷新时间
- 串口日志波特率为 115200

在 Arduino IDE 中打开 `10_U8G2_Test.ino`，上传到 ESP32-S3 开发板即可。
