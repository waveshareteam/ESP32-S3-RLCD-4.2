# ST7305 U8g2 Counter

[中文](README_ZH.md)

Arduino sketch for an ESP32-S3 board with an ST7305 300x400 RLCD panel.

## Requirements

- Arduino IDE or `arduino-cli`
- ESP32 Arduino core
- U8g2 library

## Default Pins

- SCK: GPIO 11
- MOSI: GPIO 12
- DC: GPIO 5
- CS: GPIO 40
- RST: GPIO 41

## What It Shows

- A centered frame counter
- `ST7305 refresh counter` below the number
- FPS, frame time, and flush time at the bottom
- Serial log at 115200 baud

Open `10_U8G2_Test.ino` in Arduino IDE and upload to the ESP32-S3 board.
