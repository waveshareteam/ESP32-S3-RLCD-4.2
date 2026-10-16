# 04_I2C_PCF85063

[English](README.md) | [简体中文](README_ZH.md)

## 功能

设置 PCF85063 实时时钟，每秒打印一次时间。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

1. 安装 `esp32` 开发板包，并把 [`examples/arduino/libraries/`](../../libraries/) 作为项目文件夹下的
   `libraries` 目录使用。
2. 开发板选 **ESP32S3 Dev Module**，Flash Size 选 **16MB**，Partition Scheme 选
   **16M Flash (3MB APP/9.9MB FATFS)**，PSRAM 选 **OPI PSRAM**，USB CDC On Boot 选 **Enabled**
   （[截图](../../../../docs/images/arduino-tools-configuration.png)）。
3. 打开 `04_I2C_PCF85063.ino` 并上传。

## 预期现象

串口每秒打印日期和时间，从 2025/9/9 20:15:30 开始走时。结果只输出到串口，屏幕不使用。

## 常见问题

- 草图每次启动都把固定时间写入 RTC；想从当前时间开始，修改 `setup()` 中的 `Rtc_SetTime()` 调用。
