# 05_I2C_SHTC3

[English](README.md) | [简体中文](README_ZH.md)

## 功能

每秒读取一次 SHTC3 的温度和相对湿度。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

1. 安装 `esp32` 开发板包，并把 [`examples/arduino/libraries/`](../../libraries/) 作为项目文件夹下的
   `libraries` 目录使用。
2. 开发板选 **ESP32S3 Dev Module**，Flash Size 选 **16MB**，Partition Scheme 选
   **16M Flash (3MB APP/9.9MB FATFS)**，PSRAM 选 **OPI PSRAM**，USB CDC On Boot 选 **Enabled**
   （[截图](../../../../docs/images/arduino-tools-configuration.png)）。
3. 打开 `05_I2C_SHTC3.ino` 并上传。

## 预期现象

串口每秒打印相对湿度和温度。结果只输出到串口，屏幕不使用。

## 常见问题

暂无。
