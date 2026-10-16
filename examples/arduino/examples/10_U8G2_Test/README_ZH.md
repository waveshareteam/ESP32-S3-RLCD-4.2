# 10_U8G2_Test

[English](README.md) | [简体中文](README_ZH.md)

## 功能

用 U8g2 图形库在反射式 LCD 上绘制帧计数器。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

1. 安装 `esp32` 开发板包，并把 [`examples/arduino/libraries/`](../../libraries/) 作为项目文件夹下的
   `libraries` 目录使用。
2. 开发板选 **ESP32S3 Dev Module**，Flash Size 选 **16MB**，Partition Scheme 选
   **16M Flash (3MB APP/9.9MB FATFS)**，PSRAM 选 **OPI PSRAM**，USB CDC On Boot 选 **Enabled**
   （[截图](../../../../docs/images/arduino-tools-configuration.png)）。
3. 打开 `10_U8G2_Test.ino` 并上传。

## 预期现象

屏幕显示递增的计数，下方是 `ST7305 refresh counter`，底部是帧率；串口同步打印计时数据。

## 常见问题

暂无。
