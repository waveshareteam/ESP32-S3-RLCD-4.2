# 09_LVGL_V9_Test

[English](README.md) | [简体中文](README_ZH.md)

## 功能

在反射式 LCD 上显示 LVGL 9 界面。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

1. 安装 `esp32` 开发板包，并把 [`examples/arduino/libraries/`](../../libraries/) 作为项目文件夹下的
   `libraries` 目录使用。
2. 开发板选 **ESP32S3 Dev Module**，Flash Size 选 **16MB**，Partition Scheme 选
   **16M Flash (3MB APP/9.9MB FATFS)**，PSRAM 选 **OPI PSRAM**，USB CDC On Boot 选 **Enabled**
   （[截图](../../../../docs/images/arduino-tools-configuration.png)）。
3. 打开 `09_LVGL_V9_Test.ino` 并上传。

## 预期现象

屏幕每 1.5 秒在两张图片之间切换。

## 常见问题

暂无。
