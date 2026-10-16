# 07_Audio_Test

[English](README.md) | [简体中文](README_ZH.md)

## 功能

通过两个按键控制麦克风录音，并经 ES8311 播放。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板
- 扬声器

## 怎么运行

1. 安装 `esp32` 开发板包，并把 [`examples/arduino/libraries/`](../../libraries/) 作为项目文件夹下的
   `libraries` 目录使用。
2. 开发板选 **ESP32S3 Dev Module**，Flash Size 选 **16MB**，Partition Scheme 选
   **16M Flash (3MB APP/9.9MB FATFS)**，PSRAM 选 **OPI PSRAM**，USB CDC On Boot 选 **Enabled**
   （[截图](../../../../docs/images/arduino-tools-configuration.png)）。
3. 打开 `07_Audio_Test.ino` 并上传。

## 预期现象

屏幕显示当前状态（`IDLE`、`Recording...`、`Playing...`）。双击 BOOT 录一段音，单击 BOOT 回放；双击 KEY 播放内置音乐，单击 KEY 停止。

## 常见问题

暂无。
