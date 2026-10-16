# 01_WIFI_AP

[English](README.md) | [简体中文](README_ZH.md)

## 功能

开启 Wi-Fi 热点，并打印每个接入设备的信息。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

1. 安装 `esp32` 开发板包，并把 [`examples/arduino/libraries/`](../../libraries/) 作为项目文件夹下的
   `libraries` 目录使用。
2. 开发板选 **ESP32S3 Dev Module**，Flash Size 选 **16MB**，Partition Scheme 选
   **16M Flash (3MB APP/9.9MB FATFS)**，PSRAM 选 **OPI PSRAM**，USB CDC On Boot 选 **Enabled**
   （[截图](../../../../docs/images/arduino-tools-configuration.png)）。
3. 打开 `01_WIFI_AP.ino` 并上传。

## 预期现象

出现名为 `ESP32_AP` 的网络（密码 `12345678`）；串口打印热点 IP，并每 5 秒打印一次已连接设备数。

## 常见问题

- 网络名和密码在草图开头的 `ssid`、`password` 中修改。
