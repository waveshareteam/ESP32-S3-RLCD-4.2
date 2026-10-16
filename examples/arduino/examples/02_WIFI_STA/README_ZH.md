# 02_WIFI_STA

[English](README.md) | [简体中文](README_ZH.md)

## 功能

以 Station 模式连接 Wi-Fi 网络，并打印获得的 IP 地址。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

1. 安装 `esp32` 开发板包，并把 [`examples/arduino/libraries/`](../../libraries/) 作为项目文件夹下的
   `libraries` 目录使用。
2. 开发板选 **ESP32S3 Dev Module**，Flash Size 选 **16MB**，Partition Scheme 选
   **16M Flash (3MB APP/9.9MB FATFS)**，PSRAM 选 **OPI PSRAM**，USB CDC On Boot 选 **Enabled**
   （[截图](../../../../docs/images/arduino-tools-configuration.png)）。
3. 打开 `02_WIFI_STA.ino` 并上传。

## 预期现象

连接过程中串口打印点号，连上后打印 IP 地址。

## 常见问题

- 草图会一直等到连上为止；先把草图开头的 `ssid` 和 `password` 改成自己的网络（默认是 `K2P` / `1234567890`）。
