# 01_WIFI_AP

[English](README.md) | [简体中文](README_ZH.md)

## 功能

开启 Wi-Fi 热点，并打印每个接入设备的信息。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

```bash
idf.py set-target esp32s3 build flash monitor
```

## 预期现象

出现名为 `waveshare_esp32` 的网络（密码 `wav123456`）。手机或电脑连上后，串口打印它的 MAC 和 IP 地址。

## 常见问题

- 网络名和密码在 `main/softap_example_main.c` 的 `EXAMPLE_ESP_WIFI_SSID`、`EXAMPLE_ESP_WIFI_PASS` 中修改。
