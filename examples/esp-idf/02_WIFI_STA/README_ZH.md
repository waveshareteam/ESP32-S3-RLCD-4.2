# 02_WIFI_STA

[English](README.md) | [简体中文](README_ZH.md)

## 功能

以 Station 模式连接 Wi-Fi 网络，并打印获得的 IP 地址。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

```bash
idf.py set-target esp32s3 build flash monitor
```

## 预期现象

连接成功后串口打印 `IP: <地址>`；断开时打印 `disconnected` 并重新连接。

## 常见问题

- 网络名和密码写死在代码里（`K2P` / `1234567890`）；编译前把 `components/esp_wifi_bsp/` 中的 `.ssid` 和 `.password` 改成自己的网络。
