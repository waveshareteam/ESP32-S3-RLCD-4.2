# 01_WIFI_AP

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Starts a Wi-Fi access point and logs every station that joins.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board

## How to Run

```bash
idf.py set-target esp32s3 build flash monitor
```

## Expected Result

A network named `waveshare_esp32` (password `wav123456`) appears. When a phone or computer joins, the serial monitor prints its MAC and IP address.

## Troubleshooting

- Change the network name and password with `EXAMPLE_ESP_WIFI_SSID` and `EXAMPLE_ESP_WIFI_PASS` in `main/softap_example_main.c`.
