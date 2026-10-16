# 02_WIFI_STA

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Connects to a Wi-Fi network as a station and prints the assigned IP address.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board

## How to Run

```bash
idf.py set-target esp32s3 build flash monitor
```

## Expected Result

After connecting, the serial monitor prints `IP: <address>`; it prints `disconnected` and retries if the connection drops.

## Troubleshooting

- The network name and password are fixed in the code (`K2P` / `1234567890`); change `.ssid` and `.password` in `components/esp_wifi_bsp/` to your network before building.
