# 02_WIFI_STA

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Connects to a Wi-Fi network as a station and prints the assigned IP address.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board

## How to Run

1. Install the `esp32` board package and make [`examples/arduino/libraries/`](../../libraries/)
   available as your sketchbook `libraries` folder.
2. Select **ESP32S3 Dev Module** and set Flash Size **16MB**, Partition Scheme
   **16M Flash (3MB APP/9.9MB FATFS)**, PSRAM **OPI PSRAM**, USB CDC On Boot **Enabled**
   ([screenshot](../../../../docs/images/arduino-tools-configuration.png)).
3. Open `02_WIFI_STA.ino` and upload.

## Expected Result

The serial monitor prints dots while connecting, then the IP address.

## Troubleshooting

- The sketch waits forever until it connects; set `ssid` and `password` at the top of the sketch to your network (the defaults are `K2P` / `1234567890`).
