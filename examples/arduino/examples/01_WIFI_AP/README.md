# 01_WIFI_AP

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Starts a Wi-Fi access point and logs every station that joins.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board

## How to Run

1. Install the `esp32` board package and make [`examples/arduino/libraries/`](../../libraries/)
   available as your sketchbook `libraries` folder.
2. Select **ESP32S3 Dev Module** and set Flash Size **16MB**, Partition Scheme
   **16M Flash (3MB APP/9.9MB FATFS)**, PSRAM **OPI PSRAM**, USB CDC On Boot **Enabled**
   ([screenshot](../../../../docs/images/arduino-tools-configuration.png)).
3. Open `01_WIFI_AP.ino` and upload.

## Expected Result

A network named `ESP32_AP` (password `12345678`) appears; the serial monitor prints the access point IP and, every 5 seconds, the number of connected devices.

## Troubleshooting

- Change the network name and password with `ssid` and `password` at the top of the sketch.
