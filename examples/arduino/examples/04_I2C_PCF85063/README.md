# 04_I2C_PCF85063

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Sets the PCF85063 real-time clock and prints the time once per second.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board

## How to Run

1. Install the `esp32` board package and make [`examples/arduino/libraries/`](../../libraries/)
   available as your sketchbook `libraries` folder.
2. Select **ESP32S3 Dev Module** and set Flash Size **16MB**, Partition Scheme
   **16M Flash (3MB APP/9.9MB FATFS)**, PSRAM **OPI PSRAM**, USB CDC On Boot **Enabled**
   ([screenshot](../../../../docs/images/arduino-tools-configuration.png)).
3. Open `04_I2C_PCF85063.ino` and upload.

## Expected Result

The serial monitor prints the date and time every second, starting from 2025/9/9 20:15:30. Output goes to the serial monitor only; the display is not used.

## Troubleshooting

- The sketch writes a fixed time to the RTC at every boot; change the `Rtc_SetTime()` call in `setup()` to start from the current time.
