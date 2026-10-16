# 10_U8G2_Test

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Draws a frame counter on the reflective LCD with the U8g2 graphics library.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board

## How to Run

1. Install the `esp32` board package and make [`examples/arduino/libraries/`](../../libraries/)
   available as your sketchbook `libraries` folder.
2. Select **ESP32S3 Dev Module** and set Flash Size **16MB**, Partition Scheme
   **16M Flash (3MB APP/9.9MB FATFS)**, PSRAM **OPI PSRAM**, USB CDC On Boot **Enabled**
   ([screenshot](../../../../docs/images/arduino-tools-configuration.png)).
3. Open `10_U8G2_Test.ino` and upload.

## Expected Result

The display shows an increasing counter with `ST7305 refresh counter` below it and the frame rate at the bottom; the serial monitor prints the same timing data.

## Troubleshooting

None yet.
