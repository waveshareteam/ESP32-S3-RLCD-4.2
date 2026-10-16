# 06_SD_Card

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Writes a counter to `writeTest.txt` on the TF card and reads it back.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board
- TF card

## How to Run

1. Install the `esp32` board package and make [`examples/arduino/libraries/`](../../libraries/)
   available as your sketchbook `libraries` folder.
2. Select **ESP32S3 Dev Module** and set Flash Size **16MB**, Partition Scheme
   **16M Flash (3MB APP/9.9MB FATFS)**, PSRAM **OPI PSRAM**, USB CDC On Boot **Enabled**
   ([screenshot](../../../../docs/images/arduino-tools-configuration.png)).
3. Open `06_SD_Card.ino` and upload.

## Expected Result

The serial monitor prints `read data:sdcard_writeTest : <n>` with an increasing number. Output goes to the serial monitor only; the display is not used.

## Troubleshooting

- `SD card not initialized`: insert a FAT-formatted card before resetting the board.
