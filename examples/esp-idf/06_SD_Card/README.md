# 06_SD_Card

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Writes a counter to `writeTest.txt` on the TF card and reads it back.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board
- TF card

## How to Run

```bash
idf.py set-target esp32s3 build flash monitor
```

## Expected Result

The serial monitor prints `rtest:sdcard_writeTest : <n>` about once per second with an increasing number. Output goes to the serial monitor only; the display is not used.

## Troubleshooting

- `SD card not initialized`: insert a FAT-formatted card before resetting the board.
