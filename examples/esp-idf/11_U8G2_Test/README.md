# 11_U8G2_Test

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Draws a frame counter on the reflective LCD with the U8g2 graphics library.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board

## How to Run

```bash
idf.py set-target esp32s3 build flash monitor
```

## Expected Result

The display shows an increasing counter with `ST7305 refresh counter` below it and the frame rate at the bottom; the serial monitor prints the same timing data.

## Troubleshooting

None yet.
