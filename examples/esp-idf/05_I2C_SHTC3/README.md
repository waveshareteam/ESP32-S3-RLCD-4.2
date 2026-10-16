# 05_I2C_SHTC3

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Reads temperature and relative humidity from the SHTC3 once per second.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board

## How to Run

```bash
idf.py set-target esp32s3 build flash monitor
```

## Expected Result

The serial monitor prints the relative humidity and temperature every second. Output goes to the serial monitor only; the display is not used.

## Troubleshooting

None yet.
