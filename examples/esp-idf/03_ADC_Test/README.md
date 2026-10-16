# 03_ADC_Test

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Reads the battery voltage through the ADC once per second.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board
- 18650 battery (optional)

## How to Run

```bash
idf.py set-target esp32s3 build flash monitor
```

## Expected Result

The serial monitor prints the raw ADC value and the battery voltage every second. Output goes to the serial monitor only; the display is not used.

## Troubleshooting

None yet.
