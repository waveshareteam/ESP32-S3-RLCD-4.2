# 04_I2C_PCF85063

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Sets the PCF85063 real-time clock and prints the time once per second.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board

## How to Run

```bash
idf.py set-target esp32s3 build flash monitor
```

## Expected Result

The serial monitor prints the date and time every second, starting from 2025/9/9 20:15:30. Output goes to the serial monitor only; the display is not used.

## Troubleshooting

- The example writes a fixed time to the RTC at every boot; change the `Rtc_SetTime()` call in `components/user_app/` to start from the current time.
