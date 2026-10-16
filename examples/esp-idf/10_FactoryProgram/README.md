# 10_FactoryProgram

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

The board test screen: time, temperature and humidity, battery level, TF card check, Wi-Fi and Bluetooth scans, and the audio test.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board
- TF card (optional)
- Speaker (optional)

## How to Run

```bash
idf.py set-target esp32s3 build flash monitor
```

## Expected Result

The display shows the clock, temperature, humidity and battery level, the TF card result (`passed`, `failed` or `No Card`) and the number of Wi-Fi access points and Bluetooth devices found. The buttons run the record and playback test of 07_Audio_Test.

## Troubleshooting

None yet.
