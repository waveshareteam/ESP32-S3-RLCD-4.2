# 07_Audio_Test

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Records from the microphones and plays audio through the ES8311 codec, controlled by the two buttons.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board
- Speaker

## How to Run

```bash
idf.py set-target esp32s3 build flash monitor
```

## Expected Result

The display shows the current state (`IDLE`, `Recording...`, `Playing...`). Double-click BOOT to record a short clip and click BOOT to play it back; double-click KEY to play the built-in music and click KEY to stop it.

## Troubleshooting

None yet.
