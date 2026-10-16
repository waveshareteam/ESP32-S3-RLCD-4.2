# 07_Audio_Test

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Records from the microphones and plays audio through the ES8311 codec, controlled by the two buttons.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board
- Speaker

## How to Run

1. Install the `esp32` board package and make [`examples/arduino/libraries/`](../../libraries/)
   available as your sketchbook `libraries` folder.
2. Select **ESP32S3 Dev Module** and set Flash Size **16MB**, Partition Scheme
   **16M Flash (3MB APP/9.9MB FATFS)**, PSRAM **OPI PSRAM**, USB CDC On Boot **Enabled**
   ([screenshot](../../../../docs/images/arduino-tools-configuration.png)).
3. Open `07_Audio_Test.ino` and upload.

## Expected Result

The display shows the current state (`IDLE`, `Recording...`, `Playing...`). Double-click BOOT to record a short clip and click BOOT to play it back; double-click KEY to play the built-in music and click KEY to stop it.

## Troubleshooting

None yet.
