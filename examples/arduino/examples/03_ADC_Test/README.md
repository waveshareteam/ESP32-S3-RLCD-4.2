# 03_ADC_Test

[English](README.md) | [简体中文](README_ZH.md)

## What It Does

Reads the battery voltage through the ADC once per second.

## Hardware Required

- ESP32-S3-RLCD-4.2 development board
- 18650 battery (optional)

## How to Run

1. Install the `esp32` board package and make [`examples/arduino/libraries/`](../../libraries/)
   available as your sketchbook `libraries` folder.
2. Select **ESP32S3 Dev Module** and set Flash Size **16MB**, Partition Scheme
   **16M Flash (3MB APP/9.9MB FATFS)**, PSRAM **OPI PSRAM**, USB CDC On Boot **Enabled**
   ([screenshot](../../../../docs/images/arduino-tools-configuration.png)).
3. Open `03_ADC_Test.ino` and upload.

## Expected Result

The serial monitor prints the raw ADC value and the battery voltage every second. Output goes to the serial monitor only; the display is not used.

## Troubleshooting

None yet.
