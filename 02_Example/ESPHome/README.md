# ESPHome

[中文](README_ZH.md)

[ESPHome](https://esphome.io/) lets you configure ESP32 devices for [Home Assistant](https://www.home-assistant.io/) using simple YAML — no C/C++ required.

## Examples

See [`examples/`](examples/) for ready-to-flash YAML configurations:

| Example | Description |
| --- | --- |
| [sensor](examples/esp32-s3-rlcd-42-sensor.yaml) | Display + SHTC3 temp & humidity + battery monitor + buttons |
| [speaker](examples/esp32-s3-rlcd-42-speaker.yaml) | Sensor dashboard + ES8311 speaker RTTTL ringtone test |
| [wake-word](examples/esp32-s3-rlcd-42-wake-word.yaml) | On-device "Hey Jarvis" wake word detection with counter display |

## Quick Start

1. Install [Home Assistant](https://www.home-assistant.io/installation/) and add the [ESPHome add-on](https://esphome.io/guides/getting_started_hassio.html).
2. Create a new device in ESPHome, click **EDIT**, and paste the example YAML.
3. Define Wi-Fi credentials in `secrets.yaml`:
   ```yaml
   wifi_ssid: "YourSSID"
   wifi_password: "YourPassword"
   ```
4. **SAVE → INSTALL**. First flash via USB; subsequent updates via OTA.

## Tutorials

Step-by-step guides covering Home Assistant installation, ESPHome setup, and device configuration:

- [English tutorial](https://docs.waveshare.com/ESP32-ESPHome-Tutorials)
- [中文教程](https://docs.waveshare.net/ESP32-ESPHome-Tutorials)

## GPIO Pinout

|  GPIO  |           Function           |
| :----: | :--------------------------: |
| GPIO0  | BOOT button (active low)     |
| GPIO4  | Battery ADC                  |
| GPIO5  | Display DC                   |
| GPIO8  | I²S DOUT (speaker)           |
| GPIO9  | I²S BCLK                    |
| GPIO10 | I²S DIN (microphone)         |
| GPIO11 | SPI CLK (display)            |
| GPIO12 | SPI MOSI (display)           |
| GPIO13 | I²C SDA                     |
| GPIO14 | I²C SCL                     |
| GPIO16 | I²S MCLK                    |
| GPIO18 | KEY button (active low)      |
| GPIO40 | Display CS                   |
| GPIO41 | Display RESET                |
| GPIO45 | I²S LRCLK                   |
| GPIO46 | Speaker amplifier enable     |

## References

- [ESPHome documentation](https://esphome.io/)
- [ESPHome ST7305 external component](https://github.com/kylehase/ESPHome-ST7305-RLCD)
- [ESP32-S3-RLCD-4.2 product page](https://docs.waveshare.com/ESP32-S3-RLCD-4.2)
- [devices.esphome.io](https://devices.esphome.io/)
