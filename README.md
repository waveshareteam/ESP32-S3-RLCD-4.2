<div align="center">
  <h1>ESP32-S3-RLCD-4.2</h1>
  <p><strong>ESP32-S3 4.2-inch 300 x 400 reflective LCD AIoT development board</strong></p>
  <p><strong>English</strong> | <a href="README_ZH.md">简体中文</a></p>
  <p>
    <a href="https://github.com/waveshareteam/ESP32-S3-RLCD-4.2/actions/workflows/build.yml"><img alt="Build" src="https://github.com/waveshareteam/ESP32-S3-RLCD-4.2/actions/workflows/build.yml/badge.svg"></a>
    <a href="https://github.com/waveshareteam/ESP32-S3-RLCD-4.2/releases/latest"><img alt="Latest Release" src="https://img.shields.io/github/v/release/waveshareteam/ESP32-S3-RLCD-4.2"></a>
    <a href="LICENSE"><img alt="License" src="https://img.shields.io/github/license/waveshareteam/ESP32-S3-RLCD-4.2"></a>
  </p>
  <p>
    <a href="https://www.waveshare.com/esp32-s3-rlcd-4.2.htm">🌐 Product</a> &middot;
    <a href="https://github.com/waveshareteam/ESP32-S3-RLCD-4.2/releases/latest">📦 Firmware</a> &middot;
    <a href="examples/esp-idf/">🧩 ESP-IDF</a> &middot;
    <a href="examples/arduino/examples/">🔧 Arduino</a> &middot;
    <a href="https://docs.waveshare.com/ESP32-S3-RLCD-4.2">📚 Documentation</a>
  </p>
  <p><img src="docs/images/esp32-s3-rlcd-4.2.webp" alt="ESP32-S3-RLCD-4.2 development board" width="500"></p>
</div>

---

## ✨ Overview

The ESP32-S3-RLCD-4.2 is an AIoT development board built around the ESP32-S3-WROOM-1-N16R8 module (16 MB flash,
8 MB PSRAM) with a 4.2-inch reflective LCD that needs no backlight. This repository provides its
ESP-IDF and Arduino examples and ready-to-flash example firmware.

| Feature | Device / interface |
| --- | --- |
| MCU | ESP32-S3-WROOM-1-N16R8, 2.4 GHz Wi-Fi and Bluetooth 5 LE |
| Memory | 16 MB Flash, 8 MB PSRAM |
| Display | 4.2-inch 300 x 400 reflective LCD, ST7305 controller |
| Audio | ES8311 codec; ES7210 ADC with a dual-microphone array |
| RTC | PCF85063 |
| Sensor | SHTC3 temperature and humidity sensor |
| Storage | TF card slot |
| Power | USB Type-C and an 18650 battery holder |

## 📦 Quick Start

The fastest way to try an example is a ready-to-flash package from the
[latest release](https://github.com/waveshareteam/ESP32-S3-RLCD-4.2/releases/latest).

1. Download the `*-combined.zip` package for the example and framework version you need.
2. Extract the archive and install esptool with `python -m pip install esptool`.
3. Connect the board over USB.
4. Run `flash_combined.bat COMx` on Windows or `./flash_combined.sh /dev/ttyACM0` on Linux.
5. Reset the board if it does not restart automatically.

Factory firmware (restores the out-of-box demo): download the file whose name contains `FactoryOnly` from the [Releases page](https://github.com/waveshareteam/ESP32-S3-RLCD-4.2/releases).

## 🧪 Examples

### ESP-IDF

| Example | Main feature |
| --- | --- |
| [01_WIFI_AP](examples/esp-idf/01_WIFI_AP/) | Wi-Fi access point |
| [02_WIFI_STA](examples/esp-idf/02_WIFI_STA/) | Wi-Fi station |
| [03_ADC_Test](examples/esp-idf/03_ADC_Test/) | Battery voltage |
| [04_I2C_PCF85063](examples/esp-idf/04_I2C_PCF85063/) | RTC |
| [05_I2C_SHTC3](examples/esp-idf/05_I2C_SHTC3/) | Temperature and humidity |
| [06_SD_Card](examples/esp-idf/06_SD_Card/) | TF card |
| [07_Audio_Test](examples/esp-idf/07_Audio_Test/) | Audio |
| [08_LVGL_V8_Test](examples/esp-idf/08_LVGL_V8_Test/) | LVGL 8 |
| [09_LVGL_V9_Test](examples/esp-idf/09_LVGL_V9_Test/) | LVGL 9 |
| [10_FactoryProgram](examples/esp-idf/10_FactoryProgram/) | Factory test program |
| [11_U8G2_Test](examples/esp-idf/11_U8G2_Test/) | U8g2 |

### Arduino

| Example | Main feature |
| --- | --- |
| [01_WIFI_AP](examples/arduino/examples/01_WIFI_AP/) | Wi-Fi access point |
| [02_WIFI_STA](examples/arduino/examples/02_WIFI_STA/) | Wi-Fi station |
| [03_ADC_Test](examples/arduino/examples/03_ADC_Test/) | Battery voltage |
| [04_I2C_PCF85063](examples/arduino/examples/04_I2C_PCF85063/) | RTC |
| [05_I2C_SHTC3](examples/arduino/examples/05_I2C_SHTC3/) | Temperature and humidity |
| [06_SD_Card](examples/arduino/examples/06_SD_Card/) | TF card |
| [07_Audio_Test](examples/arduino/examples/07_Audio_Test/) | Audio |
| [09_LVGL_V9_Test](examples/arduino/examples/09_LVGL_V9_Test/) | LVGL 9 |
| [10_U8G2_Test](examples/arduino/examples/10_U8G2_Test/) | U8g2 |

Libraries used by the sketches are bundled in [`examples/arduino/libraries`](examples/arduino/libraries/).
ESPHome configurations are in [`examples/esphome`](examples/esphome/README.md) and the XiaoZhi AI firmware source in [`examples/xiaozhi`](examples/xiaozhi/).

## 🛠️ Development Environment

| Framework | Version | Examples |
| --- | --- | ---: |
| ESP-IDF | `v5.5.5` | 11 |
| Arduino-ESP32 | `3.3.12` | 9 |

## 📚 Resources

- [Product page](https://www.waveshare.com/esp32-s3-rlcd-4.2.htm)
- [Wiki](https://docs.waveshare.com/ESP32-S3-RLCD-4.2) (schematic, pinout and tutorials)

## 🤝 Support

- Example or repository problem: [open an issue](https://github.com/waveshareteam/ESP32-S3-RLCD-4.2/issues/new/choose).
- Product usage, hardware or order questions: [Waveshare support](https://service.waveshare.com/).

## 📄 License

Apache License 2.0. See [LICENSE](LICENSE).
