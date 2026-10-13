<div align="center">
  <h1>ESP32-S3-RLCD-4.2</h1>
  <p><strong>ESP32-S3 4.2-inch fully reflective LCD AIoT development board</strong></p>
  <p>
    <a href="LICENSE"><img alt="License" src="https://img.shields.io/github/license/waveshareteam/ESP32-S3-RLCD-4.2"></a>
  </p>
  <p>
    <a href="README_ZH.md">中文</a> ·
    <a href="https://www.waveshare.com/esp32-s3-rlcd-4.2.htm">🌐 Product Page</a> ·
    <a href="https://docs.waveshare.com/ESP32-S3-RLCD-4.2">📚 Documentation</a> ·
    <a href="firmware/">📦 Factory Firmware</a> ·
    <a href="examples/esp-idf/">🧩 ESP-IDF Examples</a> ·
    <a href="examples/arduino/examples/">🔧 Arduino Examples</a>
  </p>
  <p>
    <img src="assets/Product-1.webp" alt="Waveshare ESP32-S3-RLCD-4.2" width="500">
  </p>
</div>

---

## ✨ Overview

The ESP32-S3-RLCD-4.2 is a fully reflective screen AIoT development board based on the ESP32-S3-WROOM-1-N16R8 (dual-core Xtensa LX7 @ 240 MHz, 8 MB PSRAM, 16 MB Flash) with 2.4 GHz Wi-Fi and Bluetooth 5 LE. It features a 4.2-inch 300 × 400 reflective LCD (no backlight required), an audio module (ES8311 + ES7210 with dual-mic array), along with a PCF85063 RTC, SHTC3 temperature/humidity sensor, TF card slot, USB Type-C, and an 18650 battery holder, making it ideal for smart ornaments, electronic calendars, and AI agent applications.

## 🗂️ Repository Structure

This repository provides sample programs, Arduino libraries, and factory firmware for the ESP32-S3-RLCD-4.2.

```
.
├── examples/
│   ├── arduino/
│   │   ├── examples/       # Arduino sketches
│   │   └── libraries/      # Third-party libraries (SensorLib, U8g2, LVGL)
│   ├── esp-idf/            # ESP-IDF examples
│   ├── esphome/            # ESPHome YAML configurations
│   └── xiaozhi/            # XiaoZhi AI application source
├── firmware/               # Pre-built firmware (.bin)
└── Tools-Configuration.png # Arduino IDE board settings reference
```

The directory layout was reorganized. If you are following an older link or tutorial, use the table below to find the new location.

| Previous path | Current path |
| --- | --- |
| `01_Arduino_Libraries/` | [`examples/arduino/libraries/`](examples/arduino/libraries/) |
| `02_Example/Arduino/` | [`examples/arduino/examples/`](examples/arduino/examples/) |
| `02_Example/ESP-IDF/` | [`examples/esp-idf/`](examples/esp-idf/) |
| `02_Example/ESPHome/` | [`examples/esphome/`](examples/esphome/README.md) |
| `02_Example/XiaoZhi/` | [`examples/xiaozhi/`](examples/xiaozhi/) |
| `03_Firmware/` | [`firmware/`](firmware/) |

## 🚀 Getting Started

Pre-built firmware is available in [`firmware/`](firmware/). For build environments, flashing steps, pin mappings, and configuration, refer to the [documentation page](https://docs.waveshare.com/ESP32-S3-RLCD-4.2).

<details>
<summary>Arduino IDE Tools Configuration</summary>

<img src="Tools-Configuration.png" alt="Arduino IDE Tools Configuration" width="500">

</details>

## 🤝 Contributing

We welcome contributions! Here's how you can help:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Commit your changes with clear descriptions.
4. Submit a pull request for review.

## 🤝 Issues and Support

Open an [issue](https://github.com/waveshareteam/ESP32-S3-RLCD-4.2/issues) with detailed information, or contact the Waveshare team with your order number for technical support.

## 📄 License

Licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

---

Thank you for using Waveshare Electronics Products! 🚀
