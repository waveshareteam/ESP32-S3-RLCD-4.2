# ESPHome

[English](README.md)

[ESPHome](https://esphome.io/) 让你只用简单的 YAML 就能为 [Home Assistant](https://www.home-assistant.io/) 配置 ESP32 设备，无需编写 C/C++ 代码。

## 示例

[`examples/`](examples/README_ZH.md) 目录提供可直接烧录的 YAML 配置：

| 示例 | 说明 |
| --- | --- |
| [sensor](examples/esp32-s3-rlcd-42-sensor.yaml) | 显示屏 + SHTC3 温湿度 + 电池监测 + 按键 |
| [speaker](examples/esp32-s3-rlcd-42-speaker.yaml) | 传感器仪表盘 + ES8311 扬声器 RTTTL 铃声测试 |
| [wake-word](examples/esp32-s3-rlcd-42-wake-word.yaml) | 设备端 “Hey Jarvis” 唤醒词检测，并在屏幕上显示计数 |

## 快速开始

1. 安装 [Home Assistant](https://www.home-assistant.io/installation/)，并添加 [ESPHome 加载项](https://esphome.io/guides/getting_started_hassio.html)。
2. 在 ESPHome 中新建设备，点击 **EDIT**，粘贴示例 YAML。
3. 在 `secrets.yaml` 中填写 Wi-Fi 凭据：
   ```yaml
   wifi_ssid: "YourSSID"
   wifi_password: "YourPassword"
   ```
4. 点击 **SAVE → INSTALL**。首次烧录需通过 USB，之后可通过 OTA 更新。

## 教程

涵盖 Home Assistant 安装、ESPHome 配置和设备配置的分步教程：

- [中文教程](https://docs.waveshare.net/ESP32-ESPHome-Tutorials)
- [English tutorial](https://docs.waveshare.com/ESP32-ESPHome-Tutorials)

## GPIO 引脚

|  GPIO  |             功能             |
| :----: | :--------------------------: |
| GPIO0  | BOOT 按键（低电平有效）      |
| GPIO4  | 电池 ADC                     |
| GPIO5  | 显示屏 DC                    |
| GPIO8  | I²S DOUT（扬声器）           |
| GPIO9  | I²S BCLK                     |
| GPIO10 | I²S DIN（麦克风）            |
| GPIO11 | SPI CLK（显示屏）            |
| GPIO12 | SPI MOSI（显示屏）           |
| GPIO13 | I²C SDA                      |
| GPIO14 | I²C SCL                      |
| GPIO16 | I²S MCLK                     |
| GPIO18 | KEY 按键（低电平有效）       |
| GPIO40 | 显示屏 CS                    |
| GPIO41 | 显示屏 RESET                 |
| GPIO45 | I²S LRCLK                    |
| GPIO46 | 扬声器功放使能               |

## 参考资料

- [ESPHome 文档](https://esphome.io/)
- [ESPHome ST7305 外部组件](https://github.com/kylehase/ESPHome-ST7305-RLCD)
- [ESP32-S3-RLCD-4.2 产品文档](https://docs.waveshare.net/ESP32-S3-RLCD-4.2)
- [devices.esphome.io](https://devices.esphome.io/)
