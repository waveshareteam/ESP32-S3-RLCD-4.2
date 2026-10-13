# ESPHome 示例配置

[English](README.md)

适用于 ESP32-S3-RLCD-4.2 的 ESPHome YAML 配置，可直接使用。烧录前请在 `secrets.yaml` 中填写 `wifi_ssid` 和 `wifi_password`。

| 文件 | 说明 | 用到的硬件 |
| --- | --- | --- |
| [esp32-s3-rlcd-42-sensor.yaml](./esp32-s3-rlcd-42-sensor.yaml) | 传感器仪表盘：温湿度、电池电量、进度条 | 显示屏 / SHTC3 / 电池 ADC / BOOT + KEY |
| [esp32-s3-rlcd-42-speaker.yaml](./esp32-s3-rlcd-42-speaker.yaml) | 传感器仪表盘 + 扬声器 RTTTL 铃声测试 | 显示屏 / SHTC3 / 电池 ADC / 扬声器 |
| [esp32-s3-rlcd-42-wake-word.yaml](./esp32-s3-rlcd-42-wake-word.yaml) | 设备端唤醒词检测，并在屏幕上显示计数 | 显示屏 / 双麦克风 |

## 使用方法

1. 在 ESPHome 面板中依次点击 **NEW DEVICE → Skip**，创建一个空设备。
2. 点击设备卡片上的 **EDIT**，粘贴 YAML 内容。
3. 将 `REPLACE_WITH_YOUR_KEY` / `REPLACE_WITH_YOUR_PASSWORD` 替换为向导生成的值，也可以新建一个设备并复制它的凭据。
4. 点击 **SAVE → INSTALL**。首次烧录需通过 USB，之后可选择 **Wirelessly**（OTA）更新。

## 教程

- [中文教程](https://docs.waveshare.net/ESP32-ESPHome-Tutorials)
- [English tutorial](https://docs.waveshare.com/ESP32-ESPHome-Tutorials)
