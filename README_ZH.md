<div align="center">
  <h1>ESP32-S3-RLCD-4.2</h1>
  <p><strong>基于 ESP32-S3 的 4.2 英寸全反射屏 AIoT 开发板</strong></p>
  <p>
    <a href="LICENSE"><img alt="License" src="https://img.shields.io/github/license/waveshareteam/ESP32-S3-RLCD-4.2"></a>
  </p>
  <p>
    <a href="README.md">English</a> ·
    <a href="https://www.waveshare.net/shop/ESP32-S3-RLCD-4.2.htm">🌐 产品页面</a> ·
    <a href="https://docs.waveshare.net/ESP32-S3-RLCD-4.2">📚 文档</a> ·
    <a href="03_Firmware/">📦 出厂固件</a> ·
    <a href="02_Example/ESP-IDF/">🧩 ESP-IDF 示例</a> ·
    <a href="02_Example/Arduino/">🔧 Arduino 示例</a>
  </p>
  <p>
    <img src="assets/Product-1.webp" alt="Waveshare ESP32-S3-RLCD-4.2" width="500">
  </p>
</div>

---

## ✨ 概述

ESP32-S3-RLCD-4.2 是一款基于 ESP32-S3-WROOM-1-N16R8（双核 Xtensa LX7 @ 240 MHz，8 MB PSRAM，16 MB Flash）的全反射屏 AIoT 开发板，支持 2.4 GHz Wi-Fi 和 Bluetooth 5 LE。板载 4.2 英寸 300 × 400 全反射 LCD（无需背光）和音频模块（ES8311 + ES7210 双麦克风阵列），并集成 PCF85063 RTC、SHTC3 温湿度传感器、TF 卡槽、USB Type-C 及 18650 电池座，适用于智能摆件、电子日历和 AI 智能体等应用。

## 🗂️ 仓库结构

本仓库提供 ESP32-S3-RLCD-4.2 的示例程序、Arduino 库和出厂固件。

```
.
├── 01_Arduino_Libraries/   # 第三方库（SensorLib、U8g2、LVGL）
├── 02_Example/             # Arduino、ESP-IDF、ESPHome 与小智示例程序
├── 03_Firmware/            # 预编译固件（.bin）
└── Tools-Configuration.png # Arduino IDE 开发板配置参考
```

## 🚀 快速开始

预编译固件位于 [`03_Firmware/`](03_Firmware)。构建环境、烧录步骤、引脚映射及配置说明请参阅[产品文档页面](https://docs.waveshare.net/ESP32-S3-RLCD-4.2)。

<details>
<summary>Arduino IDE 开发板配置</summary>

<img src="Tools-Configuration.png" alt="Arduino IDE Tools Configuration" width="500">

</details>

## 🤝 贡献

我们欢迎您的贡献！您可以通过以下方式提供帮助：

1. Fork 本仓库。
2. 为您的新功能或 Bug 修复创建一个新分支。
3. 提交您的更改并附上清晰的描述。
4. 提交 Pull Request 以供审核。

## 🤝 问题与支持

请创建 [Issue](https://gitee.com/waveshare/ESP32-S3-RLCD-4.2/issues) 并提供详细信息，或联系微雪团队并提供订单号以获取技术支持。

## 📄 许可

本仓库遵循 Apache License 2.0 许可。详情请参阅 [LICENSE](LICENSE) 文件。

---

感谢您使用微雪电子产品！🚀
