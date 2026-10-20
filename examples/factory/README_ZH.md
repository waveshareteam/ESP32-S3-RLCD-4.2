# 出厂固件

[English](README.md) | [简体中文](README_ZH.md)

## 功能

整板测试界面：时间、温湿度、电量、TF 卡检测、Wi-Fi 和蓝牙扫描，以及音频测试。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板
- TF 卡（可选）
- 扬声器（可选）

## 怎么运行

```bash
idf.py set-target esp32s3 build flash monitor
```

## 预期现象

屏幕显示时钟、温湿度和电量，TF 卡检测结果（`passed`、`failed` 或 `No Card`），以及扫描到的 Wi-Fi 热点和蓝牙设备数量。按键可进行与 07_Audio_Test 相同的录音和播放测试。

## 常见问题

暂无。
