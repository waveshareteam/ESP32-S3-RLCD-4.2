# 04_I2C_PCF85063

[English](README.md) | [简体中文](README_ZH.md)

## 功能

设置 PCF85063 实时时钟，每秒打印一次时间。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

```bash
idf.py set-target esp32s3 build flash monitor
```

## 预期现象

串口每秒打印日期和时间，从 2025/9/9 20:15:30 开始走时。结果只输出到串口，屏幕不使用。

## 常见问题

- 示例每次启动都把固定时间写入 RTC；想从当前时间开始，修改 `components/user_app/` 中的 `Rtc_SetTime()` 调用。
