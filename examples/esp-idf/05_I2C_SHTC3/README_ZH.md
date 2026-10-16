# 05_I2C_SHTC3

[English](README.md) | [简体中文](README_ZH.md)

## 功能

每秒读取一次 SHTC3 的温度和相对湿度。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

```bash
idf.py set-target esp32s3 build flash monitor
```

## 预期现象

串口每秒打印相对湿度和温度。结果只输出到串口，屏幕不使用。

## 常见问题

暂无。
