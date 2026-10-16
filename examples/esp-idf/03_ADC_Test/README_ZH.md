# 03_ADC_Test

[English](README.md) | [简体中文](README_ZH.md)

## 功能

每秒通过 ADC 读取一次电池电压。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板
- 18650 电池（可选）

## 怎么运行

```bash
idf.py set-target esp32s3 build flash monitor
```

## 预期现象

串口每秒打印 ADC 原始值和电池电压。结果只输出到串口，屏幕不使用。

## 常见问题

暂无。
