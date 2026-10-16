# 11_U8G2_Test

[English](README.md) | [简体中文](README_ZH.md)

## 功能

用 U8g2 图形库在反射式 LCD 上绘制帧计数器。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板

## 怎么运行

```bash
idf.py set-target esp32s3 build flash monitor
```

## 预期现象

屏幕显示递增的计数，下方是 `ST7305 refresh counter`，底部是帧率；串口同步打印计时数据。

## 常见问题

暂无。
