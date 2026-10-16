# 06_SD_Card

[English](README.md) | [简体中文](README_ZH.md)

## 功能

把计数值写入 TF 卡上的 `writeTest.txt`，再读回来。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板
- TF 卡

## 怎么运行

```bash
idf.py set-target esp32s3 build flash monitor
```

## 预期现象

串口大约每秒打印一次 `rtest:sdcard_writeTest : <n>`，数字递增。结果只输出到串口，屏幕不使用。

## 常见问题

- `SD card not initialized`：先插入 FAT 格式的卡，再复位开发板。
