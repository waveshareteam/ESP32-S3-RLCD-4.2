# 07_Audio_Test

[English](README.md) | [简体中文](README_ZH.md)

## 功能

通过两个按键控制麦克风录音，并经 ES8311 播放。

## 硬件需求

- ESP32-S3-RLCD-4.2 开发板
- 扬声器

## 怎么运行

```bash
idf.py set-target esp32s3 build flash monitor
```

## 预期现象

屏幕显示当前状态（`IDLE`、`Recording...`、`Playing...`）。双击 BOOT 录一段音，单击 BOOT 回放；双击 KEY 播放内置音乐，单击 KEY 停止。

## 常见问题

暂无。
