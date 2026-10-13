#include "display_bsp.h"
#include "src/app_bsp/lvgl_bsp.h"
#include "src/ExternLib/button/button_bsp.h"
#include "src/ui_src/generated/gui_guider.h"
#include "i2c_bsp.h"
#include "codec_bsp.h"

static lv_ui init_ui;
I2cMasterBus I2cbus(14, 13, 0);
CodecPort *codecport = NULL;
static uint8_t *audio_ptr = NULL;
static bool is_Music = true;
EventGroupHandle_t CodecGroups;

DisplayPort RlcdPort(12, 11, 5, 40, 41, 400, 300);

static void Lvgl_FlushCallback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  uint16_t *buffer = (uint16_t *)color_map;
  for (int y = area->y1; y <= area->y2; y++) {
    for (int x = area->x1; x <= area->x2; x++) {
      uint8_t color = (*buffer < 0x7fff) ? ColorBlack : ColorWhite;
      RlcdPort.RLCD_SetPixel(x, y, color);
      buffer++;
    }
  }
  RlcdPort.RLCD_Display();
  lv_disp_flush_ready(drv);
}

void BOOT_LoopTask(void *arg) {
  for (;;) {
    EventBits_t even = xEventGroupWaitBits(BootButtonGroups, (0x01 | 0x02 | 0x04), pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
    if (even & 0x01) {
      xEventGroupSetBits(CodecGroups, 0x02);
    } else if (even & 0x02) {
      xEventGroupSetBits(CodecGroups, 0x01);
    }
  }
}

void Codec_LoopTask(void *arg) {
  bool is_eco = 0;
  for (;;) {
    EventBits_t even = xEventGroupWaitBits(CodecGroups, (0x01 | 0x02 | 0x04), pdTRUE, pdFALSE, pdMS_TO_TICKS(8 * 1000));
    if (even & 0x01) {
      lv_label_set_text(init_ui.screen_label_1, "正在录音");
      lv_label_set_text(init_ui.screen_label_2, "Recording...");
      codecport->CodecPort_EchoRead(audio_ptr, 192 * 1000);
      lv_label_set_text(init_ui.screen_label_1, "录音完成");
      lv_label_set_text(init_ui.screen_label_2, "Rec Done");
      is_eco = 1;
    } else if (even & 0x02) {
      if (1 == is_eco) {
        is_eco = 0;
        lv_label_set_text(init_ui.screen_label_1, "正在播放");
        lv_label_set_text(init_ui.screen_label_2, "Playing...");
        codecport->CodecPort_PlayWrite(audio_ptr, 192 * 1000);
        lv_label_set_text(init_ui.screen_label_1, "播放完成");
        lv_label_set_text(init_ui.screen_label_2, "Play Done");
      }
    } else if (even & 0x04) {
      lv_label_set_text(init_ui.screen_label_1, "正在播放音乐");
      lv_label_set_text(init_ui.screen_label_2, "Play Music");
      codecport->CodecPort_SetSpeakerVol(90);
      uint32_t bytes_sizt;
      size_t bytes_write = 0;
      uint8_t *data_ptr = codecport->CodecPort_GetPcmData(&bytes_sizt);
      while (bytes_write < bytes_sizt) {
        codecport->CodecPort_PlayWrite(data_ptr, 256);
        data_ptr += 256;
        bytes_write += 256;
        if (!is_Music)
          break;
      }
      codecport->CodecPort_SetSpeakerVol(100);
      lv_label_set_text(init_ui.screen_label_1, "播放完成");
      lv_label_set_text(init_ui.screen_label_2, "Play Done");
    } else {
      lv_label_set_text(init_ui.screen_label_1, "等待操作");
      lv_label_set_text(init_ui.screen_label_2, "IDLE");
    }
  }
}

void KEY_LoopTask(void *arg) {
  for (;;) {
    EventBits_t even = xEventGroupWaitBits(GP18ButtonGroups, (0x01 | 0x02 | 0x04), pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
    if (even & 0x01) {
      is_Music = false;
    } else if (even & 0x02) {
      is_Music = true;
      xEventGroupSetBits(CodecGroups, 0x04);
    }
  }
}

void setup() {
  audio_ptr = (uint8_t *)heap_caps_malloc(288 * 1000 * sizeof(uint8_t), MALLOC_CAP_SPIRAM);
  assert(audio_ptr);
  CodecGroups = xEventGroupCreate();
  Custom_ButtonInit();
  codecport = new CodecPort(I2cbus, "S3_RLCD_4_2");
  codecport->CodecPort_SetInfo("es8311 & es7210", 1, 16000, 2, 16);
  codecport->CodecPort_SetSpeakerVol(100);
  codecport->CodecPort_SetMicGain(35);
  RlcdPort.RLCD_Init();
  Lvgl_PortInit(400, 300, Lvgl_FlushCallback);
  if (Lvgl_lock(-1)) {
    setup_ui(&init_ui);
    lv_label_set_text(init_ui.screen_label_1, "等待操作");
    lv_label_set_text(init_ui.screen_label_2, "IDLE");
    Lvgl_unlock();
  }
  xTaskCreatePinnedToCore(BOOT_LoopTask, "BOOT_LoopTask", 4 * 1024, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(KEY_LoopTask, "KEY_LoopTask", 4 * 1024, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Codec_LoopTask, "Codec_LoopTask", 4 * 1024, NULL, 4, NULL, 1);
}

void loop() {
}
