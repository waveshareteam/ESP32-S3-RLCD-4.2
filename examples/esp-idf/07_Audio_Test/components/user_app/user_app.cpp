#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>

#include "button_bsp.h"
#include "user_app.h"
#include "gui_guider.h"
#include "i2c_bsp.h"
#include "codec_bsp.h"

static lv_ui init_ui;
I2cMasterBus I2cbus(14,13,0);
CodecPort *codecport = NULL;
static uint8_t *audio_ptr = NULL;
static bool is_Music = true;
EventGroupHandle_t CodecGroups;

void BOOT_LoopTask(void *arg) {
    for(;;) {
        EventBits_t even = xEventGroupWaitBits(BootButtonGroups,(0x01 | 0x02 | 0x04),pdTRUE,pdFALSE,pdMS_TO_TICKS(2000));
        if(even & 0x01) {
            xEventGroupSetBits(CodecGroups,0x02);
        } else if(even & 0x02) {
            xEventGroupSetBits(CodecGroups,0x01);
        }
    }
}

void Codec_LoopTask(void *arg) {
    bool is_eco = 0;
    for(;;) {
        EventBits_t even = xEventGroupWaitBits(CodecGroups,(0x01 | 0x02 | 0x04),pdTRUE,pdFALSE,pdMS_TO_TICKS(8 * 1000));
		if(even & 0x01)
		{
			lv_label_set_text(init_ui.screen_label_1, "正在录音");
			lv_label_set_text(init_ui.screen_label_2, "Recording...");
			codecport->CodecPort_EchoRead(audio_ptr,192 * 1000);
			lv_label_set_text(init_ui.screen_label_1, "录音完成");
			lv_label_set_text(init_ui.screen_label_2, "Rec Done");
            is_eco = 1;
		}
		else if(even & 0x02)
		{
            if(1 == is_eco) {
                is_eco = 0;
                lv_label_set_text(init_ui.screen_label_1, "正在播放");
			    lv_label_set_text(init_ui.screen_label_2, "Playing...");
			    codecport->CodecPort_PlayWrite(audio_ptr,192 * 1000);
			    lv_label_set_text(init_ui.screen_label_1, "播放完成");
			    lv_label_set_text(init_ui.screen_label_2, "Play Done");
            }
		}
		else if(even & 0x04)
		{
			lv_label_set_text(init_ui.screen_label_1, "正在播放音乐");
			lv_label_set_text(init_ui.screen_label_2, "Play Music");
			codecport->CodecPort_SetSpeakerVol(90);
			uint32_t bytes_sizt;
			size_t bytes_write = 0;
			uint8_t *data_ptr = codecport->CodecPort_GetPcmData(&bytes_sizt);
			while (bytes_write < bytes_sizt)
            {
                codecport->CodecPort_PlayWrite(data_ptr, 256);
                data_ptr += 256;
                bytes_write += 256;
				if(!is_Music)
				break;
            }
			codecport->CodecPort_SetSpeakerVol(100);
			lv_label_set_text(init_ui.screen_label_1, "播放完成");
			lv_label_set_text(init_ui.screen_label_2, "Play Done");
		}
		else
		{
			lv_label_set_text(init_ui.screen_label_1, "等待操作");
			lv_label_set_text(init_ui.screen_label_2, "IDLE");
		}
    }
}

void KEY_LoopTask(void *arg) {
    for(;;) {
        EventBits_t even = xEventGroupWaitBits(GP18ButtonGroups,(0x01 | 0x02 | 0x04),pdTRUE,pdFALSE,pdMS_TO_TICKS(2000));
        if(even & 0x01) {
            is_Music = false;
        } else if(even & 0x02) {
            is_Music = true;
            xEventGroupSetBits(CodecGroups,0x04);
        }
    }
}


void UserApp_AppInit() {
    audio_ptr = (uint8_t *)heap_caps_malloc(288 * 1000 * sizeof(uint8_t), MALLOC_CAP_SPIRAM);
    assert(audio_ptr);
    CodecGroups = xEventGroupCreate();
    Custom_ButtonInit();
    codecport = new CodecPort(I2cbus,"S3_RLCD_4_2");
    codecport->CodecPort_SetInfo("es8311 & es7210",1,16000,2,16);
    codecport->CodecPort_SetSpeakerVol(100);
    codecport->CodecPort_SetMicGain(35);
}

void UserApp_UiInit() {
    setup_ui(&init_ui);
    lv_label_set_text(init_ui.screen_label_1, "等待操作");
    lv_label_set_text(init_ui.screen_label_2, "IDLE");
}

void UserApp_TaskInit() {
    xTaskCreatePinnedToCore(BOOT_LoopTask, "BOOT_LoopTask", 4 * 1024, NULL, 2, NULL,1);
    xTaskCreatePinnedToCore(KEY_LoopTask, "KEY_LoopTask", 4 * 1024, NULL, 2, NULL,1);
    xTaskCreatePinnedToCore(Codec_LoopTask, "Codec_LoopTask", 4 * 1024, NULL, 4, NULL,1);
}
