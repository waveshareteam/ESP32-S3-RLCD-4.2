#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include "button_bsp.h"
#include "user_app.h"
#include "gui_guider.h"
#include "i2c_equipment.h"
#include "i2c_bsp.h"
#include "sdcard_bsp.h"
#include "codec_bsp.h"
#include "adc_bsp.h"
#include "esp_wifi_bsp.h"
#include "ble_scan_bsp.h"

static lv_ui init_ui;
I2cMasterBus I2cbus(14,13,0);
CustomSDPort *sdcardPort = NULL;
Shtc3Port *shtc3port = NULL;
EventGroupHandle_t CodecGroups;
CodecPort *codecport = NULL;
static uint8_t *audio_ptr = NULL;
static bool is_Music = true;

void Lvgl_Cont1Task(void *arg) {
    lv_obj_clear_flag(init_ui.screen_label_1,LV_OBJ_FLAG_HIDDEN); 
    lv_obj_add_flag(init_ui.screen_label_2, LV_OBJ_FLAG_HIDDEN);
    vTaskDelay(pdMS_TO_TICKS(1500));
    lv_obj_clear_flag(init_ui.screen_label_2,LV_OBJ_FLAG_HIDDEN); 
    lv_obj_add_flag(init_ui.screen_label_1, LV_OBJ_FLAG_HIDDEN);
    vTaskDelay(pdMS_TO_TICKS(1500));
    lv_obj_clear_flag(init_ui.screen_cont_2,LV_OBJ_FLAG_HIDDEN); 
    lv_obj_add_flag(init_ui.screen_cont_1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(init_ui.screen_cont_3, LV_OBJ_FLAG_HIDDEN);
    vTaskDelete(NULL); 
}

void Lvgl_UserTask(void *arg) {
    uint32_t times = 0;
    uint32_t adc_time = 0;
    uint32_t rtc_time = 0;
    uint32_t shtc3_time = 0;
    char lvgl_buffer[30] = {""};
    for(;;) {
        if(times - adc_time == 10) {
            adc_time = times;
            uint8_t level = Adc_GetBatteryLevel();
            snprintf(lvgl_buffer,30,"%d%%",level);
            lv_label_set_text(init_ui.screen_label_7, lvgl_buffer);
        }
        if(times - rtc_time == 5) {
            rtc_time = times;
            rtcTimeStruct_t timerData;
            Rtc_GetTime(&timerData);
            snprintf(lvgl_buffer,30,"%02d",timerData.minute);
            lv_label_set_text(init_ui.screen_label_3, lvgl_buffer);
            snprintf(lvgl_buffer,30,"%02d",timerData.second);
            lv_label_set_text(init_ui.screen_label_4, lvgl_buffer);
        }
        if(times - shtc3_time == 25)
        {
            shtc3_time = times;
            float rh,temp;
            shtc3port->Shtc3_ReadTempHumi(&temp,&rh);
            snprintf(lvgl_buffer,30,"%d%%",(int)rh);
            lv_label_set_text(init_ui.screen_label_11, lvgl_buffer);
            snprintf(lvgl_buffer,30,"%d°",(int)temp);
            lv_label_set_text(init_ui.screen_label_12, lvgl_buffer);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
        times++;
    }
}

void Lvgl_SDcardTask(void *arg) {
    const char *str_write = "waveshare.com";
    char str_read[20] = {""};
    if(0 == sdcardPort->SDPort_GetStatus()) {
        lv_label_set_text(init_ui.screen_label_6, "No Card");
    } else {
        sdcardPort->SDPort_WriteFile("/sdcard/sdcard.txt",str_write,strlen(str_write));
        sdcardPort->SDPort_ReadFile("/sdcard/sdcard.txt",(uint8_t *)str_read,NULL);
        if(!strcmp(str_write,str_read)) {
            lv_label_set_text(init_ui.screen_label_6, "passed");
        } else {
            lv_label_set_text(init_ui.screen_label_6, "failed");
        }
    }
    vTaskDelete(NULL);
}

void Lvgl_WfifBleScanTask(void *srg) {
    char send_lvgl[10] = {""};
    uint8_t ble_scan_count = 0;
    uint8_t ble_mac[6];
    EventBits_t even = xEventGroupWaitBits(wifi_even_,0x02,pdTRUE,pdTRUE,pdMS_TO_TICKS(30000)); 
    espwifi_deinit(); //释放WIFI
    ble_scan_prepare();
    ble_stack_init();
    ble_scan_start();
    for(;xQueueReceive(ble_queue,ble_mac,3500) == pdTRUE;) {
        ble_scan_count++;
        if(ble_scan_count >= 20)
        break;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    if(get_bit_data(even,1)) {
        snprintf(send_lvgl,9,"%d",user_esp_bsp.apNum);
        lv_label_set_text(init_ui.screen_label_14, send_lvgl);
    } else {
        lv_label_set_text(init_ui.screen_label_14, "P");
    }
    snprintf(send_lvgl,10,"%d",ble_scan_count);
    lv_label_set_text(init_ui.screen_label_13, send_lvgl);
    ble_stack_deinit();    //释放BLE
    vTaskDelete(NULL);
}

void BOOT_LoopTask(void *arg) {
    bool is_cont4en = 0;
    for(;;) {
        EventBits_t even = xEventGroupWaitBits(BootButtonGroups,(0x01 | 0x02 | 0x04),pdTRUE,pdFALSE,pdMS_TO_TICKS(2000));
        if(even & 0x04) {
            if(0 == is_cont4en) {
                is_cont4en = 1;
                lv_obj_clear_flag(init_ui.screen_cont_4,LV_OBJ_FLAG_HIDDEN); 
                lv_obj_add_flag(init_ui.screen_cont_1, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(init_ui.screen_cont_2, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(init_ui.screen_cont_3, LV_OBJ_FLAG_HIDDEN);
            } else {
                is_cont4en = 0;
                lv_obj_clear_flag(init_ui.screen_cont_2,LV_OBJ_FLAG_HIDDEN); 
                lv_obj_add_flag(init_ui.screen_cont_1, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(init_ui.screen_cont_4, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(init_ui.screen_cont_3, LV_OBJ_FLAG_HIDDEN);
            }
        } else if(even & 0x01) {
            xEventGroupSetBits(CodecGroups,0x02);
        } else if(even & 0x02) {
            xEventGroupSetBits(CodecGroups,0x01);
        }
    }
}

void KEY_LoopTask(void *arg) {
    bool is_cont3en = 0;
    for(;;) {
        EventBits_t even = xEventGroupWaitBits(GP18ButtonGroups,(0x01 | 0x02 | 0x04),pdTRUE,pdFALSE,pdMS_TO_TICKS(2000));
        if(even & 0x01) {
            is_Music = false;
        } else if(even & 0x02) {
            is_Music = true;
            xEventGroupSetBits(CodecGroups,0x04);
        } else if(even & 0x04) {
            if(0 == is_cont3en) {
                is_cont3en = 1;
                lv_obj_clear_flag(init_ui.screen_cont_3,LV_OBJ_FLAG_HIDDEN); 
                lv_obj_add_flag(init_ui.screen_cont_1, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(init_ui.screen_cont_2, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(init_ui.screen_cont_4, LV_OBJ_FLAG_HIDDEN);
            } else {
                is_cont3en = 0;
                lv_obj_clear_flag(init_ui.screen_cont_2,LV_OBJ_FLAG_HIDDEN); 
                lv_obj_add_flag(init_ui.screen_cont_1, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(init_ui.screen_cont_3, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(init_ui.screen_cont_4, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

void Codec_LoopTask(void *arg) {
    bool is_eco = 0;
    for(;;) {
        EventBits_t even = xEventGroupWaitBits(CodecGroups,(0x01 | 0x02 | 0x04),pdTRUE,pdFALSE,pdMS_TO_TICKS(8 * 1000));
		if(even & 0x01)
		{
			lv_label_set_text(init_ui.screen_label_15, "正在录音");
			lv_label_set_text(init_ui.screen_label_17, "Recording...");
			codecport->CodecPort_EchoRead(audio_ptr,192 * 1000);
			lv_label_set_text(init_ui.screen_label_15, "录音完成");
			lv_label_set_text(init_ui.screen_label_17, "Rec Done");
            is_eco = 1;
		}
		else if(even & 0x02)
		{
            if(1 == is_eco) {
                is_eco = 0;
                lv_label_set_text(init_ui.screen_label_15, "正在播放");
			    lv_label_set_text(init_ui.screen_label_17, "Playing...");
			    codecport->CodecPort_PlayWrite(audio_ptr,192 * 1000);
			    lv_label_set_text(init_ui.screen_label_15, "播放完成");
			    lv_label_set_text(init_ui.screen_label_17, "Play Done");
            }
		}
		else if(even & 0x04)
		{
			lv_label_set_text(init_ui.screen_label_15, "正在播放音乐");
			lv_label_set_text(init_ui.screen_label_17, "Play Music");
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
			lv_label_set_text(init_ui.screen_label_15, "播放完成");
			lv_label_set_text(init_ui.screen_label_17, "Play Done");
		}
		else
		{
			lv_label_set_text(init_ui.screen_label_15, "等待操作");
			lv_label_set_text(init_ui.screen_label_17, "Idle");
		}
    }
}

void UserApp_AppInit() {
    audio_ptr = (uint8_t *)heap_caps_malloc(288 * 1000 * sizeof(uint8_t), MALLOC_CAP_SPIRAM);
    assert(audio_ptr);
    sdcardPort = new CustomSDPort("/sdcard");
    Adc_PortInit();
    Custom_ButtonInit();
    Rtc_Setup(&I2cbus,0x51);
    Rtc_SetTime(2026,1,5,14,30,30);
    shtc3port = new Shtc3Port(I2cbus);
    espwifi_init();
    CodecGroups = xEventGroupCreate();
    codecport = new CodecPort(I2cbus,"S3_RLCD_4_2");
    codecport->CodecPort_SetInfo("es8311 & es7210",1,16000,2,16);
    codecport->CodecPort_SetSpeakerVol(100);
    codecport->CodecPort_SetMicGain(35);
}

void UserApp_UiInit() {
    setup_ui(&init_ui);
    lv_label_set_text(init_ui.screen_label_8, "ON");
    lv_label_set_text(init_ui.screen_label_15, "等待操作");
}

void UserApp_TaskInit() {
    xTaskCreatePinnedToCore(Lvgl_Cont1Task, "Lvgl_Cont1Task", 4 * 1024, NULL, 2, NULL,1);
    xTaskCreatePinnedToCore(Lvgl_UserTask, "Lvgl_UserTask", 5 * 1024, NULL, 2, NULL,1);
    xTaskCreatePinnedToCore(Lvgl_SDcardTask, "Lvgl_SDcardTask", 4 * 1024, NULL, 2, NULL,1);
    xTaskCreatePinnedToCore(Lvgl_WfifBleScanTask, "Lvgl_WfifBleScanTask", 4 * 1024, NULL, 2, NULL,1);
    xTaskCreatePinnedToCore(BOOT_LoopTask, "BOOT_LoopTask", 4 * 1024, NULL, 2, NULL,1);
    xTaskCreatePinnedToCore(KEY_LoopTask, "KEY_LoopTask", 4 * 1024, NULL, 2, NULL,1);
    xTaskCreatePinnedToCore(Codec_LoopTask, "Codec_LoopTask", 4 * 1024, NULL, 4, NULL,1);
}
