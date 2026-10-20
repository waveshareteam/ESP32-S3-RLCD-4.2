#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include "adc_bsp.h"
#include "esp_log.h"

static adc_cali_handle_t cali_handle;
static adc_oneshot_unit_handle_t adc1_handle;


void Adc_PortInit() {
    adc_cali_curve_fitting_config_t cali_config = {};
    cali_config.unit_id = ADC_UNIT_1;
    cali_config.atten = ADC_ATTEN_DB_12;
    cali_config.bitwidth = ADC_BITWIDTH_12;
  	ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));

    adc_oneshot_unit_init_cfg_t init_config1 = {};
    init_config1.unit_id = ADC_UNIT_1;
  	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
  	adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = ADC_BITWIDTH_12;            
    config.atten = ADC_ATTEN_DB_12;
  	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
}

float Adc_GetBatteryVoltage() {
	int value;
  	int tage = 0;
    float vol = 0;
  	esp_err_t err;
  	err = adc_oneshot_read(adc1_handle,ADC_CHANNEL_3,&value);
  	if(err == ESP_OK) {
    	adc_cali_raw_to_voltage(cali_handle,value,&tage);
    	vol = 0.001 * tage * 3;
	}
	return vol;
}

uint8_t Adc_GetBatteryLevel() {
	float vol = Adc_GetBatteryVoltage();
    if(vol < 3.0) {
        return 0;
    }
    if(vol > 4.12) {
        return 100;
    }
    float level = ((vol - 3.0) / 1.12) * 100;
	//ESP_LOGW("Battery","Voltage: %.3f V, Level: %.1f %%",vol,level);
    return (uint8_t)level;
}
