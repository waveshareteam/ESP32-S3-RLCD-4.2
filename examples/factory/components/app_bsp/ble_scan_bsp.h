#ifndef BLE_SCAN_BSP_H
#define BLE_SCAN_BSP_H

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"


#ifdef __cplusplus
extern "C" {
#endif

void ble_scan_prepare(void);  //初始化之前的释放内存
void ble_stack_init(void);    //ble初始化
void ble_scan_start(void);    //ble扫描开始
void ble_stack_deinit(void);  //ble反初始化 

extern QueueHandle_t ble_queue;



#ifdef __cplusplus
}
#endif

#endif
