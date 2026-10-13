#include <stdio.h>
#include "ble_scan_bsp.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#define GATTC_TAG "GATTC_DEMO"
QueueHandle_t ble_queue;
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0


/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


static esp_ble_scan_params_t ble_scan_params = {                //ble scan parameter Settings
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE        //Filter duplicate ads
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};


/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Didn't get the initial value, so it's ESP GATT IF NONE */
    },
};


static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) // This function is called by the esp gattc cb callback function and has the opportunity to be executed
{
    //esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
    switch (event)
    {
        case ESP_GATTC_REG_EVT: //This callback function, executed by the esp gattc cb callback function, passes in the event parameters
            ESP_LOGI(GATTC_TAG, "REG_EVT");
            //esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params); //Set scan parameters
            //if (scan_ret)
            //{
            //    ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
            //}
            break;
        case ESP_GATTC_CFG_MTU_EVT: //This event is triggered when the GATT client successfully configures the mtu by calling esp ble gattc set mtu()
            if (param->cfg_mtu.status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
            }
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
            break;
        default:
            break;
    }
}
static uint8_t value = 0;
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    //uint8_t *adv_name = NULL;
    //uint8_t adv_name_len = 0;
    switch (event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: //This event is triggered after the esp ble gap set scan params() function is successfully invoked to set scan parameters
        {
            //esp_ble_gap_start_scanning(duration); //Scan time of 30 seconds
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: //This event is triggered after the esp ble gap start scanning() function is successfully invoked to start scanning
        {
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) //To check whether the ESP BT is successfully started, the value must equal to ESP BT STATUS SUCCESS
            {
                ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
                break;
            }
            ESP_LOGI(GATTC_TAG, "scan start success");
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: //Enter once every scan, not wait for the scan to finish before entering
        {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) //    Search event type     
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    ////esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6); //Bluetooth device address adv data len is the length of the AD data for the scanned device.
                    //Purpose: Advertising data is the message that the device sends when it broadcasts. This data typically includes the name of the device, service UUID, manufacturer data, and so on. 
                    //The length of the AD data is specified by adv data len Scan response data is additional information returned by the device after receiving a request from the scan device. Typically, it contains more information about the device, such as the full device name or other service information.
                    //Scenario: When a device (such as a phone or other BLE device) scans an AD pack, it can send a scan request to request more information. The target device will respond to this request and send a scan of the response data
                    ////ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len); 
                    //adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len); //NULL is returned if the name is not scanned
                    if(xQueueSend(ble_queue,scan_result->scan_rst.bda,0) == pdTRUE)
                    {
                        value++;
                        if(value == 20)
                        {
                            value = 0;
                            esp_ble_gap_stop_scanning(); //Stop scanning
                        }
                    }
                    /*if (adv_name != NULL) 
                    {
                        //ESP_LOGI(GATTC_TAG, "adv_name: %s", adv_name);
                        //esp_ble_gap_stop_scanning();                          //Stop scanning
                        //esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true); //Open gattc and start the connection
                    }*/
                    break;
                case ESP_GAP_SEARCH_INQ_CMPL_EVT: //The event is used to notify the scanning time of the BLE device when the scanning operation is completed
                    value = 0;
                    break;
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: //Usually triggered after you call esp ble gap stop scanning() function to stop scanning
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
                break;
            }
            ESP_LOGI(GATTC_TAG, "stop scan successfully");

            break;
        default:
            break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT)              // Description The event occurred when the GATT client was registered
    {
        if (param->reg.status == ESP_GATT_OK)    // Get running status
        {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if; // param->reg.app id Get the ID, which is the gattc if generated automatically by the system
            printf("gattc_if:%d\n",gattc_if);
        } 
        else 
        {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",param->reg.app_id,param->reg.status); // Otherwise, the corresponding error structure is displayed
            return;                                                                                            //Returns, does not execute the following
        }
    }
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[idx].gattc_if) //if the callback reports gattc if/gatts if as an ESP GATT IF NONE macro, this event does not correspond to any application
            {
                if (gl_profile_tab[idx].gattc_cb) 
                {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param); //Call the callback function
                }
            }
        }
    } while (0);
}

static void ble_scan_resources_init(void)
{
    ble_queue = xQueueCreate( 60 , sizeof(uint8_t) * 6); 
}
void ble_scan_prepare(void)
{
    ble_scan_resources_init();
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
}
void ble_stack_init(void)
{
  esp_err_t ret;
  //ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) 
  {
    ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) 
  {
    ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_init();
  if (ret)
  {
    ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_enable();
  if (ret)
  {
    ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_ble_gap_register_callback(esp_gap_cb);
  if (ret)
  {
    ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
    return;
  }
  ret = esp_ble_gattc_register_callback(esp_gattc_cb);
  if(ret)
  {
    ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
    return;
  }
  ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
  if (ret)
  {
    ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
  }
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
  if (local_mtu_ret)
  {
    ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
  }
}
void ble_scan_start(void)
{
    esp_ble_gap_set_scan_params(&ble_scan_params);//set scan
    esp_ble_gap_start_scanning(3);         //Scan time of 3 seconds
}
void ble_stack_deinit(void)
{
    if(value != 0)esp_ble_gap_stop_scanning();
    esp_ble_gattc_app_unregister(gl_profile_tab[0].gattc_if);
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}