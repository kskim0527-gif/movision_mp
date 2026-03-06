#pragma once
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#include <stdbool.h>
#include <stdint.h>


#define OTA_BLE_APP_ID 1

void ota_ble_init(void);
void ota_ble_gatts_event_handler(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param);
