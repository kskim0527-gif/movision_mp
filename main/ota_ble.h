#pragma once
#include <stdbool.h>
#include <stdint.h>

// OTA BLE Service initialization
void ota_ble_init(void);

// Returns true if OTA service is ready for transfer
bool ota_ble_is_ready(void);

// Constants for protocol (maintained for compatibility)
#define OTA_BLE_APP_ID 1
