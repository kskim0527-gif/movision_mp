#include "ota_ble.h"
#include "fw_update.h"
#include "host/ble_hs.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "string.h"
#include <stdbool.h>
#include <stdint.h>

static const char *TAG = "OTA_BLE";

// UUIDs moved to main.c for unified registration

uint16_t s_ota_ctrl_handle;
uint16_t s_ota_data_handle;

static esp_ota_handle_t update_handle = 0;
static const esp_partition_t *update_partition = NULL;
static const esp_partition_t *storage_partition = NULL;
static bool ota_in_progress = false;
static bool storage_update_in_progress = false;
static uint32_t storage_write_offset = 0;
static uint32_t s_ota_total_size = 0;
static uint32_t s_ota_accumulated = 0;
static bool s_needs_erase = false;

extern void update_ui_progress(int percent, const char *status);
extern void note_ble_activity(void);

typedef struct {
  uint8_t data[600];
  uint16_t len;
} ota_data_msg_t;

static QueueHandle_t s_ota_queue = NULL;
static StaticQueue_t s_ota_queue_struct;
static uint8_t *s_ota_queue_storage = NULL;

static void restart_timer_callback(void *arg) {
  esp_restart();
}

static void ota_worker_task(void *param) {
  ota_data_msg_t msg;
  static uint8_t write_buf[4096];
  static uint32_t write_buf_idx = 0;

  while (1) {
    if (s_needs_erase) {
      if (ota_in_progress) {
        update_ui_progress(0, "Erasing App...");
        esp_ota_begin(update_partition, s_ota_total_size, &update_handle);
      } else if (storage_update_in_progress) {
        uint32_t total_erase_len = (s_ota_total_size + 4095) & (~4095);
        uint32_t erased_len = 0;
        while (erased_len < total_erase_len) {
          uint32_t to_erase = (total_erase_len - erased_len < 131072) ? (total_erase_len - erased_len) : 131072;
          esp_partition_erase_range(storage_partition, erased_len, to_erase);
          erased_len += to_erase;
          update_ui_progress(0, "Erasing...");
          vTaskDelay(pdMS_TO_TICKS(10));
        }
      }
      s_needs_erase = false;
      update_ui_progress(0, "Updating...");
      write_buf_idx = 0;
    }

    if (xQueueReceive(s_ota_queue, &msg, pdMS_TO_TICKS(100))) {
      if (ota_in_progress && update_handle != 0) {
        esp_ota_write(update_handle, msg.data, msg.len);
        s_ota_accumulated += msg.len;
      } else if (storage_update_in_progress && storage_partition != NULL) {
        uint32_t data_pos = 0;
        while (data_pos < msg.len) {
          uint32_t to_copy = (msg.len - data_pos < 4096 - write_buf_idx) ? (msg.len - data_pos) : (4096 - write_buf_idx);
          memcpy(&write_buf[write_buf_idx], &msg.data[data_pos], to_copy);
          write_buf_idx += to_copy;
          data_pos += to_copy;
          if (write_buf_idx >= 4096) {
            esp_partition_write(storage_partition, storage_write_offset, write_buf, 4096);
            storage_write_offset += 4096;
            write_buf_idx = 0;
          }
        }
        s_ota_accumulated = storage_write_offset + write_buf_idx;
      }

      if (s_ota_total_size > 0) {
        update_ui_progress((int)(s_ota_accumulated * 100 / s_ota_total_size), "Updating...");
      }
    }

    if (storage_update_in_progress && write_buf_idx > 0 && uxQueueMessagesWaiting(s_ota_queue) == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (uxQueueMessagesWaiting(s_ota_queue) == 0 && write_buf_idx > 0) {
            esp_partition_write(storage_partition, storage_write_offset, write_buf, write_buf_idx);
            storage_write_offset += write_buf_idx;
            write_buf_idx = 0;
        }
    }
  }
}

int ota_on_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    note_ble_activity();
    if (attr_handle == s_ota_ctrl_handle) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint8_t *val = ctxt->om->om_data;
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            uint8_t cmd = val[0];
            ESP_LOGI(TAG, "OTA CMD: %d", cmd);
            if (cmd == 1) { // BEGIN OTA
                update_partition = esp_ota_get_next_update_partition(NULL);
                if (update_partition) {
                    if (len >= 5) memcpy(&s_ota_total_size, &val[1], 4);
                    ota_in_progress = true;
                    storage_update_in_progress = false;
                    s_needs_erase = true;
                    s_ota_accumulated = 0;
                }
            } else if (cmd == 3) { // BEGIN STORAGE
                storage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
                if (storage_partition) {
                    storage_write_offset = 0;
                    storage_update_in_progress = true;
                    ota_in_progress = false;
                    if (len >= 5) memcpy(&s_ota_total_size, &val[1], 4);
                    s_needs_erase = true;
                    s_ota_accumulated = 0;
                }
            } else if (cmd == 2 || cmd == 4) { // END
                while (uxQueueMessagesWaiting(s_ota_queue) > 0) vTaskDelay(10);
                if (ota_in_progress) {
                    esp_ota_end(update_handle);
                    esp_ota_set_boot_partition(update_partition);
                }
                const esp_timer_create_args_t args = {.callback = restart_timer_callback, .name = "reboot"};
                esp_timer_handle_t timer;
                esp_timer_create(&args, &timer);
                esp_timer_start_once(timer, 3000000);
            }
        }
    } else if (attr_handle == s_ota_data_handle) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            ota_data_msg_t msg;
            msg.len = OS_MBUF_PKTLEN(ctxt->om);
            if (msg.len > 600) msg.len = 600;
            ble_hs_mbuf_to_flat(ctxt->om, msg.data, msg.len, NULL);
            xQueueSend(s_ota_queue, &msg, 0);
        }
    }
    return 0;
}

// s_ota_svc_defs moved to main.c

void ota_ble_init(void) {
  if (s_ota_queue == NULL) {
    s_ota_queue_storage = (uint8_t *)heap_caps_malloc(2000 * sizeof(ota_data_msg_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_ota_queue = xQueueCreateStatic(2000, sizeof(ota_data_msg_t), s_ota_queue_storage, &s_ota_queue_struct);
    xTaskCreatePinnedToCore(ota_worker_task, "ota_worker", 8192, NULL, 10, NULL, 1);
  }
}

bool ota_ble_is_ready(void) {
    return true; // Simplified for NimBLE unified start
}
