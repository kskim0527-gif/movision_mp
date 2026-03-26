#include "ota_ble.h"
#include "esp_gap_ble_api.h"
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

static uint8_t ota_service_uuid128[16] = {0xCE, 0x73, 0x96, 0xFC, 0x07, 0xCC,
                                          0x8C, 0xAB, 0x0B, 0x41, 0x37, 0x35,
                                          0x55, 0x36, 0x32, 0x80};
static uint8_t ota_ctrl_uuid128[16] = {0xCE, 0x73, 0x96, 0xFC, 0x07, 0xCC,
                                       0x8C, 0xAB, 0x0B, 0x41, 0x37, 0x35,
                                       0x56, 0x36, 0x32, 0x80};
static uint8_t ota_data_uuid128[16] = {0xCE, 0x73, 0x96, 0xFC, 0x07, 0xCC,
                                       0x8C, 0xAB, 0x0B, 0x41, 0x37, 0x35,
                                       0x57, 0x36, 0x32, 0x80};

static bool s_ota_service_ready = false;

static uint16_t s_ota_gatts_if = 0xFFFF;
static uint16_t s_service_handle, s_char_ctrl_handle, s_char_data_handle;

static esp_ota_handle_t update_handle = 0;
static const esp_partition_t *update_partition = NULL;
static const esp_partition_t *storage_partition = NULL;
static bool ota_in_progress = false;
static bool storage_update_in_progress = false;
static uint32_t storage_write_offset = 0;
static uint32_t s_ota_total_size = 0;
static uint32_t s_ota_accumulated = 0;

static uint32_t s_last_erased_sector_start = 0xFFFFFFFF;
static bool s_needs_erase = false;

extern void update_ui_progress(int percent, const char *status);
extern void note_ble_activity(void);

static void restart_timer_callback(void *arg) {
  ESP_LOGI(TAG, "Rebooting now...");
  esp_restart();
}

typedef struct {
  uint8_t data[512]; // Increased to support 509B blocks
  uint16_t len;
} ota_data_msg_t;

static QueueHandle_t s_ota_queue = NULL;
static StaticQueue_t s_ota_queue_struct;
static uint8_t *s_ota_queue_storage = NULL;

static void ota_worker_task(void *param) {
  ota_data_msg_t msg;
  static uint8_t write_buf[4096];
  static uint32_t write_buf_idx = 0;

  ESP_LOGI(TAG, "OTA Worker Task Ready (Turbo Mode)");
  while (1) {
    // 1. 데이터 수신 전이라도 지우기 명령이 있으면 즉시 수행
    if (s_needs_erase) {
      if (ota_in_progress) {
        ESP_LOGI(TAG, "Worker: Erasing App Partition...");
        update_ui_progress(0, "Erasing App...");
        esp_ota_begin(update_partition, s_ota_total_size, &update_handle);
      } else if (storage_update_in_progress) {
        ESP_LOGI(TAG, "Worker: Incremental Erasing Storage...");
        uint32_t total_erase_len = (s_ota_total_size + 4095) & (~4095);
        uint32_t erased_len = 0;
        uint32_t chunk_size = 128 * 1024;

        while (erased_len < total_erase_len) {
          uint32_t to_erase = (total_erase_len - erased_len < chunk_size)
                                  ? (total_erase_len - erased_len)
                                  : chunk_size;

          esp_partition_erase_range(storage_partition, erased_len, to_erase);
          erased_len += to_erase;
          update_ui_progress(0, "Erasing...");
          vTaskDelay(pdMS_TO_TICKS(10));
          if (erased_len % (256 * 1024) == 0 || erased_len == total_erase_len) {
            ESP_LOGI(TAG, "Erase Progress: %lu/%lu (%.1f%%)", erased_len,
                     total_erase_len,
                     (float)erased_len * 100 / total_erase_len);
          }
        }
        ESP_LOGI(TAG, "Worker: Storage Erase Complete.");
      }
      s_needs_erase = false;
      update_ui_progress(0, "Updating...");
      write_buf_idx = 0;
    }

    // 2. 데이터 수신 (100ms 타임아웃으로 지우기 플래그 체크 기회 제공)
    if (xQueueReceive(s_ota_queue, &msg, pdMS_TO_TICKS(100))) {
      if (ota_in_progress && update_handle != 0) {
        esp_ota_write(update_handle, msg.data, msg.len);
        s_ota_accumulated += msg.len;
      } else if (storage_update_in_progress && storage_partition != NULL) {
        uint32_t data_pos = 0;
        while (data_pos < msg.len) {
          uint32_t can_copy = 4096 - write_buf_idx;
          uint32_t to_copy =
              (msg.len - data_pos < can_copy) ? (msg.len - data_pos) : can_copy;
          memcpy(&write_buf[write_buf_idx], &msg.data[data_pos], to_copy);
          write_buf_idx += to_copy;
          data_pos += to_copy;
          if (write_buf_idx >= 4096) {
            esp_partition_write(storage_partition, storage_write_offset,
                                write_buf, 4096);
            storage_write_offset += 4096;
            write_buf_idx = 0;
          }
        }
        s_ota_accumulated = storage_write_offset + write_buf_idx;
      }

      if (s_ota_total_size > 0) {
        static uint32_t s_last_ui_kb = 0xFFFFFFFF;
        uint32_t curr_kb = s_ota_accumulated / 1024;
        uint32_t total_kb = s_ota_total_size / 1024;
        if (s_last_ui_kb == 0xFFFFFFFF || curr_kb >= s_last_ui_kb + 100 ||
            curr_kb == total_kb) {
          char status_buf[64];
          snprintf(status_buf, sizeof(status_buf), "%lu KB / %lu KB", curr_kb,
                   total_kb);
          update_ui_progress(
              (int)(curr_kb * 100 / (total_kb > 0 ? total_kb : 1)), status_buf);

          if (curr_kb % 512 == 0 || curr_kb == total_kb) {
            ESP_LOGI(TAG, "Download Progress: %lu / %lu KB (%.1f%%)", curr_kb,
                     total_kb, (float)curr_kb * 100 / total_kb);
          }
          s_last_ui_kb = curr_kb;
        }
      }
    }

    if (storage_update_in_progress && write_buf_idx > 0 &&
        uxQueueMessagesWaiting(s_ota_queue) == 0) {
      vTaskDelay(pdMS_TO_TICKS(50));
      if (uxQueueMessagesWaiting(s_ota_queue) == 0 && write_buf_idx > 0) {
        ESP_LOGD(TAG, "Flushing buffer: %lu bytes at offset %lu", write_buf_idx,
                 storage_write_offset);
        esp_partition_write(storage_partition, storage_write_offset, write_buf,
                            write_buf_idx);
        storage_write_offset += write_buf_idx;
        write_buf_idx = 0;
      }
    }
  }
}

void ota_ble_init(void) {
  if (s_ota_queue == NULL) {
    size_t queue_size = 2000;
    size_t item_size = sizeof(ota_data_msg_t);
    s_ota_queue_storage = (uint8_t *)heap_caps_malloc(
        queue_size * item_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_ota_queue_storage) {
      s_ota_queue = xQueueCreateStatic(
          queue_size, item_size, s_ota_queue_storage, &s_ota_queue_struct);
    }
    if (s_ota_queue) {
      xTaskCreatePinnedToCore(ota_worker_task, "ota_worker", 8192, NULL, 10,
                              NULL, 1);
      ESP_LOGI(TAG, "OTA Queue created in PSRAM (size: %zu), pinned to Core 1",
               queue_size);
    }
  }
  esp_ble_gatts_app_register(OTA_BLE_APP_ID);
}

void ota_ble_gatts_event_handler(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param) {
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.app_id == OTA_BLE_APP_ID) {
      s_ota_gatts_if = gatts_if;
      ESP_LOGI(TAG, "OTA App registered, if=%d", gatts_if);
    } else {
      return;
    }
  }

  if (s_ota_gatts_if == 0xFFFF || gatts_if != s_ota_gatts_if) {
    return;
  }

  switch (event) {
  case ESP_GATTS_REG_EVT: {
    esp_gatt_srvc_id_t service_id = {
        .is_primary = true, .id.inst_id = 0, .id.uuid.len = ESP_UUID_LEN_128};
    memcpy(service_id.id.uuid.uuid.uuid128, ota_service_uuid128, 16);
    esp_ble_gatts_create_service(gatts_if, &service_id, 8);
    break;
  }
  case ESP_GATTS_CREATE_EVT:
    s_service_handle = param->create.service_handle;
    esp_bt_uuid_t ctrl_uuid = {.len = ESP_UUID_LEN_128};
    memcpy(ctrl_uuid.uuid.uuid128, ota_ctrl_uuid128, 16);
    esp_ble_gatts_add_char(s_service_handle, &ctrl_uuid, ESP_GATT_PERM_WRITE,
                           ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
    break;
  case ESP_GATTS_ADD_CHAR_EVT:
    if (param->add_char.char_uuid.len == ESP_UUID_LEN_128 &&
        memcmp(param->add_char.char_uuid.uuid.uuid128, ota_ctrl_uuid128, 16) ==
            0) {
      s_char_ctrl_handle = param->add_char.attr_handle;
      esp_bt_uuid_t data_uuid = {.len = ESP_UUID_LEN_128};
      memcpy(data_uuid.uuid.uuid128, ota_data_uuid128, 16);
      esp_ble_gatts_add_char(s_service_handle, &data_uuid, ESP_GATT_PERM_WRITE,
                             ESP_GATT_CHAR_PROP_BIT_WRITE_NR, NULL, NULL);
    } else {
      s_char_data_handle = param->add_char.attr_handle;
      esp_ble_gatts_start_service(s_service_handle);
    }
    break;
  case ESP_GATTS_START_EVT:
    if (param->start.status == ESP_GATT_OK && param->start.service_handle == s_service_handle) {
      s_ota_service_ready = true;
      ESP_LOGI(TAG, "OTA Service started successfully and is now ready.");
    }
    break;
  case ESP_GATTS_WRITE_EVT:
    note_ble_activity();
    if (param->write.handle == s_char_ctrl_handle) {
      uint8_t cmd = param->write.value[0];
      ESP_LOGI(TAG, "OTA Control CMD received: %d", cmd);
      if (cmd == 1) { // BEGIN OTA
        update_partition = esp_ota_get_next_update_partition(NULL);
        if (update_partition) {
          if (param->write.len >= 5) {
            memcpy(&s_ota_total_size, &param->write.value[1], 4);
          } else {
            s_ota_total_size = 0;
          }
          ota_in_progress = true;
          storage_update_in_progress = false;
          s_needs_erase = true;
          s_ota_accumulated = 0;
          if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK,
                                        NULL);
          }
        }
      } else if (cmd == 3) { // BEGIN STORAGE
        if (ota_in_progress) {
          while (uxQueueMessagesWaiting(s_ota_queue) > 0)
            vTaskDelay(10);
          esp_ota_end(update_handle);
          esp_ota_set_boot_partition(update_partition);
          ota_in_progress = false;
        }
        storage_partition = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
        if (storage_partition) {
          storage_write_offset = 0;
          storage_update_in_progress = true;
          ota_in_progress = false;
          s_ota_accumulated = 0;
          if (param->write.len >= 5) {
            memcpy(&s_ota_total_size, &param->write.value[1], 4);
          } else {
            s_ota_total_size = 0;
          }
          s_needs_erase = true;
          if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK,
                                        NULL);
          }
        }
      } else if (cmd == 5) { // SYNC POINT
        if (param->write.need_rsp) {
          esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                      param->write.trans_id, ESP_GATT_OK, NULL);
        }
      } else if (cmd == 2 || cmd == 4) { // END UPDATE
        if (param->write.need_rsp) {
          esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                      param->write.trans_id, ESP_GATT_OK, NULL);
        }
        ESP_LOGI(TAG, "Update finished (cmd=%d). Finalizing...", cmd);
        while (uxQueueMessagesWaiting(s_ota_queue) > 0)
          vTaskDelay(10);
        vTaskDelay(pdMS_TO_TICKS(200));
        if (ota_in_progress) {
          esp_ota_end(update_handle);
          esp_ota_set_boot_partition(update_partition);
        } else if (storage_update_in_progress) {
          ESP_LOGI(TAG, "Storage update successful. Total: %lu bytes.",
                   storage_write_offset);
        }
        const esp_timer_create_args_t restart_timer_args = {
            .callback = &restart_timer_callback, .name = "restart_timer"};
        esp_timer_handle_t restart_timer;
        esp_timer_create(&restart_timer_args, &restart_timer);
        esp_timer_start_once(restart_timer, 3000000);
        return;
      }
    } else if (param->write.handle == s_char_data_handle) {
      ota_data_msg_t msg;
      msg.len = param->write.len;
      if (msg.len > 509)
        msg.len = 509;
      memcpy(msg.data, param->write.value, msg.len);
      if (xQueueSend(s_ota_queue, &msg, pdMS_TO_TICKS(1000)) != pdPASS) {
        ESP_LOGE(TAG, "OTA Queue Full! Data lost.");
      }
    }
    if (param->write.handle == s_char_ctrl_handle)
      return;
    if (param->write.need_rsp) {
      esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                  param->write.trans_id, ESP_GATT_OK, NULL);
    }
    break;
  case ESP_GATTS_CONNECT_EVT: {
    esp_ble_conn_update_params_t conn_params = {.bda = {0},
                                                .min_int = 0x06,
                                                .max_int = 0x10,
                                                .latency = 0,
                                                .timeout = 800};
    memcpy(conn_params.bda, param->connect.remote_bda, 6);
    esp_ble_gap_update_conn_params(&conn_params);
    esp_ble_gap_set_preferred_phy(
        param->connect.remote_bda, ESP_BLE_GAP_PHY_OPTIONS_NO_PREF,
        ESP_BLE_GAP_PHY_2M_PREF_MASK, ESP_BLE_GAP_PHY_2M_PREF_MASK, 0);
    esp_ble_gap_set_pkt_data_len(param->connect.remote_bda, 251);
  } break;
  case ESP_GATTS_DISCONNECT_EVT:
    storage_update_in_progress = false;
    ota_in_progress = false;
    break;
  default:
    break;
  }
}

bool ota_ble_is_ready(void) {
  return s_ota_service_ready;
}
