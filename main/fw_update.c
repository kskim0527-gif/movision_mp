#include "fw_update.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_system.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FW_UPDATE";

// Protocol constants
#define PROTOCOL_HEADER 0x19
#define PROTOCOL_ID_FW  0x4F
#define PROTOCOL_TAIL   0x2F

#define CMD_FW_INFO     0x01 // Update Info (App -> HUD)
#define CMD_FW_REQ      0x02 // Update Request (HUD -> App)
#define CMD_FW_DATA     0x03 // Update Data (App -> HUD)
#define CMD_FW_ACK      0x04 // Update ACK (HUD -> App)
#define CMD_FW_COMPLETE 0x05 // Update Complete (HUD -> App)

typedef enum {
    FW_STATE_IDLE = 0,
    FW_STATE_PREPARING,
    FW_STATE_UPDATING,
    FW_STATE_FINISHED,
    FW_STATE_ERROR
} fw_state_t;

static struct {
    fw_state_t state;
    uint32_t total_size;
    uint32_t current_size;
    uint16_t last_seq;
    esp_ota_handle_t update_handle;
    const esp_partition_t *update_partition;
} s_fw_ctx;

// Extern functions defined in main.c
extern void hud_send_notify_bytes(const uint8_t *data, uint16_t len);
extern void save_packet_to_sdcard(const uint8_t *data, size_t len, const char *prefix);
extern void update_ui_progress(int percent, const char *status);
extern void note_ble_activity(void);

static void send_fw_response(uint8_t cmd, uint16_t seq, uint8_t error_code) {
    uint8_t resp[12];
    int len = 0;
    
    resp[0] = PROTOCOL_HEADER;
    resp[1] = PROTOCOL_ID_FW;
    resp[2] = cmd;
    
    if (cmd == CMD_FW_REQ) {
        resp[3] = 0x00;  // D-Len Hi
        resp[4] = 0x01;  // D-Len Lo
        resp[5] = error_code;
        resp[6] = PROTOCOL_TAIL;
        len = 7;
    } else if (cmd == CMD_FW_ACK) {
        resp[3] = 0x00;  // D-Len Hi
        resp[4] = 0x03;  // D-Len Lo: [SEQ:2][ERR:1]
        resp[5] = (uint8_t)(seq >> 8);
        resp[6] = (uint8_t)(seq & 0xFF);
        resp[7] = error_code;
        resp[8] = PROTOCOL_TAIL;
        len = 9;
    } else if (cmd == CMD_FW_COMPLETE) {
        resp[3] = 0x00;  // D-Len Hi
        resp[4] = 0x01;  // D-Len Lo
        resp[5] = error_code;
        resp[6] = PROTOCOL_TAIL;
        len = 7;
    }
    
    if (len > 0) {
        hud_send_notify_bytes(resp, (uint16_t)len);
        save_packet_to_sdcard(resp, (size_t)len, "TX");
    }
}

void fw_update_init(void) {
    memset(&s_fw_ctx, 0, sizeof(s_fw_ctx));
    s_fw_ctx.state = FW_STATE_IDLE;
    s_fw_ctx.update_handle = 0;
}

static void handle_fw_info(const uint8_t *data, size_t len) {
    // [19] [4F] [01] [DLEN:2] [VER:6] [SIZE:4] [EXTRA:3] [2F]
    if (len < 16) {
        ESP_LOGE(TAG, "FW Info too short: %u", (unsigned)len);
        return;
    }
    
    uint16_t dlen = (data[3] << 8) | data[4];
    if (dlen < 10) {
        ESP_LOGE(TAG, "FW Info D-Len too short: %u", dlen);
        return;
    }
    
    // Size: big-endian 4 bytes starting at index 15 (after ID(1), CMD(1), LEN(2), VER(10))
    // Index: 0:19, 1:4F, 2:01, 3:LH, 4:LL, 5~14:VER, 15~18:SIZE
    s_fw_ctx.total_size = (uint32_t)((data[15] << 24) | (data[16] << 16) | (data[17] << 8) | data[18]);
    s_fw_ctx.current_size = 0;
    s_fw_ctx.last_seq = 0xFFFF;
    
    s_fw_ctx.update_partition = esp_ota_get_next_update_partition(NULL);
    if (!s_fw_ctx.update_partition) {
        ESP_LOGE(TAG, "No update partition found");
        send_fw_response(CMD_FW_REQ, 0, 1); // Error
        return;
    }
    
    ESP_LOGI(TAG, "Starting FW update: size %lu, partition %s", 
             (unsigned long)s_fw_ctx.total_size, s_fw_ctx.update_partition->label);
    
    update_ui_progress(0, "F/W Update...");
    s_fw_ctx.state = FW_STATE_PREPARING;
    
    // Begin OTA (will erase if size is known)
    esp_err_t err = esp_ota_begin(s_fw_ctx.update_partition, s_fw_ctx.total_size, &s_fw_ctx.update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        send_fw_response(CMD_FW_REQ, 0, 2); // Error
        s_fw_ctx.state = FW_STATE_ERROR;
        return;
    }
    
    s_fw_ctx.state = FW_STATE_UPDATING;
    
    // Respond with Update Request (device ready)
    send_fw_response(CMD_FW_REQ, 0, 0); // OK
}

static void handle_fw_data(const uint8_t *data, size_t len) {
    // 19 4F 03 [DLEN:2] [SEQ:2] [Payload...] 2F
    if (len < 8) return;
    
    uint16_t dlen = (data[3] << 8) | data[4];
    uint16_t seq = (data[5] << 8) | data[6];
    uint32_t payload_len = dlen - 2;
    
    if (len < payload_len + 8) {
        ESP_LOGE(TAG, "FW Data packet corrupted: len=%u payload=%u", (unsigned)len, (unsigned)payload_len);
        return;
    }
    
    if (s_fw_ctx.state != FW_STATE_UPDATING || s_fw_ctx.update_handle == 0) {
        ESP_LOGW(TAG, "FW Data received but not in updating state");
        send_fw_response(CMD_FW_ACK, seq, 1); // Not ready
        return;
    }
    
    esp_err_t err = esp_ota_write(s_fw_ctx.update_handle, &data[7], payload_len);
    if (err == ESP_OK) {
        s_fw_ctx.current_size += payload_len;
        s_fw_ctx.last_seq = seq;
        send_fw_response(CMD_FW_ACK, seq, 0); // OK
        
        int percent = (s_fw_ctx.current_size * 100) / s_fw_ctx.total_size;
        update_ui_progress(percent, NULL);
        
        if (s_fw_ctx.current_size >= s_fw_ctx.total_size) {
            ESP_LOGI(TAG, "FW Download complete: %lu bytes. Finishing...", (unsigned long)s_fw_ctx.current_size);
            
            err = esp_ota_end(s_fw_ctx.update_handle);
            if (err == ESP_OK) {
                err = esp_ota_set_boot_partition(s_fw_ctx.update_partition);
                if (err == ESP_OK) {
                    s_fw_ctx.state = FW_STATE_FINISHED;
                    send_fw_response(CMD_FW_COMPLETE, 0, 0); // Success
                    update_ui_progress(100, "Update Success! Rebooting...");
                    
                    // Delay reboot to allow ACK to be sent
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    esp_restart();
                } else {
                    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
                    send_fw_response(CMD_FW_COMPLETE, 0, 2);
                    s_fw_ctx.state = FW_STATE_ERROR;
                }
            } else {
                ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
                send_fw_response(CMD_FW_COMPLETE, 0, 1);
                s_fw_ctx.state = FW_STATE_ERROR;
            }
        }
    } else {
        ESP_LOGE(TAG, "esp_ota_write failed at offset %lu: %s", (unsigned long)s_fw_ctx.current_size, esp_err_to_name(err));
        send_fw_response(CMD_FW_ACK, seq, 2); // Error
    }
}

void process_fw_update_command(const uint8_t *data, size_t len) {
    if (len < 5) return;
    if (data[0] != PROTOCOL_HEADER || data[1] != PROTOCOL_ID_FW) return;
    
    uint8_t cmd = data[2];
    switch (cmd) {
        case CMD_FW_INFO:
            handle_fw_info(data, len);
            break;
        case CMD_FW_DATA:
            handle_fw_data(data, len);
            break;
        default:
            ESP_LOGW(TAG, "Unknown FW command: 0x%02X", cmd);
            break;
    }
}
