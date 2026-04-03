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
    uint8_t last_percent;
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
        resp[3] = 0x03;  // D-Len: [RESULT:1][SBS:2]
        resp[4] = (error_code == 0) ? 1 : 0; // 1: OK, 0: Not OK
        resp[5] = (uint8_t)(FW_UPDATE_BLOCK_SIZE >> 8);
        resp[6] = (uint8_t)(FW_UPDATE_BLOCK_SIZE & 0xFF);
        resp[7] = PROTOCOL_TAIL;
        len = 8;
    } else if (cmd == CMD_FW_COMPLETE) {
        resp[3] = 0x01;  // D-Len
        resp[4] = error_code;
        resp[5] = PROTOCOL_TAIL;
        len = 6;
    }
    
    if (len > 0) {
        hud_send_notify_bytes(resp, (uint16_t)len);
        // Only log REQ and COMPLETE to SD to save time during data transfer
        if (cmd == CMD_FW_REQ || cmd == CMD_FW_COMPLETE) {
            save_packet_to_sdcard(resp, (size_t)len, "TX");
        }
    }
}

void fw_update_init(void) {
    memset(&s_fw_ctx, 0, sizeof(s_fw_ctx));
    s_fw_ctx.state = FW_STATE_IDLE;
    s_fw_ctx.update_handle = 0;
}

static void handle_fw_info(const uint8_t *data, size_t len) {
    // data structure in log: [19][4F][01][DLEN:2][VER:DLEN][SIZE:4][EXTRA:X][2F]
    // Based on user log: total 25 bytes, dlen 13, tail at 24, size at 15-18.
    // 24 - 15 = 9. So size starts 9 bytes before tail.
    if (len < 12) {
        ESP_LOGE(TAG, "FW Info too short: %u", (unsigned)len);
        return;
    }
    
    // Safety check: ensure we have a valid tail
    if (data[len-1] != 0x2F) {
        ESP_LOGW(TAG, "FW Info parsing without valid tail! (0x%02X)", data[len-1]);
    }

    // Extract size based on tail position (more robust for variable version lengths)
    // 0x4F 프로토콜은 2바이트 DLEN을 사용하므로 페이로드는 index 5부터 시작합니다.
    // [19][4F][01][DL_H][DL_L] [Payload...] [2F]
    // 페이로드의 마지막 4바이트가 전체 파일 크기(Big-Endian)입니다.
    if (len < 10) return; // 최소 길이 확인
    
    size_t tail_idx = len - 1;
    s_fw_ctx.total_size = (uint32_t)((data[tail_idx-4] << 24) | (data[tail_idx-3] << 16) | 
                                     (data[tail_idx-2] << 8) | data[tail_idx-1]);
    
    s_fw_ctx.current_size = 0;
    s_fw_ctx.last_seq = 0xFFFF;
    s_fw_ctx.last_percent = 0;
    
    s_fw_ctx.update_partition = esp_ota_get_next_update_partition(NULL);
    if (!s_fw_ctx.update_partition) {
        ESP_LOGE(TAG, "No update partition found");
        send_fw_response(CMD_FW_REQ, 0, 1); // Error
        return;
    }
    
    ESP_LOGI(TAG, "Starting FW update: size %lu (0x%08lX) from packet len %u", 
             (unsigned long)s_fw_ctx.total_size, (unsigned long)s_fw_ctx.total_size, (unsigned)len);
    
    update_ui_progress(0, "업데이트 준비 중 (플래시 삭제)...");
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
    
    // Print periodically or for the first few to avoid console spam
    if (seq % 100 == 0 || seq < 5) {
        ESP_LOGW("BLE_ONLY", "펌웨어 시퀀스 번호 = %02X %02X", data[5], data[6]);
    }
    
    uint32_t payload_len = dlen - 2;
    
    if (len < payload_len + 8) {
        ESP_LOGE(TAG, "FW Data packet corrupted: len=%u payload=%u", (unsigned)len, (unsigned)payload_len);
        return;
    }
    
    if (s_fw_ctx.state != FW_STATE_UPDATING || s_fw_ctx.update_handle == 0) {
        ESP_LOGW(TAG, "FW Data received but not in updating state");
        return;
    }
    
    esp_err_t err = esp_ota_write(s_fw_ctx.update_handle, &data[7], payload_len);
    if (err == ESP_OK) {
        s_fw_ctx.current_size += payload_len;
        s_fw_ctx.last_seq = seq;
        
        // 블럭별 저장 완료 로그 추가 (처음 5개 및 100개 단위)
        if (seq % 100 == 0 || seq < 5 || s_fw_ctx.current_size >= s_fw_ctx.total_size) {
            ESP_LOGW("BLE_ONLY", "[저장 완료] 펌웨어 시퀀스 번호 = %04X, 누적 크기 = %lu/%lu", 
                     seq, (unsigned long)s_fw_ctx.current_size, (unsigned long)s_fw_ctx.total_size);
        }
        
        int percent = (s_fw_ctx.current_size * 100) / s_fw_ctx.total_size;
        if (percent > s_fw_ctx.last_percent || percent == 100) {
            update_ui_progress(percent, "업데이트 진행 중..."); // 메시지를 한글로 유지
            s_fw_ctx.last_percent = (uint8_t)percent;
        }
        
        if (s_fw_ctx.current_size >= s_fw_ctx.total_size) {
            ESP_LOGI(TAG, "FW Download complete: %lu bytes. Finishing...", (unsigned long)s_fw_ctx.current_size);
            
            err = esp_ota_end(s_fw_ctx.update_handle);
            if (err == ESP_OK) {
                err = esp_ota_set_boot_partition(s_fw_ctx.update_partition);
                if (err == ESP_OK) {
                    s_fw_ctx.state = FW_STATE_FINISHED;
                    
                    // 완료 통보 전 사용자 요청으로 1초 대기 (기존 5초에서 단축)
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    send_fw_response(CMD_FW_COMPLETE, 0, 0); // 1. 완료 통보(0x05) 1회 송신
                    update_ui_progress(100, "업데이트 완료! 곧 재부팅합니다.");
                    
                    // 2. 0.5초 간격으로 시간 업데이트 요청(0x4E 0D) 2회 송신
                    static const uint8_t time_req[] = {0x19, 0x4E, 0x0D, 0x01, 0x00, 0x2F}; 
                    for (int i = 0; i < 2; i++) {
                        vTaskDelay(pdMS_TO_TICKS(500));
                        hud_send_notify_bytes(time_req, sizeof(time_req));
                        save_packet_to_sdcard(time_req, sizeof(time_req), "TX");
                    }
                    
                    update_ui_progress(100, "업데이트 완료! 곧 재부팅합니다.");
                    // 완료 통보가 PC에 도달할 시간을 충분히 줌 (1초 -> 3초)
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
