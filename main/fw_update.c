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
#define CMD_FW_DATA_ACK 0x04 // Update Data ACK (HUD -> App)
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
    } else if (cmd == CMD_FW_DATA_ACK) {
        resp[3] = 0x03;  // 데이터 길이: [시퀀스 번호:2][에러 코드:1]
        resp[4] = (uint8_t)(seq >> 8);
        resp[5] = (uint8_t)(seq & 0xFF);
        resp[6] = error_code; // 에러 코드 (0: No Error, 1: Retry, 2: Cancel)
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
        if (cmd == CMD_FW_REQ || cmd == CMD_FW_COMPLETE) {
            save_packet_to_sdcard(resp, (size_t)len, "TX");
        }
    }
}

// 널 데이터 킵얼라이브 태스크 (삭제 중 2초 간격 전송)
static bool s_keepalive_active = false;
static void fw_keepalive_task(void *pvParameters) {
    uint8_t null_pkt[] = {0x19, 0x00, 0x00, 0x01, 0x00, 0x2F};
    while (s_keepalive_active) {
        hud_send_notify_bytes(null_pkt, sizeof(null_pkt));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelete(NULL);
}

// 펌웨어 업데이트 시작 태스크 (플래시 삭제 및 준비)
static void fw_start_task(void *pvParameters) {
    ESP_LOGI(TAG, "FW Storage Erasing Task Starting (Size=%lu)...", (unsigned long)s_fw_ctx.total_size);
    update_ui_progress(0, "플래시 영역 삭제 중...");

    s_keepalive_active = true;
    // 스택 크기 증설 (2048 -> 4096)
    xTaskCreate(fw_keepalive_task, "fw_ka", 4096, NULL, 5, NULL);

    esp_err_t err = esp_ota_begin(s_fw_ctx.update_partition, s_fw_ctx.total_size, &s_fw_ctx.update_handle);
    
    s_keepalive_active = false;

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Flash Erase Complete. Device ready for data.");
        s_fw_ctx.state = FW_STATE_UPDATING;
        send_fw_response(CMD_FW_REQ, 0, 0); // OK (Device ready)
    } else {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        send_fw_response(CMD_FW_REQ, 0, 2);
        s_fw_ctx.state = FW_STATE_ERROR;
    }
    vTaskDelete(NULL);
}

// 펌웨어 업데이트 마무리 태스크 (해시 검사 및 재부팅)
static void fw_finish_task(void *pvParameters) {
    ESP_LOGI(TAG, "FW Finalizing Task Starting...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // 마지막 ACK 전송 시간을 벌어줌

    esp_err_t err = esp_ota_end(s_fw_ctx.update_handle);
    if (err == ESP_OK) {
        err = esp_ota_set_boot_partition(s_fw_ctx.update_partition);
        if (err == ESP_OK) {
            update_ui_progress(100, "업데이트 완료! 곧 재부팅합니다.");
            send_fw_response(CMD_FW_COMPLETE, 0, 0); 
            
            static const uint8_t time_req[] = {0x19, 0x4E, 0x0D, 0x01, 0x00, 0x2F}; 
            for (int i = 0; i < 2; i++) {
                vTaskDelay(pdMS_TO_TICKS(500));
                hud_send_notify_bytes(time_req, sizeof(time_req));
            }
            
            vTaskDelay(pdMS_TO_TICKS(1500));
            ESP_LOGI(TAG, "Rebooting...");
            esp_restart();
        } else {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
            send_fw_response(CMD_FW_COMPLETE, 0, 2);
        }
    } else {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        send_fw_response(CMD_FW_COMPLETE, 0, 1);
    }
    vTaskDelete(NULL);
}

void fw_update_init(void) {
    memset(&s_fw_ctx, 0, sizeof(s_fw_ctx));
    s_fw_ctx.state = FW_STATE_IDLE;
    s_fw_ctx.update_handle = 0;
}

static void handle_fw_info(const uint8_t *data, size_t len) {
    if (len < 16) return;
    // 패킷 규격: [Header:1][ID:1][CMD:1][LEN:2][Version:6][Size:4][Tail:1]
    // 파일 크기는 고정 위치(index 11~14)에서 파싱
    s_fw_ctx.total_size = (uint32_t)((data[11] << 24) | (data[12] << 16) | 
                                     (data[13] << 8) | data[14]);
    
    s_fw_ctx.current_size = 0;
    s_fw_ctx.last_seq = 0xFFFF;
    s_fw_ctx.last_percent = 0;
    
    s_fw_ctx.update_partition = esp_ota_get_next_update_partition(NULL);
    if (!s_fw_ctx.update_partition) {
        send_fw_response(CMD_FW_REQ, 0, 1);
        return;
    }
    
    s_fw_ctx.state = FW_STATE_PREPARING;
    
    // OTA 시작 시 고속 전송을 위해 연결 파라미터 업데이트 요청
    extern void hud_request_fast_conn(void);
    hud_request_fast_conn();

    // 준비 작업(플래시 삭제)을 전용 태스크로 분리 (BLE 블로킹 방지)
    xTaskCreate(fw_start_task, "fw_start", 4096, NULL, 5, NULL);
}

static void handle_fw_data(const uint8_t *data, size_t len) {
    if (len < 8) return;
    uint16_t dlen = (data[3] << 8) | data[4];
    uint16_t seq = (data[5] << 8) | data[6];
    uint32_t payload_len = dlen - 2;
    
    if (s_fw_ctx.state != FW_STATE_UPDATING || s_fw_ctx.update_handle == 0) return;
    
    esp_err_t err = esp_ota_write(s_fw_ctx.update_handle, &data[7], payload_len);
    if (err == ESP_OK) {
        s_fw_ctx.current_size += payload_len;
        s_fw_ctx.last_seq = seq;
        // 5블록 단위 그룹 ACK 송신 (또는 마지막 블록)
        bool is_last = (s_fw_ctx.current_size >= s_fw_ctx.total_size);
        if (seq % 5 == 4 || is_last) {
            send_fw_response(CMD_FW_DATA_ACK, seq, 0);
            
            // 저장 및 ACK 송신 로그 (윈도우 단위 또는 처음/끝만 출력)
            if (seq % 100 == 99 || seq < 5 || is_last) {
                ESP_LOGW("BLE_ONLY", "[저장 완료 ACK] Seq=%u, Size=%lu/%lu (Window x5 Mode)", 
                         seq, (unsigned long)s_fw_ctx.current_size, (unsigned long)s_fw_ctx.total_size);
            }
        }
        
        int percent = (s_fw_ctx.current_size * 100) / s_fw_ctx.total_size;
        if (percent > s_fw_ctx.last_percent || percent == 100) {
            update_ui_progress(percent, "업데이트 진행 중...");
            s_fw_ctx.last_percent = (uint8_t)percent;
        }
        
        if (s_fw_ctx.current_size >= s_fw_ctx.total_size) {
            s_fw_ctx.state = FW_STATE_FINISHED;
            xTaskCreate(fw_finish_task, "fw_finish", 4096, NULL, 5, NULL);
        }
    } else {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
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
            break;
    }
}
