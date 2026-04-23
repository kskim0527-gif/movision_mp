#include "img_transfer.h"
#include "esp_log.h"
#include "fw_update.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>


static const char *TAG = "IMG_TRANSFER";

// Protocol constants
#define PROTOCOL_HEADER 0x19
#define PROTOCOL_ID_IMG 0x50
#define PROTOCOL_TAIL 0x2F

// Commands from APP
#define CMD_IMG_INFO 0x01
#define CMD_IMG_DATA 0x02
#define CMD_IMG_DELETE 0x05
#define CMD_IMG_AUTO_MODE 0x06
#define CMD_IMG_STATUS_REQ 0x07
#define CMD_IMG_STATUS_SYNC 0x08

// Commands from Device
#define CMD_IMG_INFO_RES 0x11
#define CMD_IMG_RES_SEQ 0x03
#define CMD_IMG_RES_CMP 0x04

typedef enum {
  IMG_STATE_IDLE = 0,
  IMG_STATE_RECEIVING,
  IMG_STATE_ERROR
} img_state_t;

typedef struct {
  img_state_t state;
  uint8_t image_seq;  // 1~5
  uint8_t image_type; // 0: Intro, 1: Background
  uint32_t total_size;
  uint32_t current_size;
  uint16_t last_block_seq;
  FILE *file;
  char file_path[128];
  uint8_t last_percent;
  uint32_t last_block_time; // Last block received time (ms)
} img_transfer_ctx_t;

static img_transfer_ctx_t s_ctx;
static char s_img_base_dir[64] = "/littlefs/Photo"; // Default base dir

// Extern from main.c
extern void hud_send_notify_bytes(const uint8_t *data, uint16_t len);
extern void log_ble_packet(const uint8_t *data, size_t len,
                             const char *prefix);
extern void update_img_transfer_ui(int percent, bool finished);
extern void load_image_from_sd(int direction); // Force load image
extern void update_album_option_from_ble(uint8_t mode);
extern void note_ble_activity(void);

// State variables from main.c
extern uint8_t s_album_option;
extern int s_current_image_index;
extern int s_image_count;
extern bool s_img_transfer_finished_flag;
extern bool s_img_transfer_active;
extern void start_reboot_task(void);

void img_transfer_init(void) {
  memset(&s_ctx, 0, sizeof(s_ctx));
  s_ctx.state = IMG_STATE_IDLE;

  // Check if lowercase /littlefs/photo exists, otherwise use /littlefs/Photo
  struct stat st;
  if (stat("/littlefs/photo", &st) == 0 && S_ISDIR(st.st_mode)) {
    strncpy(s_img_base_dir, "/littlefs/photo", sizeof(s_img_base_dir));
    ESP_LOGD(TAG, "Initialization: using existing /littlefs/photo");
  } else {
    strncpy(s_img_base_dir, "/littlefs/Photo", sizeof(s_img_base_dir));
    mkdir(s_img_base_dir, 0755);
    ESP_LOGI(TAG, "Initialization: /littlefs/Photo checked/created");
  }
}

static void send_response(uint8_t cmd, uint16_t seq, uint8_t error_code) {
  uint8_t resp[10];
  int len = 0;
  resp[0] = PROTOCOL_HEADER;
  resp[1] = PROTOCOL_ID_IMG;
  resp[2] = cmd;

  if (cmd == CMD_IMG_RES_SEQ) {
    resp[3] = 0x03; // D-Len: [SEQ:2][ERR:1] (1-byte per protocol doc)
    resp[4] = (seq >> 8) & 0xFF;
    resp[5] = seq & 0xFF;
    resp[6] = error_code;
    resp[7] = PROTOCOL_TAIL;
    len = 8;
    ESP_LOGI(TAG, "TX -> 19 50 03 03 %02X %02X %02X 2F", resp[4], resp[5], resp[6]);
  } else if (cmd == CMD_IMG_RES_CMP) {
    resp[3] = 0x01; // D-Len: [ERR:1] (1-byte)
    resp[4] = error_code;
    resp[5] = PROTOCOL_TAIL;
    len = 6;
  } else if (cmd == CMD_IMG_INFO_RES) {
    resp[3] = 0x02; // D-Len: [SBS:2]
    resp[4] = (uint8_t)(FW_UPDATE_BLOCK_SIZE >> 8);
    resp[5] = (uint8_t)(FW_UPDATE_BLOCK_SIZE & 0xFF);
    resp[6] = PROTOCOL_TAIL;
    len = 7;
  }

  if (len > 0) {
    hud_send_notify_bytes(resp, len);
  }
}

// 3.7.7 Status Report (0x07) - HUD -> APP
static void send_status_report(void) {
  uint8_t resp[7];
  resp[0] = PROTOCOL_HEADER;
  resp[1] = PROTOCOL_ID_IMG;
  resp[2] = CMD_IMG_STATUS_REQ;
  resp[3] = 0x02;                               // D-Len: [Mode:1][Index:1]
  resp[4] = (s_album_option == 0) ? 0x00 : 0x01; // 0: Auto, 1: Manual
  resp[5] = (uint8_t)(s_current_image_index + 1);
  resp[6] = PROTOCOL_TAIL;

  hud_send_notify_bytes(resp, sizeof(resp));
}

static void handle_img_info(const uint8_t *data, size_t len) {
  // Index: 0 1 2 3 4:Len, 5:Seq, 6~9:Size, 10:Type
  if (len < 12)
    return;

  s_ctx.image_seq = data[5];
  s_ctx.total_size = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9];
  s_ctx.image_type = data[10];
  s_ctx.current_size = 0;
  s_ctx.last_block_seq = 0xFFFF;
  s_ctx.last_percent = 0;
  s_ctx.last_block_time = (uint32_t)(esp_timer_get_time() / 1000); // 전송 시작 시 타임아웃 타이머 리셋

  if (s_ctx.image_type == 0) {
    snprintf(s_ctx.file_path, sizeof(s_ctx.file_path),
             "/littlefs/intro_new.tmp");
  } else {
    // 임시 파일 경로 설정
    snprintf(s_ctx.file_path, sizeof(s_ctx.file_path), "%s/album_%d.tmp",
             s_img_base_dir, s_ctx.image_seq);
             
    // 전송 시작 시 임시 파일 생성 (기존 임시 파일이 있다면 삭제)
    unlink(s_ctx.file_path);
  }

  if (s_ctx.file)
    fclose(s_ctx.file);
  s_ctx.file = fopen(s_ctx.file_path, "wb");
  if (s_ctx.file) {
    // [Performance] 파일 쓰기 성능 향상 및 플래시 부하 감소를 위해 4KB 버퍼 할당
    static char s_file_buf[4096];
    setvbuf(s_ctx.file, s_file_buf, _IOFBF, sizeof(s_file_buf));
    
    s_ctx.state = IMG_STATE_RECEIVING;
    s_img_transfer_active = true;           // 전송 중 상태 표시
    s_img_transfer_finished_flag = false;   // 이전 세션의 종료 처리(UI 숨기기 등)가 진행 중이라면 무효화
    note_ble_activity();                    // 재부팅 타이머 즉시 리셋
    
    ESP_LOGI(TAG, "Starting download: %s, size: %lu", s_ctx.file_path,
             (unsigned long)s_ctx.total_size);
    update_img_transfer_ui(0, false);
    
    // INFO 수신 응답 전송 (ID=0x50, CMD=0x11)
    send_response(CMD_IMG_INFO_RES, 0, 0);
    ESP_LOGI(TAG, "Sent INFO response (0x11)");
    s_ctx.last_block_time = (uint32_t)(esp_timer_get_time() / 1000);
  } else {
    s_ctx.state = IMG_STATE_ERROR;
    ESP_LOGE(TAG, "Fail to open: %s (errno=%d)", s_ctx.file_path, errno);
  }
}

static void handle_img_data(const uint8_t *data, size_t len) {
  if (len < 7)
    return;
  uint16_t seq = (data[5] << 8) | data[6];
  s_ctx.last_block_time = (uint32_t)(esp_timer_get_time() / 1000);

  // 이미지 블록 수신 시 BLE 활동 갱신 (reboot task의 비활성 타이머 방지)
  note_ble_activity();

  if (s_ctx.state != IMG_STATE_RECEIVING || !s_ctx.file) {
    // 아직 수신 상태 아님 — ACK 없이 무시
    return;
  }

  uint16_t dlen = (data[3] << 8) | data[4];
  uint32_t payload_len = dlen - 2;

  if (len < payload_len + 8)
    return;

  size_t written = fwrite(&data[7], 1, payload_len, s_ctx.file);
  if (written == payload_len) {
    // 중복 패킷 수신 시 용량이 이중으로 계산되는 것을 방지
    if (seq != s_ctx.last_block_seq || s_ctx.last_block_seq == 0xFFFF) {
        s_ctx.current_size += payload_len;
    }
    s_ctx.last_block_seq = seq;
    
    // 5블록 주기마다 ACK 전송
    if ((seq + 1) % 5 == 0) {
      ESP_LOGI(TAG, "Progress: %lu / %lu bytes (block %u)", 
               (unsigned long)s_ctx.current_size, (unsigned long)s_ctx.total_size, seq);
      send_response(CMD_IMG_RES_SEQ, seq, 0);
    }
    
    if (s_ctx.current_size >= s_ctx.total_size) {
      ESP_LOGI(TAG, "Progress: %lu / %lu bytes (block %u) [FINAL]", 
               (unsigned long)s_ctx.current_size, (unsigned long)s_ctx.total_size, seq);
    }

    int percent = (int)(s_ctx.current_size * 100 / s_ctx.total_size);
    if (percent > s_ctx.last_percent || percent == 100) {
      update_img_transfer_ui(percent, false);
      s_ctx.last_percent = (uint8_t)percent;
    }

    if (s_ctx.current_size >= s_ctx.total_size) {
      ESP_LOGI(TAG, "Image transfer finished: %lu bytes. Closing file...",
               (unsigned long)s_ctx.current_size);

      // 파일 기록 완료 보장 후 닫기
      fflush(s_ctx.file);
      fclose(s_ctx.file);
      s_ctx.file = NULL;

      if (s_ctx.image_type == 0) {
        unlink("/littlefs/intro.gif");
        rename("/littlefs/intro_new.tmp", "/littlefs/intro.gif");
      } else {
        // Read header to determine real extension
        FILE *f = fopen(s_ctx.file_path, "rb");
        char ext[10] = ".png"; // Default
        if (f) {
          uint8_t header[4] = {0};
          if (fread(header, 1, 4, f) == 4) {
            if (header[0] == 0xFF && header[1] == 0xD8 && header[2] == 0xFF) {
              strcpy(ext, ".jpg");
            } else if (header[0] == 0x89 && header[1] == 0x50 && header[2] == 0x4E && header[3] == 0x47) {
              strcpy(ext, ".png");
            } else if (header[0] == 'G' && header[1] == 'I' && header[2] == 'F') {
              strcpy(ext, ".gif");
            }
          }
          fclose(f);
        }
        
        char final_path[128];
        snprintf(final_path, sizeof(final_path), "%s/album_%d%s", s_img_base_dir, s_ctx.image_seq, ext);
        
        // 기존 파일이 열려있어서 rename이 실패하는 것을 방지하기 위해 먼저 삭제 시도
        char old_path[128];
        const char *extensions[] = {".png", ".jpg", ".jpeg", ".gif", ".bmp"};
        for (int i = 0; i < 5; i++) {
            snprintf(old_path, sizeof(old_path), "%s/album_%d%s", s_img_base_dir, s_ctx.image_seq, extensions[i]);
            unlink(old_path);
        }

        if (rename(s_ctx.file_path, final_path) == 0) {
            ESP_LOGI(TAG, "File saved with proper extension: %s", final_path);
        } else {
            ESP_LOGE(TAG, "Failed to rename %s to %s (errno=%d)", s_ctx.file_path, final_path, errno);
        }
    }

      // 상태 초기화
      s_ctx.state = IMG_STATE_IDLE;
      s_img_transfer_active = false; // 전송 종료
      
      // UI 제거는 메인 루프에서 비동기로 수행하여 BLE 태스크가 빨리 리턴되게 함
      s_img_transfer_finished_flag = true;
      
      // 앱에 완료 통보 (파일 시스템 안착을 위해 100ms 대기 후 전송)
      vTaskDelay(pdMS_TO_TICKS(100));
      send_response(CMD_IMG_RES_CMP, 0, 0);

      // 독립된 재부팅 전용 태스크 시작
      start_reboot_task();

      ESP_LOGI(TAG, "Image session ended OK (0x04 sent). Reboot pending...");
    }
  } else {
    // Write failed
    send_response(CMD_IMG_RES_SEQ, seq, 1);
  }
}

void process_img_command(const uint8_t *data, size_t len) {
  if (len < 5)
    return;
  if (data[0] != PROTOCOL_HEADER || data[1] != PROTOCOL_ID_IMG)
    return;

  uint8_t cmd = data[2];
  if (cmd == CMD_IMG_INFO)
    handle_img_info(data, len);
  else if (cmd == CMD_IMG_DATA)
    handle_img_data(data, len);
  else if (cmd == CMD_IMG_DELETE) {
    char path[128];
    snprintf(path, sizeof(path), "%s/album_%d.png", s_img_base_dir, data[5]);
    unlink(path);
    snprintf(path, sizeof(path), "%s/album_%d.jpg", s_img_base_dir, data[5]);
    unlink(path);
    snprintf(path, sizeof(path), "%s/album_%d.jpeg", s_img_base_dir, data[5]);
    unlink(path);
    snprintf(path, sizeof(path), "%s/album_%d.gif", s_img_base_dir, data[5]);
    unlink(path);
    snprintf(path, sizeof(path), "%s/album_%d.bmp", s_img_base_dir, data[5]);
    unlink(path);
    ESP_LOGI(TAG, "Deleted images for album_%d", data[5]);
  } else if (cmd == CMD_IMG_AUTO_MODE) {
    // 3.7.7 Auto Mode (0x06): 0: Auto, 1: Manual
    // 문서 규격(1바이트 길이)에 따라 데이터 위치는 data[4] (19 50 06 01 [MODE] 2F)
    if (len >= 6) {
      uint8_t mode = data[4];
      update_album_option_from_ble(mode);
      send_status_report(); // Sync state to APP
    }
  } else if (cmd == CMD_IMG_STATUS_REQ) {
    // 3.7.8 Status Request (0x07): APP asks current state
    send_status_report();
  } else if (cmd == CMD_IMG_STATUS_SYNC) {
    // 3.7.9 Status Sync (0x08): APP sets specific image index
    if (len >= 6) {
      uint8_t index = data[5]; // 1~5
      if (index >= 1 && index <= s_image_count) {
        s_album_option = index; // Fixed mode
        s_current_image_index = index - 1;
        ESP_LOGI(TAG, "Album Sync: index %d", index);
        load_image_from_sd(0); // Show it immediately
      }
      send_status_report();
    }
  }
}

void img_transfer_check_timeout(void) {
  if (s_ctx.state != IMG_STATE_RECEIVING || s_ctx.file == NULL)
    return;

  uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
  // [MOD] 워치독 시간을 1초에서 3초로 대폭 늘려 플래시 쓰기 지연 시간을 확보합니다.
  if (now - s_ctx.last_block_time > 3000) {
    ESP_LOGW(TAG, "Transfer stuck at current size %lu / %lu (seq %u). Free Heap: %lu. Sending watchdog ACK...", 
             (unsigned long)s_ctx.current_size, (unsigned long)s_ctx.total_size, s_ctx.last_block_seq,
             (unsigned long)esp_get_free_heap_size());
    // 유효한 블록을 한 번이라도 받았을 때만 동일 번호로 ACK 재전송
    if (s_ctx.last_block_seq != 0xFFFF) {
      send_response(CMD_IMG_RES_SEQ, s_ctx.last_block_seq, 0);
    }
    s_ctx.last_block_time = now; // 타이머 리셋
  }
}
