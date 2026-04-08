#include "img_transfer.h"
#include "esp_log.h"
#include "fw_update.h"
#include "sdkconfig.h"
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
} img_transfer_ctx_t;

static img_transfer_ctx_t s_ctx;
static char s_img_base_dir[64] = "/littlefs/Photo"; // Default base dir

// Extern from main.c
extern void hud_send_notify_bytes(const uint8_t *data, uint16_t len);
extern void save_packet_to_sdcard(const uint8_t *data, size_t len,
                                  const char *prefix);
extern void update_img_transfer_ui(int percent, bool finished);
extern void load_image_from_sd(int direction); // Force load image

// State variables from main.c
extern uint8_t s_album_option;
extern int s_current_image_index;
extern int s_image_count;
extern bool s_img_transfer_finished_flag;
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
    save_packet_to_sdcard(resp, len, "TX");
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
  save_packet_to_sdcard(resp, sizeof(resp), "TX");
}

static void handle_img_info(const uint8_t *data, size_t len) {
  // Index: 0 1 2 3 4:Len, 5:Seq, 6~9:Size, 10:Type
  if (len < 12)
    return;

  s_ctx.image_seq = data[5];
  s_ctx.total_size =
      (data[6] << 24) | (data[7] << 16) | (data[8] | (data[9]));
  s_ctx.total_size = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9];
  s_ctx.image_type = data[10];
  s_ctx.current_size = 0;
  s_ctx.last_block_seq = 0xFFFF;
  s_ctx.last_percent = 0;

  if (s_ctx.image_type == 0) {
    snprintf(s_ctx.file_path, sizeof(s_ctx.file_path),
             "/littlefs/intro_new.gif");
  } else {
    // Save background images as album_X.png in the discovered base dir
    snprintf(s_ctx.file_path, sizeof(s_ctx.file_path), "%s/album_%d.png",
             s_img_base_dir, s_ctx.image_seq);
  }

  if (s_ctx.file)
    fclose(s_ctx.file);
  s_ctx.file = fopen(s_ctx.file_path, "wb");

  if (s_ctx.file) {
    s_ctx.state = IMG_STATE_RECEIVING;
    ESP_LOGI(TAG, "Starting download: %s, size: %lu", s_ctx.file_path,
             (unsigned long)s_ctx.total_size);
    update_img_transfer_ui(0, false);
    send_response(CMD_IMG_INFO_RES, 0, 0);
  } else {
    s_ctx.state = IMG_STATE_ERROR;
    ESP_LOGE(TAG, "Fail to open: %s (errno=%d)", s_ctx.file_path, errno);
  }
}

static void handle_img_data(const uint8_t *data, size_t len) {
  if (len < 7)
    return;
  uint16_t seq = (data[5] << 8) | data[6];

  if (s_ctx.state != IMG_STATE_RECEIVING || !s_ctx.file) {
    // App may send data before info, must ACK for the app to continue
    send_response(CMD_IMG_RES_SEQ, seq, 0);
    return;
  }

  uint16_t dlen = (data[3] << 8) | data[4];
  uint32_t payload_len = dlen - 2;

  if (len < payload_len + 8)
    return;

  size_t written = fwrite(&data[7], 1, payload_len, s_ctx.file);
  if (written == payload_len) {
    s_ctx.current_size += payload_len;
    s_ctx.last_block_seq = seq;
    
    // 프로토콜 v1.11 시퀀스 다이어그램에 따라 지연 방지를 위해 데이터 블록별 수신 응답(0x5003) 제거
    // 단, 안정성을 위해 사용자 요청에 따라 5번째 블록마다 널 데이터(Dummy) 전송
    if (seq % 5 == 0) {
      static const uint8_t null_data[] = {0x19, 0x00, 0x00, 0x01, 0x00, 0x2F};
      hud_send_notify_bytes(null_data, sizeof(null_data));
      save_packet_to_sdcard(null_data, sizeof(null_data), "TX");
    }

    int percent = (int)(s_ctx.current_size * 100 / s_ctx.total_size);
    if (percent > s_ctx.last_percent || percent == 100) {
      update_img_transfer_ui(percent, false);
      s_ctx.last_percent = (uint8_t)percent;
    }

    if (s_ctx.current_size >= s_ctx.total_size) {
      ESP_LOGI(TAG, "Image transfer finished: %lu bytes. Closing file...",
               (unsigned long)s_ctx.current_size);

      // 파일 기록 완료 보장 (fsync 제거하여 속도 향상 및 콜백 블로킹 방지)
      fclose(s_ctx.file);
      s_ctx.file = NULL;

      if (s_ctx.image_type == 0) {
        unlink("/littlefs/intro.gif");
        rename("/littlefs/intro_new.gif", "/littlefs/intro.gif");
      }

      // 상태 초기화
      s_ctx.state = IMG_STATE_IDLE;
      
      // UI 제거는 메인 루프에서 비동기로 수행하여 BLE 태스크가 빨리 리턴되게 함
      s_img_transfer_finished_flag = true;
      
      // 앱에 완료 통보 (즉시 실행)
      send_response(CMD_IMG_RES_CMP, 0, 0);

      // 독립된 재부팅 전용 태스크 시작 (정확히 2초 보장)
      start_reboot_task();

      ESP_LOGI(TAG, "Image session ended OK (0x04 sent). Fast reboot pending...");
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
    ESP_LOGI(TAG, "Deleted: %s", path);
  } else if (cmd == CMD_IMG_AUTO_MODE) {
    // 3.7.7 Auto Mode (0x06): 0: Auto, 1: Manual
    if (len >= 6) {
      uint8_t mode = data[5];
      s_album_option = (mode == 0) ? 0 : (s_current_image_index + 1);
      ESP_LOGI(TAG, "Album Mode changed: %s", (mode == 0) ? "AUTO" : "MANUAL");
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
