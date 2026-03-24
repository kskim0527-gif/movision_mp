#include "img_transfer.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>

static const char *TAG = "IMG_TRANSFER";

// Protocol constants
#define PROTOCOL_HEADER 0x19
#define PROTOCOL_ID_IMG 0x50
#define PROTOCOL_TAIL   0x2F

// Commands from APP
#define CMD_IMG_INFO    0x01
#define CMD_IMG_DATA    0x02
#define CMD_IMG_DELETE  0x05

// Commands from Device
#define CMD_IMG_RES_SEQ 0x03
#define CMD_IMG_RES_CMP 0x04

typedef enum {
    IMG_STATE_IDLE = 0,
    IMG_STATE_RECEIVING,
    IMG_STATE_ERROR
} img_state_t;

typedef struct {
    img_state_t state;
    uint8_t image_seq;      // 1~5
    uint8_t image_type;     // 0: Intro, 1: Background
    uint32_t total_size;
    uint32_t current_size;
    uint16_t last_block_seq;
    FILE *file;
    char file_path[128];
} img_transfer_ctx_t;

static img_transfer_ctx_t s_ctx;
static char s_img_base_dir[64] = "/littlefs/Photo"; // Default base dir

// Extern from main.c
extern void hud_send_notify_bytes(const uint8_t *data, uint16_t len);
extern void save_packet_to_sdcard(const uint8_t *data, size_t len, const char *prefix);

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
        resp[3] = 0x03; 
        resp[4] = (seq >> 8) & 0xFF;
        resp[5] = seq & 0xFF;
        resp[6] = error_code;
        resp[7] = PROTOCOL_TAIL;
        len = 8;
    } else if (cmd == CMD_IMG_RES_CMP) {
        resp[3] = 0x01;
        resp[4] = error_code;
        resp[5] = PROTOCOL_TAIL;
        len = 6;
    }
    
    if (len > 0) {
        hud_send_notify_bytes(resp, len);
        save_packet_to_sdcard(resp, len, "TX");
    }
}

static void handle_img_info(const uint8_t *data, size_t len) {
    // Index: 0 1 2 3 4:Len, 5:Seq, 6~9:Size, 10:Type
    if (len < 12) return;
    
    s_ctx.image_seq = data[5];
    s_ctx.total_size = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9];
    s_ctx.image_type = data[10];
    s_ctx.current_size = 0;
    s_ctx.last_block_seq = 0xFFFF;
    
    if (s_ctx.image_type == 0) {
        snprintf(s_ctx.file_path, sizeof(s_ctx.file_path), "/littlefs/intro_new.gif");
    } else {
        // Save background images as album_X.png in the discovered base dir
        snprintf(s_ctx.file_path, sizeof(s_ctx.file_path), "%s/album_%d.png", s_img_base_dir, s_ctx.image_seq);
    }
    
    if (s_ctx.file) fclose(s_ctx.file);
    s_ctx.file = fopen(s_ctx.file_path, "wb");
    
    if (s_ctx.file) {
        s_ctx.state = IMG_STATE_RECEIVING;
        ESP_LOGI(TAG, "Starting download: %s, size: %lu", s_ctx.file_path, (unsigned long)s_ctx.total_size);
        // Removed ACK response for Image Info metadata as per v1.6 protocol update
    } else {
        s_ctx.state = IMG_STATE_ERROR;
        ESP_LOGE(TAG, "Fail to open: %s (errno=%d)", s_ctx.file_path, errno);
    }
}

static void handle_img_data(const uint8_t *data, size_t len) {
    if (len < 7) return;
    uint16_t seq = (data[5] << 8) | data[6];

    if (s_ctx.state != IMG_STATE_RECEIVING || !s_ctx.file) {
        // App may send data before info, must ACK for the app to continue
        send_response(CMD_IMG_RES_SEQ, seq, 0);
        return;
    }
    
    uint16_t dlen = (data[3] << 8) | data[4];
    uint32_t payload_len = dlen - 2;
    
    if (len < payload_len + 8) return;
    
    size_t written = fwrite(&data[7], 1, payload_len, s_ctx.file);
    if (written == payload_len) {
        s_ctx.current_size += payload_len;
        s_ctx.last_block_seq = seq;
        send_response(CMD_IMG_RES_SEQ, seq, 0); 
        
        if (s_ctx.current_size >= s_ctx.total_size) {
            ESP_LOGI(TAG, "Transfer complete: %lu bytes", (unsigned long)s_ctx.current_size);
            fclose(s_ctx.file); s_ctx.file = NULL;
            s_ctx.state = IMG_STATE_IDLE;
            if (s_ctx.image_type == 0) {
                unlink("/littlefs/intro.gif");
                rename("/littlefs/intro_new.gif", "/littlefs/intro.gif");
            }
            send_response(CMD_IMG_RES_CMP, 0, 0);
        }
    } else {
        send_response(CMD_IMG_RES_SEQ, seq, 1);
    }
}

void process_img_command(const uint8_t *data, size_t len) {
    if (len < 5) return;
    if (data[0] != PROTOCOL_HEADER || data[1] != PROTOCOL_ID_IMG) return;
    
    uint8_t cmd = data[2];
    if (cmd == CMD_IMG_INFO) handle_img_info(data, len);
    else if (cmd == CMD_IMG_DATA) handle_img_data(data, len);
    else if (cmd == CMD_IMG_DELETE) {
        char path[128];
        snprintf(path, sizeof(path), "%s/album_%d.png", s_img_base_dir, data[5]);
        unlink(path);
        ESP_LOGI(TAG, "Deleted: %s", path);
    }
}
