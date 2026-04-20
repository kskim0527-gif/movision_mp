#define LV_USE_GIF 1
// #define FACTORY_RESCUE_MODE // Factory(1MB) 빌드 시 이 주석을 해제하세요.
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h" // IWYU pragma: keep (must be included before task.h in ESP-IDF)
#include "freertos/queue.h"
#include "freertos/task.h"

// Some ESP-IDF headers require FreeRTOS.h before task.h.
// This file uses FreeRTOS macros (e.g. pdMS_TO_TICKS), so keep both includes.

#include "esp_app_desc.h" // For firmware version
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_random.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "driver/gpio.h"       // IWYU pragma: keep (GPIO_NUM_* pin macros)
#include "driver/i2c_master.h" // I2C driver for touch
#include "driver/spi_master.h" // IWYU pragma: keep (spi_bus_initialize/SPI2_HOST)
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_sh8601.h"

// Bluetooth (NimBLE)
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/ans/ble_svc_ans.h"
#include "services/bas/ble_svc_bas.h"
#include "services/hid/ble_svc_hid.h"
#include "store/config/ble_store_config.h"

// Forward declaration for NimBLE storage helper (contained in library but sometimes missing header)
void ble_store_config_init(void);

#include "esp_timer.h"
#include "esp_wifi.h"
#include "lvgl.h"
// Custom fonts (Dynamically loaded from LittleFS)
static lv_font_t *s_font_kopub_20 = NULL;
static lv_font_t *s_font_kopub_25 = NULL;

static lv_font_t *s_font_kopub_35 = NULL;
static lv_font_t *s_font_kopub_40 = NULL;
static lv_font_t *s_font_orb_100 = NULL;
static lv_font_t *s_font_orb_155 = NULL;
// static lv_font_t *s_font_gman_188 = NULL;
static lv_font_t *s_font_addr_30 = NULL; // 도로명 표시용 폰트 (font_addr_30)

#define SAFE_FONT(f)                                                           \
  ((f) ? (f) : (&lv_font_montserrat_14)) // LVGL default fallback
#define font_kopub_20 (*SAFE_FONT(s_font_kopub_20))
#define font_kopub_25 (*SAFE_FONT(s_font_kopub_25))

#define font_kopub_35 (*SAFE_FONT(s_font_kopub_35))
#define font_kopub_40 (*SAFE_FONT(s_font_kopub_40))
#define font_ORB_100 (*SAFE_FONT(s_font_orb_100))
#define font_ORB_155 (*SAFE_FONT(s_font_orb_155))
#define font_gman_188 (*SAFE_FONT(s_font_gman_188))
#define font_addr_30 (*SAFE_FONT(s_font_addr_30))
#if LV_USE_FS_POSIX
// POSIX file system driver header (will be available after build)
// Function declaration is in lv_fsdrv.h, but we can declare it here
extern void lv_fs_posix_init(void);
#endif

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

// LittleFS
#include "esp_littlefs.h"
#include "img_transfer.h"
#include "fw_update.h"
#include "version.h"

// SDMMC (SD Card)
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <math.h> // for cos, sin, M_PI

#include "extra/libs/qrcode/lv_qrcode.h"
#include "ota_ble.h"
#include "ota_sd.h"
#include "ota_wifi.h"

// [Diagnostic] Dump all NVS entries to verify persistence (bonding keys, etc.)
// NVS diagnostic functions removed

#include "src/extra/libs/gif/lv_gif.h"
#include "esp_phy_cert_test.h"

static const char *TAG = "BLE_ONLY";
static const char *SYS_MON_TAG = "SYS_MON";

// RF Certification Test State

// Global mutexes and semaphores
static SemaphoreHandle_t s_lvgl_mutex = NULL; // Mutex for LVGL thread safety
static SemaphoreHandle_t s_lcd_flush_sem =
    NULL; // Semaphore for LCD flush completion

#define LVGL_LOCK()                                                            \
  do {                                                                         \
    if (s_lvgl_mutex)                                                          \
      xSemaphoreTakeRecursive(s_lvgl_mutex, portMAX_DELAY);                    \
  } while (0)

#define LVGL_UNLOCK()                                                          \
  do {                                                                         \
    if (s_lvgl_mutex)                                                          \
      xSemaphoreGiveRecursive(s_lvgl_mutex);                                   \
  } while (0)

#include "lv_psram_mem.h"

// Custom memory allocators for LVGL (use PSRAM)
// These functions will be used by LVGL and PNG decoder
// Must be non-static for LVGL custom allocator
void *lvgl_psram_malloc(size_t size) {
  // Try PSRAM first, fallback to internal RAM if PSRAM is not available
  void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (ptr == NULL) {
    // Fallback to internal RAM if PSRAM allocation fails
    ptr = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  return ptr;
}

void lvgl_psram_free(void *ptr) {
  if (ptr != NULL) {
    heap_caps_free(ptr);
  }
}

void *lvgl_psram_realloc(void *ptr, size_t new_size) {
  if (ptr == NULL) {
    return lvgl_psram_malloc(new_size);
  }
  // Try PSRAM first for realloc
  void *new_ptr =
      heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (new_ptr == NULL) {
    // Fallback to internal RAM
    new_ptr =
        heap_caps_realloc(ptr, new_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  return new_ptr;
}

// Display modes
typedef enum {
  DISPLAY_MODE_BOOT = 0,
  DISPLAY_MODE_GUIDE,   // [Integrated] Standby + Speedometer + HUD
  DISPLAY_MODE_CLOCK,   // [Integrated] Clock 1 + Clock 2
  DISPLAY_MODE_ALBUM,
  DISPLAY_MODE_SETTING,
  DISPLAY_MODE_OTA,
  DISPLAY_MODE_MAX
} display_mode_t;

// Sub-modes for DISPLAY_MODE_GUIDE
typedef enum {
  GUIDE_SUB_STANDBY = 0,
  GUIDE_SUB_SPEEDOMETER,
  GUIDE_SUB_NAVI
} guide_sub_mode_t;

static display_mode_t s_current_mode = DISPLAY_MODE_BOOT;
static guide_sub_mode_t s_guide_sub_mode = GUIDE_SUB_STANDBY;
static uint8_t s_clock_option = 0; // 0: Clock 1, 1: Clock 2, 2: Auto
static display_mode_t s_last_base_mode = DISPLAY_MODE_GUIDE;
static bool s_is_manual_mode_switch = false;

int get_current_display_mode(void) { return (int)s_current_mode; }
// static bool s_virtual_drive_active = false; // Removed
static QueueHandle_t s_button_queue = NULL;
static QueueHandle_t s_image_update_queue =
    NULL; // Queue for image update requests (TBT)
static QueueHandle_t s_safety_update_queue =
    NULL; // Queue for Safety_DRV image update requests
static QueueHandle_t s_speed_update_queue =
    NULL; // Queue for speed update requests
static QueueHandle_t s_destination_update_queue =
    NULL; // Queue for destination update requests
static QueueHandle_t s_circle_update_queue =
    NULL; // Queue for circle update requests
static QueueHandle_t s_clear_display_queue =
    NULL; // Queue for clear display requests
static QueueHandle_t s_road_name_update_queue =
    NULL; // Queue for road name update requests (도로명 전송 0x0E)

// Task handles for monitoring
static TaskHandle_t s_lvgl_task_handle = NULL;
static TaskHandle_t s_button_task_handle = NULL;
static TaskHandle_t s_monitor_task_handle = NULL;
static TaskHandle_t s_virt_drive_task_handle = NULL;
static TaskHandle_t s_ble_tx_task_handle = NULL;
static TaskHandle_t s_lcd_task_handle = NULL;
static bool s_virt_drive_active = false;
static int s_secret_swipe_count = 0;
static uint32_t s_secret_swipe_start_tick = 0;

// Forward Declarations
void toggle_virtual_drive(bool enable);

// BOOT button (GPIO0) for mode switching
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// touch FreeRTOS config so include is considered "used" by some linters
// (keep FreeRTOS.h included before task.h as required by ESP-IDF)

// ---------------------------------------------------------------------------
// Device / UUIDs
// ---------------------------------------------------------------------------

#define DEVICE_NAME "MOVISION HUD1"

// Documented AMOLED DISPLAY UUIDs (16-bit UUIDs using Bluetooth base UUID)
// - Service UUID: 0xFFEA  (0000ffea-0000-1000-8000-00805f9b34fb)
// - Read UUID:    0xFFF1  (0000fff1-0000-1000-8000-00805f9b34fb)
// - Write UUID:   0xFFF2  (0000fff2-0000-1000-8000-00805f9b34fb)
#define HUD_SERVICE_UUID16 (0xFFEA)
#define HUD_CHAR_UUID_READ16 (0xFFF1)
#define HUD_CHAR_UUID_WRITE16 (0xFFF2)

#define NOTIFICATION_DESC_UUID 0x2902

// MCON AMOLED-like protocol (kept to match existing app expectations)
#define PROTOCOL_HEADER 0x19
#define PROTOCOL_ID 0x4D
#define PROTOCOL_TAIL 0x2F
// Document: TBT(Turn By Turn) uses Command=0x01, DataLen=0x05
#define CMD_TBT_DATA 0x01

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_connected = false;
static uint16_t s_mtu = 23;

// static bool s_peer_bda_valid = false;
// static uint8_t s_peer_bda[6];
// static int8_t s_rssi_value = -127;
// static bool s_rssi_read_once = false;
// HUD Service (FFF0) Handles
static uint16_t s_hud_char_ready_handle;
static uint16_t s_hud_char_write_handle;


static bool s_need_fast_conn = false;
static bool s_time_initialized = false;
static bool s_boot_clock_trigger = false;
static bool s_hud_notify_enabled = false;
static int s_time_req_count = 0;
static TickType_t s_last_time_req_tick = 0;

static bool s_hud_seen_first_cmd = false;
static bool s_app_communicated = false; // [New] Set to true when ANY app command received
static TickType_t s_boot_tick = 0;      // [New] System startup tick for 20s monitor

// ---------------------------------------------------------------------------
// Device Information Service (DIS) 0x180A (helps some apps accept the device as
// "real hardware")
// ---------------------------------------------------------------------------
// Device Information Handles (Managed by NimBLE unified callback)
 
 // Battery Handles
 static uint16_t s_batt_level_handle;
 static uint8_t s_batt_level = 100;


// HID over GATT (HOGP) - minimal service 0x1812
// ---------------------------------------------------------------------------

// HID Handles
static uint16_t s_hid_char_rpt_in_handle;
static uint16_t s_hid_cccd = 0;
// Device Information
static char s_device_serial[64] = "NO_SN";
static char s_app_reg_num[64] = "판매사에 문의하세요..";
// HID Reporting state (NimBLE)
static uint8_t s_hid_in_report[8] = {0x00}; // Reset to zeros
// BLE RX Reassembly Buffer (for packets split by MTU)
static uint8_t s_rx_buffer[2048] = {0};
static uint16_t s_rx_pos = 0;
static uint16_t s_rx_expected_len = 0;

// Thread-safe Command Queue for BLE
typedef struct {
    uint8_t data[256];
    uint16_t len;
} ble_cmd_t;
static QueueHandle_t s_ble_cmd_queue = NULL;
static TaskHandle_t s_ble_cmd_task_handle = NULL;

// Forward declaration
static void process_app_command(const uint8_t *data, size_t len);

// Dedicated task for processing BLE commands safely
static void ble_cmd_handler_task(void *pvParameters) {
    ble_cmd_t cmd;
    ESP_LOGI("BLE_CMD", "BLE Command Handler Task Started");
    while (1) {
        if (xQueueReceive(s_ble_cmd_queue, &cmd, portMAX_DELAY) == pdPASS) {
            // Process command in a safe context with LVGL protection
            LVGL_LOCK();
            process_app_command(cmd.data, cmd.len);
            LVGL_UNLOCK();
        }
    }
}

// ---------------------------------------------------------------------------
// Packet logging to SD card
// ---------------------------------------------------------------------------
#define LOG_FILE_BASE_DIR "/sdcard"
#define PACKET_LOG_BUFFER_SIZE (128 * 1024) // 128KB buffer in RAM

// static bool s_adv_data_ready = false;
// static bool s_scan_rsp_ready = false;
// static bool s_hud_service_started = false;
// static bool s_hid_service_started = false;
// static bool s_adv_active = false;
// static bool s_adv_starting = false;
// static bool s_waiting_mtu_logged = false;

// static uint32_t s_tx_count = 0;
static uint32_t s_tx_ok = 0;   // Exported for hud_send_notify_bytes
static uint32_t s_tx_fail = 0; // Exported for hud_send_notify_bytes
// static TickType_t s_last_status_tick = 0;
// static bool s_waiting_cccd_logged = false;

// static uint32_t s_hid_tx_count = 0;
// static uint32_t s_hid_tx_ok = 0;
// static uint32_t s_hid_tx_fail = 0;
// static TickType_t s_last_hid_tx_tick = 0;

// static bool s_feature_active = false;
// static TickType_t s_feature_active_tick = 0;

// Forward declarations
// static void try_start_advertising(void); // Removed: declared but never defined
static esp_err_t load_image_data_csv(void);
static void monitor_buffers_and_queues(void);
static void check_task_heartbeats(void);
static void monitor_system_health(void);
// static void update_heartbeat_ble(void); // Removed conflicting static
// declaration
static void buffer_queue_monitor_task(void *arg);
void hud_request_fast_conn(uint16_t conn_handle); 
static void ble_mp_advertise(void);

// ---------------------------------------------------------------------------
// LCD (SH8601, 466x466) - draw white border once after boot
// Peripheral settings mirrored from: C:\vscode\AI_DRV\main\main.c
// ---------------------------------------------------------------------------

#define LCD_HOST SPI2_HOST
#define LCD_H_RES (466)
#define LCD_V_RES (466)
#define LCD_BITS_PER_PIXEL (16) // RGB565

#include "board_config.h"
#define PIN_NUM_LCD_TE (-1) // TFT_TE (not used)

static esp_lcd_panel_handle_t s_lcd_panel = NULL;
static esp_lcd_panel_io_handle_t s_lcd_io_handle =
    NULL;                                // LCD IO handle for brightness control
static uint8_t s_brightness_level = 0;   // 0=Auto, 1=max, 5=min
uint8_t s_album_option = 0;       // 0=Auto(10s), 1=Manual(Static)
static uint8_t s_setting_page_index = 1; // 1 or 2

typedef struct {
  uint16_t sunrise_min;
  uint16_t sunset_min;
} sun_times_t;

// Monthly sunrise/sunset times in minutes from midnight (Seoul approx.)
static const sun_times_t s_monthly_sun_times[12] = {
    {466, 1064}, // 1월: 07:46, 17:44
    {430, 1106}, // 2월: 07:10, 18:26
    {395, 1130}, // 3월: 06:35, 18:50
    {358, 1148}, // 4월: 05:58, 19:08
    {324, 1175}, // 5월: 05:24, 19:35
    {310, 1196}, // 6월: 05:10, 19:56
    {323, 1194}, // 7월: 05:23, 19:54
    {348, 1164}, // 8월: 05:48, 19:24
    {374, 1118}, // 9월: 06:14, 18:38
    {400, 1073}, // 10월: 06:40, 17:53
    {432, 1041}, // 11월: 07:12, 17:21
    {460, 1033}  // 12월: 07:40, 17:13
};

#define LCD_COLOR_WHITE_RGB565 (0xFFFF)
#define LCD_COLOR_GREEN_RGB565 (0x07E0)
#define LCD_COLOR_ORANGE_RGB565 (0xFD20) // RGB565 "orange" (approx)

// ---------------------------------------------------------------------------
// LVGL integration for on-screen clock (HH:MM:SS)
// ---------------------------------------------------------------------------
#define LVGL_TICK_PERIOD_MS 1
#define LVGL_BUFFER_HEIGHT                                                     \
  466 // Full Screen Buffer for No Tearing (AI_DRV reference)

// Forward declarations
static void switch_display_mode(display_mode_t new_mode);
static void update_display_mode_ui(display_mode_t mode);
void update_heartbeat_lvgl(void);
void update_heartbeat_ble(void);
void update_heartbeat_button(void);
// Reduced chunk height from 40 to 10 to save Internal RAM for Wi-Fi OTA
#define LCD_FLUSH_CHUNK_HEIGHT 40

static lv_disp_draw_buf_t s_disp_draw_buf;
static lv_color_t *s_disp_buf1 = NULL;
static lv_color_t *s_disp_buf2 = NULL;
static lv_disp_drv_t s_disp_drv;
static lv_disp_t *s_disp = NULL;

// Screens for different modes
static lv_obj_t *s_boot_screen = NULL;     // Boot Mode Screen
static lv_obj_t *s_boot_time_label = NULL; // Boot Mode Digital Time

static lv_obj_t *s_boot_date_label = NULL; // Boot Mode Digital Date Header
static lv_obj_t *s_boot_reg_title_label = NULL; // App Registration Title
static lv_obj_t *s_boot_reg_val_label = NULL;   // App Registration Value
static lv_obj_t *s_boot_install_title_label = NULL;
static lv_obj_t *s_boot_install_sub_label = NULL;
static lv_obj_t *s_boot_install_qr = NULL;
static uint32_t s_boot_reg_timer_sec = 0;
static bool s_boot_reg_shown = false;
static lv_obj_t *s_hud_screen = NULL;
static lv_obj_t *s_speedometer_screen = NULL; // Speedometer Screen
static lv_obj_t *s_clock_screen = NULL;
static lv_obj_t *s_clock2_screen = NULL; // New Clock 2 Screen
static lv_obj_t *s_album_screen = NULL;
static lv_obj_t *s_setting_screen = NULL;       // Setting Screen
// static lv_obj_t *s_virtual_drive_screen = NULL; // Removed
static lv_obj_t *s_ota_screen = NULL;           // OTA Mode Screen

// Forward declarations for UI creation
static void create_boot_ui(void);

// Touch device handle
static lv_indev_t *s_touch_indev = NULL;

// Touch Pins (CST92xx I2C)
#define TOUCH_I2C_ADDR 0x5A
#define TOUCH_I2C_FREQ_HZ 100000

// CST92xx Register Definitions
#define CST92XX_READ_COMMAND 0xD000
#define CST92XX_ACK 0xAB
#define CST92XX_MAX_FINGER_NUM 2

// Forward declarations for touch
static esp_err_t init_touch(void);
static void touch_read_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static void create_clock_ui(void);
static void create_clock2_ui(void); // Forward decl
static void draw_analog_clock2(int hour, int minute,
                               int second); // Forward decl
static void create_album_ui(void);
static void create_setting_ui(void);        // Setting UI Forward Decl
void update_setting_ui_labels(void); // Helper Decl
void save_nvs_settings(void);
void update_album_option_from_ble(uint8_t mode);
static void create_ota_ui(void);            // Forward decl
static display_mode_t s_nvs_restored_mode = DISPLAY_MODE_GUIDE; // NVS에서 복구할 마지막 모드 보관용

// Labels for new modes
static lv_obj_t *s_clock_canvas = NULL;
// static lv_color_t *s_canvas_buf = NULL; // Canvas buffer
// static lv_obj_t *s_clock_date_label = NULL;

// Helper declarations
// Helper declarations
static void draw_analog_clock(int hour, int minute, int second);
static void set_lcd_brightness(uint8_t level, bool notify); // Forward declaration
void load_image_from_sd(int direction);
static void reset_album_to_default_image(void);
static void show_restart_msg(void);

// ==================== NVS Settings Storage ====================
void save_nvs_settings(void) {
  // [User Request] Do not save to NVS during Virtual Drive mode
  if (s_virt_drive_active) {
      return;
  }
  static uint8_t last_bright = 0xFF;
  static uint8_t last_album = 0xFF;
  static uint8_t last_clock = 0xFF;
  static uint8_t last_mode = 0xFF;

  // [User Request] NVS 저장 시도 로그 추가 및 비교 로직 유지하되 로그로 상태 확인
  if (last_bright == s_brightness_level && last_album == s_album_option &&
      last_clock == s_clock_option && last_mode == (uint8_t)s_current_mode) {
    // ESP_LOGD(TAG, "NVS Save skipped: same as last session");
    return; // No change
  }

  nvs_handle_t nvs;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs);
  if (err == ESP_OK) {
    nvs_set_u8(nvs, "bright_lvl", s_brightness_level);
    nvs_set_u8(nvs, "album_opt", s_album_option);
    nvs_set_u8(nvs, "clock_opt", s_clock_option);

    uint8_t save_val = 0;
    if (s_current_mode == DISPLAY_MODE_CLOCK) save_val = 1;
    else if (s_current_mode == DISPLAY_MODE_ALBUM) save_val = 2;
    else save_val = 0;

    nvs_set_u8(nvs, "boot_mode", save_val);
    err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        last_bright = s_brightness_level;
        last_album = s_album_option;
        last_clock = s_clock_option;
        last_mode = (uint8_t)s_current_mode;
        ESP_LOGW(TAG, "NVS Settings SAVED: Br=%d, Alb=%d, Clk=%d, Mode=%d", 
                 last_bright, last_album, last_clock, save_val);
    } else {
        ESP_LOGE(TAG, "NVS Commit FAILED: %s", esp_err_to_name(err));
    }
  } else {
    ESP_LOGE(TAG, "NVS Open FAILED: %s", esp_err_to_name(err));
  }
}

static void load_nvs_settings(void) {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs);
  if (err == ESP_OK) {
    uint8_t val;
    if (nvs_get_u8(nvs, "bright_lvl", &val) == ESP_OK && val <= 5)
      s_brightness_level = val;
    if (nvs_get_u8(nvs, "album_opt", &val) == ESP_OK && val <= 5)
      s_album_option = val;
    if (nvs_get_u8(nvs, "clock_opt", &val) == ESP_OK && val <= 2)
      s_clock_option = val;
    if (nvs_get_u8(nvs, "boot_mode", &val) == ESP_OK) {
      // [User Request] 0:기본(GUIDE), 1:시계, 2:앨범 매핑 적용
      if (val == 1) s_current_mode = DISPLAY_MODE_CLOCK;
      else if (val == 2) s_current_mode = DISPLAY_MODE_ALBUM;
      else s_current_mode = DISPLAY_MODE_GUIDE; // 0 또는 그 외는 모두 GUIDE로 복원

      s_nvs_restored_mode = s_current_mode; // 부팅 시 읽어온 모드 보관
      s_current_mode = DISPLAY_MODE_BOOT;   // 앱 연결 전까지는 로고 화면 강제

      // If we restored a guidance mode, ensure it starts in STANDBY sub-mode
      if (s_current_mode == DISPLAY_MODE_GUIDE) {
        s_guide_sub_mode = GUIDE_SUB_STANDBY;
      }
      ESP_LOGI(TAG, "NVS Loaded Boot Mode: %d (Old index) -> Consolidated Mode: %d", val, s_current_mode);
    }

    size_t len = sizeof(s_device_serial);
    if (nvs_get_str(nvs, "serial_num", s_device_serial, &len) != ESP_OK) {
      strcpy(s_device_serial, "NO_SN");
    }

    len = sizeof(s_app_reg_num);
    if (nvs_get_str(nvs, "app_reg_num", s_app_reg_num, &len) != ESP_OK) {
      strcpy(s_app_reg_num, "판매사에 문의하세요..");
    }

    nvs_close(nvs);
    ESP_LOGW(TAG, "################################################");
    ESP_LOGW(TAG, "NVS LOADED: Br=%d, Alb=%d, Clk=%d, SN=%s",
             s_brightness_level, s_album_option, s_clock_option, s_device_serial);
    ESP_LOGW(TAG, "################################################");
  } else {
    ESP_LOGE(TAG, "NVS empty or failed to open, using defaults");
    strcpy(s_device_serial, "NO_SN");
    strcpy(s_app_reg_num, "판매사에 문의하세요..");
  }
}
// =============================================================
static lv_obj_t *s_album_img = NULL;
static lv_obj_t *s_album_gif = NULL;
static lv_obj_t *s_album_guide_label = NULL;

// Speedometer mode objects
static lv_obj_t *s_speedometer_bg_img = NULL;
static lv_obj_t *s_speedometer_needle_line = NULL;
static lv_obj_t *s_speedometer_center_img = NULL;
static lv_point_t s_speedometer_needle_points[5];

static lv_obj_t *s_speedometer_speed_label = NULL;
static lv_obj_t *s_speedometer_unit_label = NULL;
static lv_obj_t *s_speedometer_safety_image = NULL;
static lv_obj_t *s_speedometer_safety_arc = NULL;
static lv_obj_t *s_speedometer_safety_value_label = NULL;
static lv_obj_t *s_speedometer_safety_unit_label = NULL;
static lv_obj_t *s_speedometer_avr_speed_title_label =
    NULL; // Speedometer Average Speed Title
static lv_obj_t *s_speedometer_avr_speed_value_label =
    NULL; // Speedometer Average Speed Value
static lv_obj_t *s_speedometer_avr_speed_unit_label =
    NULL; // Speedometer Average Speed Unit
static uint8_t s_speedometer_safety_tt_val = 0;
static lv_obj_t *s_speedometer_road_name_label = NULL; // Speedometer Road Name Label
static lv_obj_t *s_speedometer_road_name_sub_label = NULL; // Speedometer Road Name Sub-Label

static lv_obj_t *s_hyd_msg_label =
    NULL; // HYD TX 메시지 표시용 label (앱->ESP32)
static lv_obj_t *s_hud_tx_label =
    NULL; // HUD TX 메시지 표시용 label (ESP32->앱, FFF1 채널)
static lv_obj_t *s_hud_tx_label_large = NULL; // HUD 모드용 큰 HUD TX label
static lv_obj_t *s_hud_image = NULL;          // HUD 모드용 이미지 객체
static lv_obj_t *s_length_tbt_value_label =
    NULL; // TBT계산값 표시용 label (48pt)
static lv_obj_t *s_length_tbt_unit_label = NULL; // TBT단위 표시용 label (20pt)
static lv_obj_t *s_safety_image = NULL; // Safety_DRV 모드용 이미지 객체
static lv_obj_t *s_safety_length_value_label =
    NULL; // safety거리값 표시용 label (40pt)
static lv_obj_t *s_safety_length_unit_label =
    NULL; // safety단위 표시용 label (20pt)
// s_safety_border removed - using outer ring (s_circle_ring) instead
static lv_obj_t *s_normal_speed_value_label =
    NULL; // 일반속도 값 표시용 (100pt)
static lv_obj_t *s_normal_speed_unit_label =
    NULL;                                        // 일반속도 단위 표시용 (30pt)
static lv_obj_t *s_avr_speed_title_label = NULL; // 구간속도 타이틀
static lv_obj_t *s_avr_speed_value_label = NULL; // 구간속도 값 표시용 (40pt)
static lv_obj_t *s_avr_speed_unit_label = NULL;  // 구간속도 단위 표시용 (20pt)
static lv_obj_t *s_setting_bright_val_label = NULL;
static lv_obj_t *s_setting_btn_down = NULL;
static lv_obj_t *s_setting_btn_up = NULL;
static lv_obj_t *s_setting_album_val_label = NULL;
static lv_obj_t *s_setting_album_btn_down = NULL;
static lv_obj_t *s_setting_album_btn_up = NULL;
static lv_obj_t *s_setting_clock_val_label = NULL;
static lv_obj_t *s_setting_clock_btn_down = NULL;
static lv_obj_t *s_setting_clock_btn_up = NULL;
static lv_obj_t *s_setting_line_bright_dn = NULL;
static lv_obj_t *s_setting_line_bright_up = NULL;
static lv_obj_t *s_setting_line_album_dn = NULL;
static lv_obj_t *s_setting_line_album_up = NULL;
static lv_obj_t *s_setting_line_clock_dn = NULL;
static lv_obj_t *s_setting_line_clock_up = NULL;
static lv_obj_t *s_setting_circ_bright_dn = NULL;
static lv_obj_t *s_setting_circ_bright_up = NULL;
static lv_obj_t *s_setting_circ_album_dn = NULL;
static lv_obj_t *s_setting_circ_album_up = NULL;
static lv_obj_t *s_setting_circ_clock_dn = NULL;
static lv_obj_t *s_setting_circ_clock_up = NULL;
static lv_obj_t *s_setting_page1_obj = NULL;
static lv_obj_t *s_setting_page2_obj = NULL;
static lv_obj_t *s_setting_save_btn = NULL;
static lv_obj_t *s_setting_save_label = NULL;
static lv_obj_t *s_setting_title_label =
    NULL; // Single title that updates (SETUP 1/2)
// 설정 모드 밝기 숫자 라벨
static lv_obj_t *s_circle_ring = NULL;   // 외곽 링 객체 (GPS 상태 표시용)
static uint8_t s_last_gps_status = 0x01; // 0x00=Blue, 0x01=Green
// Time display labels (날짜: MM월dd일, 시간: HH:mm:ss)
// 각 부분별로 색상이 다르므로 개별 label 사용
static lv_obj_t *s_time_month_label = NULL;      // MM (하늘색)
static lv_obj_t *s_time_month_unit_label = NULL; // 월 (회색)
static lv_obj_t *s_time_day_label = NULL;        // dd (하늘색)
static lv_obj_t *s_time_day_unit_label = NULL;   // 일 (회색)
static lv_obj_t *s_time_hour_label = NULL;       // HH (하늘색)
static lv_obj_t *s_time_colon1_label = NULL;     // : (회색)
static lv_obj_t *s_time_minute_label = NULL;     // mm (하늘색)
static lv_obj_t *s_time_colon2_label = NULL;     // : (회색)
static lv_obj_t *s_time_second_label = NULL;     // ss (하늘색)
static lv_obj_t *s_dest_time_value_label =
    NULL; // 목적지남은시간값 표시용 label (30pt, 노랑색) - 시 숫자 또는 분 숫자
static lv_obj_t *s_dest_time_hour_unit_label =
    NULL; // 목적지남은시간 "시간" 단위 표시용 label (30pt, 회색)
static lv_obj_t *s_dest_time_minute_value_label =
    NULL; // 목적지남은시간 분 숫자 표시용 label (30pt, 노랑색)
static lv_obj_t *s_dest_time_unit_label =
    NULL; // 목적지남은시간단위 표시용 label (30pt, 회색) - "분"만
static lv_obj_t *s_dest_distance_value_label =
    NULL; // 목적지남은거리값 표시용 label (30pt, 하늘색)
static lv_obj_t *s_dest_distance_unit_label =
    NULL; // 목적지남은거리단위 표시용 label (20pt, 회색)
static lv_obj_t *s_dest_image = NULL;      // 목적지정보 이미지 (go_to.bmp)
static lv_obj_t *s_dest_label = NULL;      // "도착지까지" 라벨
static lv_obj_t *s_road_name_label = NULL; // 도로명 표시 라벨 (Sector 12)
static lv_obj_t *s_road_name_sub_label = NULL; // 도로명 서브 라벨 (Sector 12)
static lv_obj_t *s_rssi_label = NULL; // 블루투스 RSSI 표시용 label (20pt, 흰색)
static lv_obj_t *s_speed_mark_value_label =
    NULL; // speed_mark 속도 값 표시용 (155pt, 흰색)
static lv_obj_t *s_speed_mark_unit_label =
    NULL; // speed_mark KM/H 단위 표시용 (35pt, 흰색)
static lv_obj_t *s_black_screen_overlay =
    NULL; // 전체 화면 검정색 오버레이 (화면 지우기용)
static esp_timer_handle_t s_time_update_timer = NULL; // 시간 업데이트 타이머
static bool s_logging_enabled = false; // 시간 데이터 수신 후 로깅 시작 플래그
#define MAX_IMAGE_FILES 200
static char s_image_files[MAX_IMAGE_FILES][280] EXT_RAM_BSS_ATTR; // Image paths
int s_image_count = 0;
int s_current_image_index = 0;
bool s_img_transfer_finished_flag = false; // 이미지 전송 완료 비동기 처리용
bool s_img_transfer_active = false;        // 이미지 전송 중 여부
bool s_fw_update_active = false;           // 펌웨어 업데이트 중 여부
static int s_auto_clock_offset = -1;  // 부팅 시 결정된 앨범/시계 로테이션 오프셋
static int s_album_auto_timer = 0;

static lv_obj_t *s_intro_image = NULL; // 부팅 인트로 이미지 객체

// Image data mapping from CSV file
// CSV format: start,ID,commend,data
// length,data1,data2,data3,data4,data5,end,image,sector,size
typedef struct {
  uint8_t start;           // CSV: start (0x19)
  uint8_t id;              // CSV: ID (0x4D)
  uint8_t commend;         // CSV: commend (0x01)
  uint8_t data_length;     // CSV: data length (0x05)
  uint8_t data1;           // CSV: data1
  uint8_t data2;           // CSV: data2
  char image_filename[64]; // CSV: image (e.g., "arrow_0.bmp")
  uint8_t sector;          // CSV: sector (LCD 위치)
  char size[32];           // CSV: size (e.g., "150pt x 150pt")
} image_data_entry_t;

// Safety_DRV data mapping from CSV file
// CSV format:
// start,ID,commend,data_length,data1,data2,data3,data4,data5,data6,end,image,sector
typedef struct {
  uint8_t start;           // CSV: start (0x19)
  uint8_t id;              // CSV: ID (0x4D)
  uint8_t commend;         // CSV: commend (0x02)
  uint8_t data_length;     // CSV: data length (0x06)
  uint8_t data1;           // CSV: data1
  char image_filename[64]; // CSV: image (e.g., "camera_tt.bmp")
  uint8_t sector;          // CSV: sector (LCD 위치)
  char size[32];           // CSV: size (e.g., "150pt x 150pt")
} safety_data_entry_t;

static image_data_entry_t *s_image_data_entries = NULL;
static size_t s_image_data_count = 0;

static safety_data_entry_t *s_safety_data_entries = NULL;

// Road Name update request structure
typedef struct {
  char road_name[128];
} road_name_update_request_t;
static size_t s_safety_data_count = 0;

// Current BLE data values for image matching (data2, data3, data4 from HUD TX
// packet)
// static uint8_t s_current_data2 = 0; // HUD TX packet data[5]
// static uint8_t s_current_data3 = 0; // HUD TX packet data[6]
// static uint8_t s_current_data4 = 0; // HUD TX packet data[7]

// Current displayed image path (to avoid reloading same image)
static char s_current_image_path[128] = {0};        // TBT 방향표시 이미지 경로
static char s_current_safety_image_path[128] = {0}; // Safety_DRV 이미지 경로
static char s_current_dest_image_path[128] = {
    0}; // 목적지정보 이미지 경로 (go_to.bmp)

// Safety_DRV ring flashing variables
static lv_timer_t *s_safety_ring_timer = NULL;
static int s_safety_ring_flash_count = 0;

// Image update request structure
typedef struct {
  uint8_t start;
  uint8_t id;
  uint8_t commend;
  uint8_t data_length;
  uint8_t data1;
  uint8_t data2;
  bool force_update;
  // TBT distance data (optional, only used for TBT direction display)
  bool has_distance; // true if distance data is present
  uint8_t data3;     // Distance high byte
  uint8_t data4;     // Distance middle byte
  uint8_t data5;     // Distance low byte
} image_update_request_t;

// Safety_DRV image update request structure
typedef struct {
  uint8_t start;
  uint8_t id;
  uint8_t commend;
  uint8_t data_length;
  uint8_t data1;
  uint8_t data2;
  uint8_t data3; // Ring color (1=red, 0=green)
  uint8_t data4; // Distance high byte
  uint8_t data5; // Distance middle byte
  uint8_t data6; // Distance low byte
} safety_update_request_t;

// Speed update request structure
typedef struct {
  uint8_t start;
  uint8_t id;
  uint8_t commend;
  uint8_t data_length;
  uint8_t data1;
  uint8_t data2; // Speed value
} speed_update_request_t;

// Destination update request structure (for queue-based communication)
typedef struct {
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
  uint8_t data4;
  uint8_t data5;
} destination_update_request_t;

// Circle update request structure (for queue-based communication)
typedef struct {
  uint8_t start;
  uint8_t id;
  uint8_t commend;
  uint8_t data_length;
  uint8_t data1;
} circle_update_request_t;

// Clear display request structure (for queue-based communication)
typedef struct {
  uint8_t data1;
} clear_display_request_t;

// Last image update request (for duplicate filtering)
static SemaphoreHandle_t s_pkt_log_mutex = NULL;
static image_update_request_t s_last_image_request = {0};
static bool s_last_image_request_valid = false;

// Last safety update request (for duplicate filtering)
static safety_update_request_t s_last_safety_request = {0};
static bool s_last_safety_request_valid = false;

// Last speed update request (for duplicate filtering)
static speed_update_request_t s_last_speed_request = {0};
static bool s_last_speed_request_valid = false;

// Last destination update request (for duplicate filtering)
static destination_update_request_t s_last_destination_request = {0};
static bool s_last_destination_request_valid = false;

// Last circle update request (for duplicate filtering)
static circle_update_request_t s_last_circle_request = {0};
static bool s_last_circle_request_valid = false;

// Last clear display request (for duplicate filtering)
static clear_display_request_t s_last_clear_display_request = {0};
static bool s_last_clear_display_request_valid = false;

// LittleFS mount status (declared before functions that use it)
static bool s_littlefs_mounted = false;

void log_ble_packet(const uint8_t *data, size_t len, const char *prefix);
// Forward declaration for functions
static void display_tbt_direction(uint8_t start, uint8_t id, uint8_t commend,
                                  uint8_t data_length, uint8_t data1,
                                  uint8_t data2, uint8_t data3, uint8_t data4,
                                  uint8_t data5);
static void request_image_update(uint8_t start, uint8_t id, uint8_t commend,
                                 uint8_t data_length, uint8_t data1,
                                 uint8_t data2, bool force_update,
                                 bool has_distance, uint8_t data3,
                                 uint8_t data4, uint8_t data5);
static esp_err_t load_image_data_csv(void);
static esp_err_t load_safety_data_csv(void);
static void scan_intro_images(void);
// Safety_DRV functions
static const safety_data_entry_t *
find_safety_image_entry(uint8_t start, uint8_t id, uint8_t commend,
                        uint8_t data_length, uint8_t data1);
static void request_safety_update(uint8_t start, uint8_t id, uint8_t commend,
                                  uint8_t data_length, uint8_t data1,
                                  uint8_t data2, uint8_t data3, uint8_t data4,
                                  uint8_t data5, uint8_t data6);
static void update_safety_image_for_data(const safety_data_entry_t *entry,
                                         uint8_t data2, uint8_t data3,
                                         uint8_t data4, uint8_t data5,
                                         uint8_t data6);
static void safety_drive(uint8_t start, uint8_t id, uint8_t commend,
                         uint8_t data_length, uint8_t data1, uint8_t data2,
                         uint8_t data3, uint8_t data4, uint8_t data5,
                         uint8_t data6);
// Circle drawing function
// static void circle_dwg(uint8_t start, uint8_t id, uint8_t commend,
//                        uint8_t data_length, uint8_t data1);
static void request_circle_update(uint8_t start, uint8_t id, uint8_t commend,
                                  uint8_t data_length, uint8_t data1);
static void update_circle_display(uint8_t start, uint8_t id, uint8_t commend,
                                  uint8_t data_length, uint8_t data1);
static void safety_ring_timer_cb(lv_timer_t *timer); // Safety_DRV 링 점멸 콜백
// Speed mark function
static void speed_mark(uint8_t start, uint8_t id, uint8_t commend,
                       uint8_t data_length, uint8_t data1, uint8_t data2);
static void request_speed_update(uint8_t start, uint8_t id, uint8_t commend,
                                 uint8_t data_length, uint8_t data1,
                                 uint8_t data2);
static void avr_speed_mark(uint8_t start, uint8_t id, uint8_t commend,
                           uint8_t data_length, uint8_t data1, uint8_t data2);
static void update_speed_label(uint8_t data1, uint8_t speed);
static void create_speedometer_ui(void);
// SD card drive callbacks for LVGL
static void *lv_fs_open_sd(lv_fs_drv_t *drv, const char *path,
                           lv_fs_mode_t mode) {
  int flags = 0;
  if (mode == LV_FS_MODE_RD)
    flags = O_RDONLY;
  else if (mode == LV_FS_MODE_WR)
    flags = O_WRONLY | O_CREAT;
  else if (mode == (LV_FS_MODE_RD | LV_FS_MODE_WR))
    flags = O_RDWR | O_CREAT;

  char full_path[256];
  const char *prefixes[] = {"/littlefs/", "/littlefs/flash_data/", "/littlefs/Flash_Data/"};
  bool found = false;

  if (strncmp(path, "/littlefs/", 10) == 0) {
    const char *remainder = path + 10;
    for (int i = 0; i < 3; i++) {
      snprintf(full_path, sizeof(full_path), "%s%s", prefixes[i], remainder);
      struct stat st;
      if (stat(full_path, &st) == 0) {
        found = true;
        break;
      }
    }
  }

  if (!found) {
    if (path[0] == '/') {
      snprintf(full_path, sizeof(full_path), "%s", path);
    } else {
      snprintf(full_path, sizeof(full_path), "/%s", path);
    }
  }

  int f = open(full_path, flags, 0666);
  if (f < 0) {
    return NULL;
  }

  return (void *)(uintptr_t)(f + 1);
}
static lv_fs_res_t lv_fs_close_sd(lv_fs_drv_t *drv, void *file_p) {
  int f = (int)(uintptr_t)file_p - 1;
  close(f);
  return LV_FS_RES_OK;
}
static lv_fs_res_t lv_fs_read_sd(lv_fs_drv_t *drv, void *file_p, void *buf,
                                 uint32_t btr, uint32_t *br) {
  int f = (int)(uintptr_t)file_p - 1;
  ssize_t res = read(f, buf, btr);
  if (res < 0)
    return LV_FS_RES_UNKNOWN;
  *br = (uint32_t)res;
  return LV_FS_RES_OK;
}
static lv_fs_res_t lv_fs_seek_sd(lv_fs_drv_t *drv, void *file_p, uint32_t pos,
                                 lv_fs_whence_t whence) {
  int f = (int)(uintptr_t)file_p - 1;
  off_t res = lseek(f, pos, whence);
  return (res < 0) ? LV_FS_RES_FS_ERR : LV_FS_RES_OK;
}
static lv_fs_res_t lv_fs_tell_sd(lv_fs_drv_t *drv, void *file_p,
                                 uint32_t *pos_p) {
  int f = (int)(uintptr_t)file_p - 1;
  off_t res = lseek(f, 0, SEEK_CUR);
  if (res < 0)
    return LV_FS_RES_FS_ERR;
  *pos_p = (uint32_t)res;
  return LV_FS_RES_OK;
}
// Time set function
static void time_set(const uint8_t *data, size_t data_len);
static void update_time_display(void);
// 목적지정보 함수
// static void destination_info(uint8_t data1,
// uint8_t data2, uint8_t data3,
//                              uint8_t data4,
//                              uint8_t
//                              data5);
static void request_destination_update(uint8_t data1, uint8_t data2,
                                       uint8_t data3, uint8_t data4,
                                       uint8_t data5);
static void update_destination_info(uint8_t data1, uint8_t data2, uint8_t data3,
                                    uint8_t data4, uint8_t data5);
// 화면 지우기 관련 함수
static void hide_black_screen_overlay(void);
static void ensure_circle_ring_created(void);
// static void clear_display(uint8_t data1);
static void request_clear_display(uint8_t data1);
static void update_clear_display(uint8_t data1);

// --- Packet Processing Logic (Extracted for re-use in Virtual Drive) ---
static void update_auto_brightness(bool force);
void hud_send_notify_bytes(const uint8_t *data, uint16_t len);
static void request_road_name_update(const char *road_name) {
  if (s_road_name_update_queue == NULL)
    return;

  road_name_update_request_t req;
  memset(req.road_name, 0, sizeof(req.road_name));
  strncpy(req.road_name, road_name, sizeof(req.road_name) - 1);

  xQueueSend(s_road_name_update_queue, &req, 0);
}

static void process_app_command(const uint8_t *data, size_t len) {
  if (len < 5)
    return;

  uint8_t start = data[0];       // 0x19
  uint8_t id = data[1];          // 0x4D, 0x4F, 0x50
  uint8_t commend = data[2];     // cmd
  uint8_t data_length = data[3]; // dlen

  // Basic validation
  if (start != 0x19)
    return;

  if (id == 0x50) {
    s_app_communicated = true;
    s_hud_seen_first_cmd = true;
    process_img_command(data, len);
    return;
  }
  if (id == 0x4F) {
    s_app_communicated = true;
    s_hud_seen_first_cmd = true;
    process_fw_update_command(data, len);
    return;
  }

  // [User Request] 안내 모드(GUIDE)가 아니더라도 백그라운드에서 데이터를 지속적으로 갱신하여 
  // 사용자가 스와이프했을 때 최신 정보를 볼 수 있도록 전역 필터를 제거합니다.

  if (id != 0x4D)
    return;
  
  // 첫 유효 명령 수신 표시 (인트로 폴링 루프 조기 종료 및 부팅 타이머 중지용)
  s_app_communicated = true; 
  if (!s_hud_seen_first_cmd) {
    s_hud_seen_first_cmd = true;
    // [User Request] 앱 연결 후 첫 패킷 수신 시, 외곽 링을 회색(0x02)으로 초기화 (GPS 수신 전 상태)
    request_circle_update(start, id, 0x04, 0x01, 0x02);

    // [User Request] 앱 연결 후 첫 패킷 수신 시, NVS에 저장되어 있던 마지막 모드로 자동 복원
    if (s_current_mode == DISPLAY_MODE_BOOT) {
      ESP_LOGI(TAG, "First App Command -> Restoring mode from NVS: %d", s_nvs_restored_mode);
      s_is_manual_mode_switch = true; // 부팅 시 복원은 잠금을 우회하여 적용
      switch_display_mode(s_nvs_restored_mode);
      s_is_manual_mode_switch = false;

      // [Protocol 3.4.3~5] Initial Sync Requests to APP
      uint8_t sync_req_time[] = {0x19, 0x4E, 0x0D, 0x01, 0x00, 0x2F};
      uint8_t sync_req_mode[] = {0x19, 0x4E, 0x11, 0x01, 0x00, 0x2F};
      uint8_t sync_req_gps[]  = {0x19, 0x4E, 0x12, 0x01, 0x00, 0x2F};
      
      ESP_LOGI(TAG, "Sending Initial Sync Requests (Time, Mode, GPS) with 300ms pacing...");
      hud_send_notify_bytes(sync_req_time, sizeof(sync_req_time));
      vTaskDelay(pdMS_TO_TICKS(300));
      
      hud_send_notify_bytes(sync_req_mode, sizeof(sync_req_mode));
      vTaskDelay(pdMS_TO_TICKS(300));
      
      hud_send_notify_bytes(sync_req_gps, sizeof(sync_req_gps));
    }
  }


  // 1. Time Set
  if (commend == 0x09 && data_length == 0x0E && len >= 19) {
    time_set(data, len);
  }

  // 2. Speed Mark
  if (commend == 0x03) {
    if (data_length == 0x02 && len >= 7) {
      uint8_t data1 = data[4];       // data1
      uint8_t speed_data2 = data[5]; // data2
      // [Auto-Switch] ONLY from BOOT mode. Other modes like CLOCK/ALBUM stay visual but update background.
      if (s_current_mode == DISPLAY_MODE_BOOT) {
        ESP_LOGI(TAG, "Active command (Speed) received. Auto-switching from BOOT to SPEEDOMETER.");
        s_guide_sub_mode = GUIDE_SUB_SPEEDOMETER;
        switch_display_mode(DISPLAY_MODE_GUIDE);
        
        // [Add] 주행 시작 시 외곽 링을 기본 회색(0x02)으로 초기화
        ESP_LOGI(TAG, "Initializing outer ring to Grey on first speed packet.");
        request_circle_update(start, id, 0x04, 0x01, 0x02); 
      }
      
      if (data1 == 0x00) {
        speed_mark(start, id, commend, data_length, data1, speed_data2);
      } else if (data1 == 0x01) {
        avr_speed_mark(start, id, commend, data_length, data1, speed_data2);
      }
    }
  }

  // 3. TBT Direction
  // TBT uses data1, data2 and optionally data3,4,5
  if (commend == 0x01 && len >= 7) {
    uint8_t data1 = data[4];
    uint8_t data2 = data[5];

    // [Auto-Switch] ONLY from BOOT mode.
    if (s_current_mode == DISPLAY_MODE_BOOT) {
        ESP_LOGI(TAG, "TBT command received. Auto-switching from BOOT to SPEEDOMETER.");
        s_guide_sub_mode = GUIDE_SUB_SPEEDOMETER;
        switch_display_mode(DISPLAY_MODE_GUIDE);
    }
    
    uint8_t tbt_data3 = (len >= 7) ? data[6] : 0;
    uint8_t tbt_data4 = (len >= 8) ? data[7] : 0;
    uint8_t tbt_data5 = (len >= 9) ? data[8] : 0;

    display_tbt_direction(start, id, commend, data_length, data1, data2,
                          tbt_data3, tbt_data4, tbt_data5);
    }

  // 4. Safety_DRV
  if (commend == 0x02 && data_length == 0x06 && len >= 11) {
    uint8_t safety_data1 = data[4];
    uint8_t safety_data2 = data[5];
    uint8_t safety_data3 = data[6];
    uint8_t safety_data4 = data[7];
    uint8_t safety_data5 = data[8];
    uint8_t safety_data6 = data[9];

    // [Auto-Switch] ONLY from BOOT mode.
    if (s_current_mode == DISPLAY_MODE_BOOT) {
        ESP_LOGI(TAG, "Safety command received. Auto-switching from BOOT to SPEEDOMETER.");
        s_guide_sub_mode = GUIDE_SUB_SPEEDOMETER;
        switch_display_mode(DISPLAY_MODE_GUIDE);
    }
    
    safety_drive(start, id, commend, data_length, safety_data1, safety_data2,
                 safety_data3, safety_data4, safety_data5, safety_data6);
  }

  // 5. Destination Info
  if (commend == 0x0A && data_length == 0x05 && len >= 10) {
    if (s_current_mode == DISPLAY_MODE_BOOT || 
       (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_STANDBY) ||
       (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER)) { // 속도계에서 목적지 수신 시 내비로 전환
      ESP_LOGI(TAG, "Destination info received. Auto-switching to NAVI.");
      s_guide_sub_mode = GUIDE_SUB_NAVI;
      switch_display_mode(DISPLAY_MODE_GUIDE);
    }
    uint8_t dest_data1 = data[4];
    uint8_t dest_data2 = data[5];
    uint8_t dest_data3 = data[6];
    uint8_t dest_data4 = data[7];
    uint8_t dest_data5 = data[8];

    request_destination_update(dest_data1, dest_data2, dest_data3, dest_data4,
                               dest_data5);
  }

  // 5.1 Clear Destination Info (19 4D 0B 01 00 2F)
  if (commend == 0x0B && len >= 6) {
    request_clear_display(0x08); // 0x08 clears destination info
  }

  // 6. Circle Drawing
  if (commend == 0x04 && data_length == 0x01 && len >= 6) {
    uint8_t circle_data1 = data[4];
    request_circle_update(start, id, commend, data_length, circle_data1);
  }

  // 7. Clear Display
  if (commend == 0x05 && data_length == 0x01 && len >= 6) {
    if (s_current_mode == DISPLAY_MODE_CLOCK || s_current_mode == DISPLAY_MODE_ALBUM || s_current_mode == DISPLAY_MODE_SETTING) {
      ESP_LOGI(TAG, "Specialty Mode (%d): Ignoring Clear Display command (0x05)", s_current_mode);
    } else if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_STANDBY) {
      ESP_LOGI(TAG, "Standby Screen: Ignoring Clear Display command (0x05)");
    } else {
      uint8_t clear_data1 = data[4];
      request_clear_display(clear_data1);
    }
  }

  // 8. Brightness Query & Firmware Info Query
  if (commend == 0x06 && len >= 6 && data_length == 0x01) {
    if (data[4] == 0x01) {
      // (1) Brightness Query
      // Cmd: 19 4D 06 01 01 2F -> Resp: 19 4E 0B 02 xx xx 2F
      uint8_t resp[7] = {
          0x19, 0x4E, 0x0B, 0x02, s_brightness_level, s_brightness_level, 0x2F};
      hud_send_notify_bytes(resp, sizeof(resp));
    } else if (data[4] == 0x03) {
      // (2) Firmware Info Query -> 버전 응답만 전송 (STANDBY 전환 없음)
      // Cmd: 19 4D 06 01 03 2F -> Resp: 19 4E 0C 0A [HUD1] [YYMMDD] 2F
      const esp_app_desc_t *app_desc = esp_app_get_description();
      uint8_t resp[15];
      resp[0] = 0x19; // Header
      resp[1] = 0x4E; // ID (Updated to 0x4E per v1.5 protocol)
      resp[2] = 0x0C; // Cmd (F/W Info Resp)
      resp[3] = 0x0A; // D-Len (10 bytes)

      // Model Name: "HUD1" (4 bytes)
      resp[4] = 'H';
      resp[5] = 'U';
      resp[6] = 'D';
      resp[7] = '1';

      // F/W Version (6 bytes, e.g., "260309")
      // app_desc->version contains the version string from CMake
      memset(&resp[8], 0, 6);
      strncpy((char *)&resp[8], app_desc->version, 6);

      resp[14] = 0x2F; // Tail

      hud_send_notify_bytes(resp, sizeof(resp));

      ESP_LOGI(TAG, "Firmware Info Resp: Model=HUD1, Ver=%.6s",
               app_desc->version);
    }
  }

  // 9. Brightness Set (화면밝기설정)
  // Cmd: 19 4D 07 02 data1 data2 2F
  if (commend == 0x07 && len >= 7 && data_length == 0x02) {
    uint8_t data1 = data[4];
    if (data1 <= 5) {
      if (data1 != s_brightness_level) {
        set_lcd_brightness(data1, true); // Level 0-5 mapping (data1 matches level)
        save_nvs_settings();             // [User Request] NVS 저장 추가
      } else {
        ESP_LOGI(TAG, "Brightness command received, but value same (%d). Syncing UI anyway.", data1);
        LVGL_LOCK();
        update_setting_ui_labels();
        LVGL_UNLOCK();
      }
    }
  }
  // 10. Mode Change via 0x0C Command
  if (commend == 0x0C && data_length == 0x01 && len >= 6) {
    uint8_t mode_val = data[4];
    // Protocol 1.10: 0x00: Safety Driving Mode, 0x01: Destination Mode
    
    // [User Request] Block auto-switch in specialized modes (CLOCK, ALBUM)
    if (s_current_mode == DISPLAY_MODE_CLOCK || s_current_mode == DISPLAY_MODE_ALBUM || s_current_mode == DISPLAY_MODE_SETTING) {
      ESP_LOGI(TAG, "0x0C command (Dest Mode) ignored in specialty mode: %d", s_current_mode);
      return;
    }

    s_guide_sub_mode = (mode_val == 0x01) ? GUIDE_SUB_NAVI : GUIDE_SUB_SPEEDOMETER;
    ESP_LOGI(TAG, "Mode switch command received: Sub-mode set to %d", s_guide_sub_mode);
    
    // Switch to integrated GUIDE mode
    switch_display_mode(DISPLAY_MODE_GUIDE);
  }

  // 11. Destination Arrived (목적지 도착) -> 속도계 화면으로 자동 전환
  if (commend == 0x0D && data_length == 0x01 && len >= 6) {
    if (s_current_mode == DISPLAY_MODE_BOOT || s_current_mode == DISPLAY_MODE_GUIDE) {
        ESP_LOGI(TAG, "Destination arrived -> Switching to SPEEDOMETER");
        s_guide_sub_mode = GUIDE_SUB_SPEEDOMETER;
        switch_display_mode(DISPLAY_MODE_GUIDE);
    } else {
        ESP_LOGI(TAG, "Destination arrived received in specialty mode. Keeping current mode.");
        s_guide_sub_mode = GUIDE_SUB_SPEEDOMETER; // 내부 상태만 갱신
    }
  }

  // 12. Road Name (도로명 전송)
  if (commend == 0x0E && len >= 5) {
    size_t name_len = (data_length > (len - 4)) ? (len - 4) : data_length;
    if (name_len > 0) {
      char road_name[128];
      if (name_len > 127)
        name_len = 127;
      memcpy(road_name, &data[4], name_len);
      road_name[name_len] = '\0';
      request_road_name_update(road_name);
    }
  }
}



// Helper to convert hex char to int
static int hex_char_to_int(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Virtual Drive Task (Replays data from log file)
static void virtual_drive_task(void *arg) {
  FILE *f = NULL;
  char line[256];
  uint8_t buffer[128];
  ESP_LOGI("VIRT", "Virtual Drive Task Started (Log Replay Mode)");

  while (1) {
    if (s_virt_drive_active && !s_connected) {
      if (f == NULL) {
        const char *log_search_paths[] = {
            "/littlefs/flash_data/merged_log.txt",
            "/littlefs/merged_log.txt",
            "/littlefs/Flash_Data/merged_log.txt",
            "/sdcard/merged_log.txt"
        };
        
        for (int i = 0; i < 4; i++) {
            // Check LittleFS mount status before opening
            if (strncmp(log_search_paths[i], "/littlefs/", 10) == 0 && !s_littlefs_mounted) continue;
            
            f = fopen(log_search_paths[i], "r");
            if (f) {
                ESP_LOGI("VIRT", "Opened Log: %s", log_search_paths[i]);
                break;
            }
        }

        if (f == NULL) {
          ESP_LOGW("VIRT", "Log file not found. Disabling Virtual Drive.");
          s_virt_drive_active = false;
          vTaskDelay(pdMS_TO_TICKS(1000));
          continue;
        }
      }

      if (fgets(line, sizeof(line), f) != NULL) {
        size_t len = 0;
        size_t line_len = strlen(line);
        for (size_t i = 0; i < line_len; i++) {
          if (line[i] == ' ' || line[i] == '\r' || line[i] == '\n') continue;
          if (i + 1 < line_len) {
            int hi = hex_char_to_int(line[i]);
            int lo = hex_char_to_int(line[i + 1]);
            if (hi >= 0 && lo >= 0) {
              if (len < sizeof(buffer)) {
                buffer[len++] = (uint8_t)((hi << 4) | lo);
                i++;
              } else break;
            }
          }
        }
        if (len > 0) process_app_command(buffer, len);
      } else {
        fseek(f, 0, SEEK_SET);
      }
      vTaskDelay(pdMS_TO_TICKS(100)); 
    } else {
      if (f != NULL) { fclose(f); f = NULL; ESP_LOGI("VIRT", "Log file closed"); }
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

// Toggle Virtual Drive Mode
void toggle_virtual_drive(bool enable) {
    if (enable && !s_virt_drive_active) {
        s_virt_drive_active = true;
        if (s_virt_drive_task_handle == NULL) {
            xTaskCreate(virtual_drive_task, "virt_drive", 8192, NULL, 5, &s_virt_drive_task_handle);
        }
        ESP_LOGW("VIRT", "Virtual Drive Mode ENABLED (Endless Replay, No NVS save, No Touch)");
        s_guide_sub_mode = GUIDE_SUB_SPEEDOMETER;
        
        // [User Request] Ensure outer ring is visible in Virtual Drive mode
        LVGL_LOCK();
        ensure_circle_ring_created();
        if (s_circle_ring) {
            // Initialize to Grey (0x02) directly
            lv_obj_set_style_border_color(s_circle_ring, lv_color_hex(0x808080), 0);
            lv_obj_set_style_border_width(s_circle_ring, 5, 0);
            lv_obj_set_size(s_circle_ring, 463, 463);
            lv_obj_center(s_circle_ring);
            lv_obj_clear_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
            lv_obj_invalidate(s_circle_ring);
        }
        LVGL_UNLOCK();

        // Use a direct mode switch that avoids NVS save by the guard in save_nvs_settings
        switch_display_mode(DISPLAY_MODE_GUIDE);
    } else if (!enable && s_virt_drive_active) {
        s_virt_drive_active = false;
        ESP_LOGW("VIRT", "Virtual Drive Mode DISABLED");
    }
}


// Parse CSV file and load image data mappings
static esp_err_t load_image_data_csv(void) {
  if (!s_littlefs_mounted) {
    ESP_LOGE(TAG, "LittleFS not mounted, "
                  "cannot load CSV file");
    return ESP_ERR_INVALID_STATE;
  }

  // Debug: List files in /littlefs to see what's actually there
  /*
  ESP_LOGI(TAG, "Listing files in /littlefs:");
  DIR *dir = opendir("/littlefs");
  if (dir != NULL) {
    struct dirent *entry;
    int file_count = 0;
    while ((entry = readdir(dir)) != NULL) {
      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
        ESP_LOGI(TAG, "  Found: %s (type=%d)", entry->d_name, entry->d_type);
        file_count++;
      }
    }
    closedir(dir);
    if (file_count == 0) {
      ESP_LOGW(TAG, "  /littlefs directory is empty");
    }
  } else {
    ESP_LOGW(TAG, "Failed to open /littlefs directory");
  }
  */

  // Try to list /littlefs/image if it exists
  // (might contain CSV)
  DIR *image_dir = opendir("/littlefs/image");
  if (image_dir != NULL) {
    /*
    ESP_LOGI(TAG, "Listing files in /littlefs/image:");
    struct dirent *entry;
    int file_count = 0;
    while ((entry = readdir(image_dir)) != NULL) {
      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
        ESP_LOGI(TAG, "  Found: %s (type=%d)", entry->d_name, entry->d_type);
        file_count++;
      }
    }
    */
    closedir(image_dir);
    /*
    if (file_count == 0) {
      ESP_LOGW(TAG, "  /littlefs/image "
                    "directory is empty");
    }
    */
  } else {
    ESP_LOGD(TAG, "/littlefs/image directory "
                  "does not exist");
  }

  // Try to list /littlefs/flash_data if it exists
  /*
  DIR *flash_data_dir = opendir("/littlefs/flash_data");
  if (flash_data_dir != NULL) {
    ESP_LOGI(TAG, "Listing files in /littlefs/flash_data:");
    struct dirent *entry;
    int file_count = 0;
    while ((entry = readdir(flash_data_dir)) != NULL) {
      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
        ESP_LOGI(TAG, "  Found: %s (type=%d)", entry->d_name, entry->d_type);
        file_count++;
      }
    }
    closedir(flash_data_dir);
    if (file_count == 0) {
      ESP_LOGW(TAG, "  /littlefs/flash_data directory is empty");
    }
  } else {
    ESP_LOGD(TAG, "/littlefs/flash_data directory does not exist");
  }
  */

  // Try multiple possible paths for CSV file
  const char *csv_paths[] = {
      "/littlefs/TBT.CSV", "/littlefs/flash_data/TBT.CSV",
      "/littlefs/TBT.csv",            // lowercase
      "/littlefs/flash_data/TBT.csv", // lowercase
      "/littlefs/image/TBT.CSV",      // image
                                      // directory
      "/littlefs/image/TBT.csv",      // lowercase
                                      // in image
                                      // directory
      // Legacy paths for backward
      // compatibility
      "/littlefs/image/image.CSV", "/littlefs/image.CSV",
      "/littlefs/flash_data/image.CSV", "/littlefs/image/image.csv",
      "/littlefs/image.csv", "/littlefs/flash_data/image.csv"};
  FILE *fp = NULL;
  const char *csv_path = NULL;

  for (int i = 0; i < sizeof(csv_paths) / sizeof(csv_paths[0]); i++) {
    csv_path = csv_paths[i];
    fp = fopen(csv_path, "r");
    if (fp != NULL) {
      ESP_LOGI(TAG, "CSV file found at: %s", csv_path);
      break;
    }
  }

  if (fp == NULL) {
    ESP_LOGW(TAG,
             "CSV file not found. Tried %zu "
             "different paths",
             sizeof(csv_paths) / sizeof(csv_paths[0]));
    return ESP_ERR_NOT_FOUND;
  }

  // Debug: Read first few bytes to check if
  // file is text or binary
  char first_bytes[64];
  size_t bytes_read = fread(first_bytes, 1, sizeof(first_bytes) - 1, fp);
  first_bytes[bytes_read] = '\0';
  fseek(fp, 0,
        SEEK_SET); // Reset to beginning

/*
  ESP_LOGI(TAG,
           "CSV file opened, first %zu bytes "
           "(hex):",
           bytes_read);
  for (size_t i = 0; i < bytes_read && i < 64; i++) {
    if (i % 16 == 0) {
      ESP_LOGI(TAG, "  %04zu: ", i);
    }
    ESP_LOGI(TAG, "%02X ", (unsigned char)first_bytes[i]);
    if (i % 16 == 15) {
      ESP_LOGI(TAG, "");
    }
  }
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "CSV file first %zu bytes (text): %.*s", bytes_read,
           (int)bytes_read, first_bytes);
*/

  // Count lines first (skip header line)
  size_t line_count = 0;
  char line[512];
  bool first_line = true;
  while (fgets(line, sizeof(line), fp) != NULL) {
    if (first_line) {
      first_line = false;
      continue; // Skip header line
    }
    // Skip empty lines and comments
    if (line[0] == '\n' || line[0] == '\r' || line[0] == '#' ||
        line[0] == '\0') {
      continue;
    }
    line_count++;
  }
  rewind(fp);

  if (line_count == 0) {
    fclose(fp);
    ESP_LOGW(TAG, "CSV file is empty (no data lines)");
    return ESP_OK;
  }

  // Allocate memory for entries (use PSRAM)
  s_image_data_entries = (image_data_entry_t *)lvgl_psram_malloc(
      line_count * sizeof(image_data_entry_t));
  if (s_image_data_entries == NULL) {
    fclose(fp);
    ESP_LOGE(TAG, "Failed to allocate memory "
                  "for image data entries");
    return ESP_ERR_NO_MEM;
  }

  // Parse CSV file
  // Format: start,ID,commend,data
  // length,data1,data2,data3,data4,data5,end,,image,sector,size
  // We need: data2 (column 5), data3 (column
  // 6), data4 (column 7), image (column 11)
  size_t idx = 0;
  first_line = true;
  while (fgets(line, sizeof(line), fp) != NULL && idx < line_count) {
    // Skip header line
    if (first_line) {
      first_line = false;
      continue;
    }

    // Skip empty lines and comments
    if (line[0] == '\n' || line[0] == '\r' || line[0] == '#' ||
        line[0] == '\0') {
      continue;
    }

    // Remove trailing newline
    size_t len = strlen(line);
    if (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
      line[len - 1] = '\0';
      len--;
    }

    // Parse CSV line by finding commas
    // (handle empty fields properly) CSV
    // format: start,ID,commend,data
    // length,data1,data2,data3,data4,data5,end,,image,sector,size
    // 1-based column: 1   2   3       4 5 6
    // 7      8      9     10 11  12     13 14
    // 0-based index: 0   1   2       3 4 5 6
    // 7      8      9  10  11     12     13
    // We extract: data2 (index 5, 6th
    // column), data3 (index 6, 7th column),
    // data4 (index 7, 8th column), image
    // (index 11, 12th column)
    char *fields[14] = {NULL}; // Need 14 fields: 0-13 (image
                               // is at index 11, which is
                               // 12th column)
    char *p = line;
    int field_idx = 0;
    bool in_quotes = false;

    // Split by comma, handling quoted fields
    // properly
    for (int i = 0; i < 14; i++) {
      fields[i] = p;

      // Find next comma, but skip commas
      // inside quoted fields
      char *next_comma = NULL;
      if (in_quotes) {
        // If we're inside quotes, find the
        // closing quote first
        char *quote_end = strchr(p + 1, '"');
        if (quote_end != NULL) {
          in_quotes = false;
          next_comma = strchr(quote_end + 1, ',');
        } else {
          // No closing quote found, treat
          // rest as one field
          next_comma = NULL;
        }
      } else {
        // Check if field starts with quote
        if (*p == '"') {
          in_quotes = true;
          char *quote_end = strchr(p + 1, '"');
          if (quote_end != NULL) {
            in_quotes = false;
            next_comma = strchr(quote_end + 1, ',');
          } else {
            // No closing quote, treat as
            // unquoted
            next_comma = strchr(p, ',');
          }
        } else {
          next_comma = strchr(p, ',');
        }
      }

      if (next_comma != NULL) {
        *next_comma = '\0';
        p = next_comma + 1;
      } else {
        // Last field, no more commas
        p = NULL;
      }
      field_idx++;
      if (p == NULL)
        break;
    }

    // Debug: log all fields to see what we're
    // parsing
    // Debug: log all fields only if needed (disabled for boot speed)
    /*
    if (idx < 3) {
      ESP_LOGI(TAG, "CSV line %zu: field_idx=%d", idx, field_idx);
      for (int i = 0; i < 14 && i < field_idx; i++) {
        ESP_LOGI(TAG, "  field[%d]='%s'", i, fields[i] ? fields[i] : "(empty)");
      }
    }
    */

    // Extract: start(0), id(1), commend(2),
    // data_length(3), data1(4), data2(5),
    // data3(6), data4(7), data5(8), end(9),
    // image(10), sector(11) TBT.CSV has 12
    // fields (no size field)
    if (field_idx < 12) {
      ESP_LOGW(TAG,
               "CSV line %zu has insufficient "
               "fields (%d), skipping. Line: %s",
               idx, field_idx, line);
      continue;
    }

    char *start_str = fields[0];
    char *id_str = fields[1];
    char *commend_str = fields[2];
    char *data_length_str = fields[3];
    char *data1_str = fields[4];
    char *data2_str = fields[5];
    char *filename_str = fields[10]; // image is at index 10
    char *sector_str = fields[11];   // sector is at index 11

    // Debug: log field values for first few
    // entries
    // Debug: log field values only if needed (disabled for boot speed)
    /*
    if (idx < 3) {
      ESP_LOGI(TAG,
               "CSV line %zu fields: start='%s' "
               "id='%s' commend='%s' dlen='%s' "
               "data1='%s' data2='%s'",
               idx, start_str ? start_str : "NULL", id_str ? id_str : "NULL",
               commend_str ? commend_str : "NULL",
               data_length_str ? data_length_str : "NULL",
               data1_str ? data1_str : "NULL", data2_str ? data2_str : "NULL");
      ESP_LOGI(TAG,
               "CSV line %zu fields: "
               "filename='%s' sector='%s'",
               idx, filename_str ? filename_str : "NULL",
               sector_str ? sector_str : "NULL");
    }
    */

    if (!start_str || !id_str || !commend_str || !data_length_str ||
        !data1_str || !data2_str || !filename_str || !sector_str) {
      ESP_LOGW(TAG,
               "CSV line %zu has NULL "
               "fields, skipping",
               idx);
      continue;
    }

    // Trim whitespace
    while (*start_str == ' ' || *start_str == '\t')
      start_str++;
    while (*id_str == ' ' || *id_str == '\t')
      id_str++;
    while (*commend_str == ' ' || *commend_str == '\t')
      commend_str++;
    while (*data_length_str == ' ' || *data_length_str == '\t')
      data_length_str++;
    while (*data1_str == ' ' || *data1_str == '\t')
      data1_str++;
    while (*data2_str == ' ' || *data2_str == '\t')
      data2_str++;
    while (*filename_str == ' ' || *filename_str == '\t')
      filename_str++;
    while (*sector_str == ' ' || *sector_str == '\t')
      sector_str++;

    // Remove surrounding quotes from filename
    // if present
    const char *filename_start = filename_str;
    size_t filename_len = strlen(filename_str);
    if (filename_len >= 2 && filename_str[0] == '"' &&
        filename_str[filename_len - 1] == '"') {
      filename_start = filename_str + 1;
      filename_len -= 2;
    }

    // Parse hex values
    // Empty fields are treated as "ignore"
    // (0xFF) for matching
    uint32_t start = strtoul(start_str, NULL, 16);
    uint32_t id = strtoul(id_str, NULL, 16);
    uint32_t commend = strtoul(commend_str, NULL, 16);
    uint32_t data_length = strtoul(data_length_str, NULL, 16);
    // Check if data1 is empty (after
    // trimming)
    uint32_t data1 =
        (strlen(data1_str) == 0) ? 0xFF : strtoul(data1_str, NULL, 16);
    // Check if data2 is empty (after
    // trimming)
    uint32_t data2 =
        (strlen(data2_str) == 0) ? 0xFF : strtoul(data2_str, NULL, 16);
    uint32_t sector = strtoul(sector_str, NULL,
                              10); // sector is decimal

    if (start > 255 || id > 255 || commend > 255 || data_length > 255 ||
        (data1 != 0xFF && data1 > 255) || (data2 != 0xFF && data2 > 255)) {
      ESP_LOGW(TAG,
               "Invalid data values in CSV "
               "line %zu, skipping",
               idx);
      continue;
    }

    s_image_data_entries[idx].start = (uint8_t)start;
    s_image_data_entries[idx].id = (uint8_t)id;
    s_image_data_entries[idx].commend = (uint8_t)commend;
    s_image_data_entries[idx].data_length = (uint8_t)data_length;
    s_image_data_entries[idx].data1 = (uint8_t)data1;
    s_image_data_entries[idx].data2 = (uint8_t)data2;
    s_image_data_entries[idx].sector = (uint8_t)sector;

    // Copy filename
    size_t copy_len = filename_len;
    if (copy_len >= sizeof(s_image_data_entries[idx].image_filename)) {
      copy_len = sizeof(s_image_data_entries[idx].image_filename) - 1;
    }
    size_t j = 0;
    for (size_t i = 0; i < copy_len && filename_start[i] != '\0'; i++) {
      unsigned char c = (unsigned char)filename_start[i];
      if (c == '"')
        continue;
      if (c >= 0x20 && c <= 0x7E) {
        s_image_data_entries[idx].image_filename[j++] = c;
      } else if (c == '\0') {
        break;
      }
    }
    s_image_data_entries[idx].image_filename[j] = '\0';

    // Size field doesn't exist in TBT.CSV,
    // set to empty
    s_image_data_entries[idx].size[0] = '\0';

    ESP_LOGD(TAG,
             "CSV parsed entry %zu: start=0x%02X "
             "id=0x%02X commend=0x%02X "
             "dlen=0x%02X data1=0x%02X "
             "data2=0x%02X -> %s (sector=%u)",
             idx, s_image_data_entries[idx].start, s_image_data_entries[idx].id,
             s_image_data_entries[idx].commend,
             s_image_data_entries[idx].data_length,
             s_image_data_entries[idx].data1, s_image_data_entries[idx].data2,
             s_image_data_entries[idx].image_filename,
             s_image_data_entries[idx].sector, s_image_data_entries[idx].size);

    idx++;
  }
  fclose(fp);

  s_image_data_count = idx;
  ESP_LOGI(TAG,
           "Loaded %zu image data entries "
           "from CSV",
           s_image_data_count);
  for (size_t i = 0; i < s_image_data_count; i++) {
    ESP_LOGI(TAG,
             "  Entry %zu: start=0x%02X id=0x%02X "
             "commend=0x%02X dlen=0x%02X "
             "data1=0x%02X data2=0x%02X -> %s "
             "(sector=%u, size=%s)",
             i, s_image_data_entries[i].start, s_image_data_entries[i].id,
             s_image_data_entries[i].commend,
             s_image_data_entries[i].data_length, s_image_data_entries[i].data1,
             s_image_data_entries[i].data2,
             s_image_data_entries[i].image_filename,
             s_image_data_entries[i].sector, s_image_data_entries[i].size);
  }

  return ESP_OK;
}

// Parse Safety_DRV CSV file and load image
// data mappings
static esp_err_t load_safety_data_csv(void) {
  if (!s_littlefs_mounted) {
    ESP_LOGE(TAG, "LittleFS not mounted, cannot "
                  "load Safety_DRV CSV file");
    return ESP_ERR_INVALID_STATE;
  }

  // Try multiple possible paths for CSV file
  const char *csv_paths[] = {
      "/littlefs/flash_data/Safety_DRV.CSV", "/littlefs/Safety_DRV.CSV",
      "/littlefs/flash_data/safety_drv.csv", "/littlefs/safety_drv.csv",
      "/littlefs/image/Safety_DRV.CSV",      "/littlefs/image/safety_drv.csv"};
  FILE *fp = NULL;
  const char *csv_path = NULL;

  for (int i = 0; i < sizeof(csv_paths) / sizeof(csv_paths[0]); i++) {
    csv_path = csv_paths[i];
    fp = fopen(csv_path, "r");
    if (fp != NULL) {
      ESP_LOGI(TAG, "Safety_DRV CSV file found at: %s", csv_path);
      break;
    }
  }

  if (fp == NULL) {
    ESP_LOGW(TAG,
             "Safety_DRV CSV file not found. "
             "Tried %zu different paths",
             sizeof(csv_paths) / sizeof(csv_paths[0]));
    return ESP_ERR_NOT_FOUND;
  }

  // Count lines first (skip header line)
  size_t line_count = 0;
  char line[512];
  bool first_line = true;
  while (fgets(line, sizeof(line), fp) != NULL) {
    if (first_line) {
      first_line = false;
      continue; // Skip header line
    }
    // Skip empty lines and comments
    if (line[0] == '\n' || line[0] == '\r' || line[0] == '#' ||
        line[0] == '\0') {
      continue;
    }
    line_count++;
  }
  rewind(fp);

  if (line_count == 0) {
    fclose(fp);
    ESP_LOGW(TAG, "Safety_DRV CSV file is "
                  "empty (no data lines)");
    return ESP_OK;
  }

  // Allocate memory for entries (use PSRAM)
  s_safety_data_entries = (safety_data_entry_t *)lvgl_psram_malloc(
      line_count * sizeof(safety_data_entry_t));
  if (s_safety_data_entries == NULL) {
    fclose(fp);
    ESP_LOGE(TAG, "Failed to allocate memory "
                  "for safety data entries");
    return ESP_ERR_NO_MEM;
  }

  // Parse CSV file
  // Format:
  // start,ID,commend,data_length,data1,data2,data3,data4,data5,data6,end,image,sector
  size_t idx = 0;
  first_line = true;
  while (fgets(line, sizeof(line), fp) != NULL && idx < line_count) {
    // Skip header line
    if (first_line) {
      first_line = false;
      continue;
    }

    // Skip empty lines and comments
    if (line[0] == '\n' || line[0] == '\r' || line[0] == '#' ||
        line[0] == '\0') {
      continue;
    }

    // Remove trailing newline
    size_t len = strlen(line);
    if (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
      line[len - 1] = '\0';
      len--;
    }

    // Parse CSV line by finding commas
    // CSV format:
    // start,ID,commend,data_length,data1,data2,data3,data4,data5,data6,end,image,sector
    // 0-based index: 0    1   2       3 4 5
    // 6     7     8 9 10  11     12
    char *fields[14] = {NULL};
    char *p = line;
    int field_idx = 0;
    bool in_quotes = false;

    // Split by comma, handling quoted fields
    // properly
    for (int i = 0; i < 14; i++) {
      fields[i] = p;

      // Find next comma, but skip commas
      // inside quoted fields
      char *next_comma = NULL;
      if (in_quotes) {
        char *quote_end = strchr(p + 1, '"');
        if (quote_end != NULL) {
          in_quotes = false;
          next_comma = strchr(quote_end + 1, ',');
        } else {
          next_comma = NULL;
        }
      } else {
        if (*p == '"') {
          in_quotes = true;
          next_comma = strchr(p + 1, ',');
        } else {
          next_comma = strchr(p, ',');
        }
      }

      if (next_comma != NULL) {
        *next_comma = '\0';
        p = next_comma + 1;
      } else {
        p = NULL;
      }
      field_idx++;
      if (p == NULL)
        break;
    }

    // Extract: start(0), id(1), commend(2),
    // data_length(3), data1(4), image(11),
    // sector(12) Safety_DRV.CSV has 13 fields
    // (indices 0-12), no size field
    if (field_idx < 13) {
      ESP_LOGW(TAG,
               "Safety_DRV CSV line %zu has "
               "insufficient fields (%d), "
               "skipping",
               idx, field_idx);
      continue;
    }

    char *start_str = fields[0];
    char *id_str = fields[1];
    char *commend_str = fields[2];
    char *data_length_str = fields[3];
    char *data1_str = fields[4];
    char *filename_str = fields[11]; // image is at index 11
    char *sector_str = fields[12];   // sector is at index 12

    if (!start_str || !id_str || !commend_str || !data_length_str ||
        !data1_str || !filename_str || !sector_str) {
      ESP_LOGW(TAG,
               "Safety_DRV CSV line %zu has "
               "NULL fields, skipping",
               idx);
      continue;
    }

    // Trim whitespace
    while (*start_str == ' ' || *start_str == '\t')
      start_str++;
    while (*id_str == ' ' || *id_str == '\t')
      id_str++;
    while (*commend_str == ' ' || *commend_str == '\t')
      commend_str++;
    while (*data_length_str == ' ' || *data_length_str == '\t')
      data_length_str++;
    while (*data1_str == ' ' || *data1_str == '\t')
      data1_str++;
    while (*filename_str == ' ' || *filename_str == '\t')
      filename_str++;
    while (*sector_str == ' ' || *sector_str == '\t')
      sector_str++;

    // Remove surrounding quotes from filename
    // if present
    const char *filename_start = filename_str;
    size_t filename_len = strlen(filename_str);
    if (filename_len >= 2 && filename_str[0] == '"' &&
        filename_str[filename_len - 1] == '"') {
      filename_start = filename_str + 1;
      filename_len -= 2;
    }

    // Parse hex values
    uint32_t start = strtoul(start_str, NULL, 16);
    uint32_t id = strtoul(id_str, NULL, 16);
    uint32_t commend = strtoul(commend_str, NULL, 16);
    uint32_t data_length = strtoul(data_length_str, NULL, 16);
    uint32_t data1 = strtoul(data1_str, NULL, 16);
    uint32_t sector = strtoul(sector_str, NULL,
                              10); // sector is decimal

    if (start > 255 || id > 255 || commend > 255 || data_length > 255 ||
        data1 > 255) {
      ESP_LOGW(TAG,
               "Invalid data values in Safety_DRV "
               "CSV line %zu, skipping",
               idx);
      continue;
    }

    s_safety_data_entries[idx].start = (uint8_t)start;
    s_safety_data_entries[idx].id = (uint8_t)id;
    s_safety_data_entries[idx].commend = (uint8_t)commend;
    s_safety_data_entries[idx].data_length = (uint8_t)data_length;
    s_safety_data_entries[idx].data1 = (uint8_t)data1;
    s_safety_data_entries[idx].sector = (uint8_t)sector;

    // Copy filename
    size_t copy_len = filename_len;
    if (copy_len >= sizeof(s_safety_data_entries[idx].image_filename)) {
      copy_len = sizeof(s_safety_data_entries[idx].image_filename) - 1;
    }
    size_t j = 0;
    for (size_t i = 0; i < copy_len && filename_start[i] != '\0'; i++) {
      unsigned char c = (unsigned char)filename_start[i];
      if (c == '"')
        continue;
      if (c >= 0x20 && c <= 0x7E) {
        s_safety_data_entries[idx].image_filename[j++] = c;
      } else if (c == '\0') {
        break;
      }
    }
    s_safety_data_entries[idx].image_filename[j] = '\0';

    // Size field doesn't exist in
    // Safety_DRV.CSV, set to empty
    s_safety_data_entries[idx].size[0] = '\0';

    ESP_LOGD(TAG,
             "Safety_DRV CSV parsed entry %zu: "
             "start=0x%02X id=0x%02X "
             "commend=0x%02X dlen=0x%02X "
             "data1=0x%02X -> %s (sector=%u)",
             idx, s_safety_data_entries[idx].start,
             s_safety_data_entries[idx].id, s_safety_data_entries[idx].commend,
             s_safety_data_entries[idx].data_length,
             s_safety_data_entries[idx].data1,
             s_safety_data_entries[idx].image_filename,
             s_safety_data_entries[idx].sector,
             s_safety_data_entries[idx].size);

    idx++;
  }
  fclose(fp);

  s_safety_data_count = idx;
  ESP_LOGI(TAG,
           "Loaded %zu safety data entries "
           "from CSV",
           s_safety_data_count);
  return ESP_OK;
}

// Find image entry for given start, id,
// commend, data_length, data1, data2 values
// TBT방향표시: start, id, commend,
// data_length, data1, data2가 일치하면 실행
// 참고파일: "littlefs/TBT.csv"
static const image_data_entry_t *
find_tbt_image_entry(uint8_t start, uint8_t id, uint8_t commend,
                     uint8_t data_length, uint8_t data1, uint8_t data2) {
  for (size_t i = 0; i < s_image_data_count; i++) {
    // start, id, commend, data_length, data1,
    // data2가 모두 일치해야 함
    if (s_image_data_entries[i].start == start &&
        s_image_data_entries[i].id == id &&
        s_image_data_entries[i].commend == commend &&
        s_image_data_entries[i].data_length == data_length &&
        s_image_data_entries[i].data1 == data1 &&
        s_image_data_entries[i].data2 == data2) {
      return &s_image_data_entries[i];
    }
  }
  return NULL; // No matching data found
}

// TBT방향표시 함수
// 참고파일: "littlefs/TBT.csv"
// data 형식: "start, ID, commend,
// data_length, data1, data2, data3, data4,
// data5, end" start, id, commend,
// data_length, data1, data2가 일치하면
// 실행한다. 이미지파일은 "littlefs/image/"에
// 있다. CSV "image"항에 해당 이미지를
// 사용한다. "sector"를 참고하여 LCD에
// 표기한다. 해당 이미지파일 호출과 표기는
// 표기하고있는 이미지가 없거나 변경되었을때만
// 화면을 갱신한다. data3, data4, data5는
// 16진수 미터단위의 정보다. 각 바이트는
// 16진수 자리를 표현하며 이를 10진수로
// 변환하여 "TBT계산값"으로 사용한다.
static void display_tbt_direction(uint8_t start, uint8_t id, uint8_t commend,
                                  uint8_t data_length, uint8_t data1,
                                  uint8_t data2, uint8_t data3, uint8_t data4,
                                  uint8_t data5) {
  const image_data_entry_t *entry =
      find_tbt_image_entry(start, id, commend, data_length, data1, data2);
  if (entry == NULL) {
    return;
  }

  // Request image update via queue (이미지가
  // 변경되었을 때만 갱신) TBT distance 정보도
  // 함께 전달하여 LVGL 태스크에서 처리하도록
  // 함
  request_image_update(start, id, commend, data_length, data1, data2, false,
                       true, data3, data4, data5);
}

// Find Safety_DRV image entry for given
// start, id, commend, data_length, data1
// values
static const safety_data_entry_t *
find_safety_image_entry(uint8_t start, uint8_t id, uint8_t commend,
                        uint8_t data_length, uint8_t data1) {
  // Log removed - only HUD TX data is logged

  for (size_t i = 0; i < s_safety_data_count; i++) {
    if (s_safety_data_entries[i].start == start &&
        s_safety_data_entries[i].id == id &&
        s_safety_data_entries[i].commend == commend &&
        s_safety_data_entries[i].data_length == data_length &&
        s_safety_data_entries[i].data1 == data1) {
      // Log removed - only HUD TX data is
      // logged
      return &s_safety_data_entries[i];
    }
  }
  // Log removed - only HUD TX data is logged
  return NULL; // No matching data found
}

// Request Safety_DRV image update via queue
// (to avoid crash in BLE callback)
static void request_safety_update(uint8_t start, uint8_t id, uint8_t commend,
                                  uint8_t data_length, uint8_t data1,
                                  uint8_t data2, uint8_t data3, uint8_t data4,
                                  uint8_t data5, uint8_t data6) {
  if (s_safety_update_queue == NULL) {
    ESP_LOGW(TAG, "Safety update queue not "
                  "initialized, cannot "
                  "request safety update");
    return;
  }

  safety_update_request_t req = {.start = start,
                                 .id = id,
                                 .commend = commend,
                                 .data_length = data_length,
                                 .data1 = data1,
                                 .data2 = data2,
                                 .data3 = data3,
                                 .data4 = data4,
                                 .data5 = data5,
                                 .data6 = data6};

  // Filter duplicate requests (same request
  // as last one)
  if (s_last_safety_request_valid) {
    if (memcmp(&req, &s_last_safety_request, sizeof(safety_update_request_t)) ==
        0) {
      // Same request as last one, skip to
      // prevent queue overflow
      return;
    }
  }

  // Save last request
  memcpy(&s_last_safety_request, &req, sizeof(safety_update_request_t));
  s_last_safety_request_valid = true;

  // Check queue usage and remove old items if
  // queue is >80% full to maintain buffer
  UBaseType_t queue_size = uxQueueMessagesWaiting(s_safety_update_queue);
  UBaseType_t queue_capacity =
      uxQueueSpacesAvailable(s_safety_update_queue) + queue_size;
  if (queue_capacity > 0) {
    float usage = (float)queue_size / (float)queue_capacity;
    const float CLEANUP_THRESHOLD = 0.80f; // 80% threshold
    if (usage >= CLEANUP_THRESHOLD) {
      // Remove old items until queue usage
      // drops to ~50% to maintain buffer
      safety_update_request_t dummy;
      UBaseType_t target_size = queue_capacity / 2; // Target 50% usage
      UBaseType_t items_to_remove =
          (queue_size > target_size) ? (queue_size - target_size) : 0;
      UBaseType_t removed_count = 0;
      for (UBaseType_t i = 0; i < items_to_remove; i++) {
        if (xQueueReceive(s_safety_update_queue, &dummy, 0) == pdTRUE) {
          removed_count++;
        } else {
          break; // No more items to remove
        }
      }
      if (removed_count > 0) {
        ESP_LOGW(TAG,
                 "Safety update queue cleanup: "
                 "removed %lu old items (usage "
                 "was %.1f%%, now %.1f%%)",
                 (unsigned long)removed_count, usage * 100.0f,
                 ((float)(queue_size - removed_count) / (float)queue_capacity) *
                     100.0f);
      }
    }
  }

  // Non-blocking queue send to prevent BLE
  // callback blocking
  if (xQueueSend(s_safety_update_queue, &req, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Safety update queue full, dropping "
                  "request (non-blocking)");
  }
}

// Update Safety_DRV image and distance labels
// (called from LVGL task) NOTE: This function
// should only be called from LVGL handler
// task
static void update_safety_image_for_data(const safety_data_entry_t *entry,
                                         uint8_t data2, uint8_t data3,
                                         uint8_t data4, uint8_t data5,
                                         uint8_t data6) {
  if (entry == NULL) {
    ESP_LOGW(TAG, "update_safety_image_for_data: entry is NULL");
    return;
  }

  // 연결이 끊겼을 때는 앱으로부터의 정보를 무시함 (잔상 방지)
  // [User Request] 가상 운행 모드에서는 연결이 없어도 업데이트 허용
  if (!s_connected && !s_virt_drive_active) {
      return;
  }

  hide_black_screen_overlay();

  // 링(ring) 처리는 모든 모드에서 허용
  // 이미지/거리 표시는 HUD/SPEEDOMETER 모드에서만
  bool ring_only_mode = (s_current_mode != DISPLAY_MODE_GUIDE);
  if (ring_only_mode) {
    ESP_LOGD(TAG,
             "Not in HUD or Speedometer mode (current mode=%d), "
             "ring only (no image display)",
             s_current_mode);
  }

  // 이미지/거리 표시는 HUD/SPEEDOMETER 모드에서만 처리
  if (!ring_only_mode) {
    // Replace "tt" in image filename with data2
    // (decimal)
    char img_filename[64];
    strncpy(img_filename, entry->image_filename, sizeof(img_filename) - 1);
    img_filename[sizeof(img_filename) - 1] = '\0';

    // Find "tt" and replace with data2 (decimal)
    // Constraint: 30 <= data2 <= 120
    char *tt_pos = strstr(img_filename, "tt");
    if (tt_pos != NULL) {
      uint8_t effective_data2 = data2;
      if (effective_data2 < 30) effective_data2 = 30;
      else if (effective_data2 > 120) effective_data2 = 120;

      char data2_str[4];
      snprintf(data2_str, sizeof(data2_str), "%02d", effective_data2);
      // Replace "tt" with data2_str
      size_t tt_len = strlen("tt");
      size_t data2_len = strlen(data2_str);
      size_t filename_len = strlen(img_filename);
      size_t pos = tt_pos - img_filename;

      if (pos + tt_len <= filename_len) {
        // Shift remaining characters
        memmove(tt_pos + data2_len, tt_pos + tt_len,
                filename_len - pos - tt_len + 1);
        // Insert data2_str
        memcpy(tt_pos, data2_str, data2_len);
      }
    }

    // Build full image path
    // Use LVGL VFS path (S: drive) for consistency with other image loading
    char img_path[128];
    snprintf(img_path, sizeof(img_path), "S:/littlefs/image/%s", img_filename);

    // Only update image if the path has changed
    // (이미지가 없거나 변경되었을 때만 갱신)
    // 표기하고있는 이미지가 없거나
    // 변경되었을때만 화면을 갱신한다.
    bool should_load_image =
        (strcmp(img_path, s_current_safety_image_path) != 0);
    if (should_load_image) {
      ESP_LOGI(TAG,
               "Safety_DRV: data1=0x%02X "
               "data2=0x%02X(%d) -> filename=%s "
               "(sector=%u)",
               entry->data1, data2, data2, img_filename, entry->sector);

      // Update current image path
      strncpy(s_current_safety_image_path, img_path,
              sizeof(s_current_safety_image_path) - 1);
      s_current_safety_image_path[sizeof(s_current_safety_image_path) - 1] =
          '\0';
    }

    // Calculate position based on sector first

    lv_coord_t offset_x = 0;
    lv_coord_t offset_y = 0;

    switch (entry->sector) {
    case 0: // LCD 중앙에서 위로 66pt 이동
      offset_x = 0;
      offset_y = -66;
      break;
    case 12: // LCD 중앙에서 위로 85pt, 우측으로 130pt 이동 (이전 -90에서 수정)
      offset_x = 130;
      offset_y = -85;
      break;
    case 6: // LCD 중앙에서 아래로 100pt 이동
      offset_x = 0;
      offset_y = 100;
      break;
    case 3: // LCD 중앙에서 우측으로 100pt 이동
      offset_x = 100;
      offset_y = 0;
      break;
    case 9: // LCD 중앙에서 좌측으로 150pt 이동
      offset_x = -150;
      offset_y = 0;
      break;
    default:
      offset_x = 0;
      offset_y = 0;
      break;
    }

    // Only load image if path has changed
    if (should_load_image) {
      // Removed lv_timer_handler() to prevent
      // potential reentrancy/hangs during heavy
      // image decoding tasks.

      ESP_LOGI(TAG, "Safety_DRV: Starting image load: %s (sector=%u)", img_path,
               entry->sector);

      // Create or update safety    // 이미지 객체가 없으면 생성
      if (s_safety_image == NULL) {
        s_safety_image = lv_img_create(s_hud_screen);
        if (s_safety_image == NULL) {
          ESP_LOGE(TAG, "Safety_DRV: Failed to "
                        "create image object");
          return;
        }
        // Set position immediately when
        // creating object (don't use
        // lv_obj_center)
        lv_obj_align(s_safety_image, LV_ALIGN_CENTER, offset_x, offset_y);
        // Move safety image to foreground so
        // it's not hidden behind other objects
        lv_obj_move_foreground(s_safety_image);
      } else {
        // Update position if object already
        // exists
        lv_obj_align(s_safety_image, LV_ALIGN_CENTER, offset_x, offset_y);
      }

      // Keep TBT image visible - both images
      // can be displayed simultaneously Safety
      // image will be on top (foreground) due
      // to move_foreground call

      // Set image source (after object is
      // properly initialized) Hide image first
      // to prevent white rectangle flash during
      // image swap
      lv_obj_add_flag(s_safety_image, LV_OBJ_FLAG_HIDDEN);
      lv_obj_move_foreground(s_safety_image); // Ensure it's on top

      // Clear previous image source before
      // loading new one to free memory This
      // prevents memory corruption when
      // switching between different sized
      // images
      lv_img_set_src(s_safety_image, NULL);
      // Removed redundant lv_timer_handler() to
      // prevent potential reentrancy/hangs
      update_heartbeat_lvgl();       // 하트비트
                                     // 업데이트
                                     // (메모리 정리
                                     // 전)
      vTaskDelay(pdMS_TO_TICKS(10)); // Allow time for memory cleanup
      update_heartbeat_lvgl();       // 하트비트
                                     // 업데이트
                                     // (딜레이 후)

      // int64_t set_src_start_us =
      // esp_timer_get_time();
      lv_img_set_src(s_safety_image, img_path);
      // int64_t set_src_end_us =
      // esp_timer_get_time(); int64_t
      // set_src_duration_ms = (set_src_end_us -
      // set_src_start_us) / 1000; ESP_LOGI(TAG,
      // "Safety_DRV: lv_img_set_src() took %lld
      // ms for %s",
      //          set_src_duration_ms,
      //          img_path);
      lv_obj_invalidate(s_safety_image);

      // LVGL은 이미지를 비동기로 디코딩하므로,
      // 명시적인 디코딩 루프가 필요 없음
      // 이미지를 설정하고 invalidate만 하면
      // LVGL이 자동으로 백그라운드에서 디코딩함
      // 하트비트 업데이트 (이미지 설정 후)
      update_heartbeat_lvgl();

      // Show image (LVGL이 백그라운드에서
      // 디코딩하는 동안에도 표시 가능)
      if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_NAVI) {
        lv_obj_clear_flag(s_safety_image, LV_OBJ_FLAG_HIDDEN);
      } else if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER) {
        if (s_speedometer_safety_image != NULL) {
          lv_img_set_src(s_speedometer_safety_image, img_path);
          lv_obj_clear_flag(s_speedometer_safety_image, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(s_speedometer_safety_image);
        }

        if (s_speedometer_safety_arc != NULL) {
          int limit_speed = data2 > 220 ? 220 : data2;
          // 어린이 보호구역(children.png)일 경우 속도 제한을 30km/h로 고정
          if (strcmp(img_filename, "children.png") == 0) {
            limit_speed = 30;
          }
          s_speedometer_safety_tt_val = (uint8_t)limit_speed;

          ESP_LOGI(TAG, "Speedometer Arc Update: data2=%u, limit_speed=%d, current_sub_mode=%d", 
                   data2, limit_speed, s_guide_sub_mode);

          if (limit_speed > 0) {
            int start_angle = (int)((limit_speed / 220.0) * 228.0);
            ESP_LOGI(TAG, "Setting Safety Arc Start Angle: %d (limit: %d km/h)", start_angle, limit_speed);
            
            // 시인성 확보를 위해 두께와 색상을 강제 재설정 (주황색/10px로 복구)
            lv_obj_set_style_arc_width(s_speedometer_safety_arc, 10, LV_PART_INDICATOR);
            lv_obj_set_style_arc_color(s_speedometer_safety_arc, lv_color_hex(0xFF8800), LV_PART_INDICATOR);
            
            lv_arc_set_angles(s_speedometer_safety_arc, start_angle, 228);
            lv_obj_clear_flag(s_speedometer_safety_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(s_speedometer_safety_arc);
            lv_obj_invalidate(s_speedometer_safety_arc);
          } else {
            // 제한속도 표기가 없는 경우 180~220km/h 구간 표시
            int start_angle = (int)((180 / 220.0) * 228.0);
            lv_arc_set_angles(s_speedometer_safety_arc, start_angle, 228);
            lv_obj_clear_flag(s_speedometer_safety_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_invalidate(s_speedometer_safety_arc);
          }
        }

        // Hide speed and road name labels in speedometer mode when safety image
        // is active
        if (s_speedometer_speed_label)
          lv_obj_add_flag(s_speedometer_speed_label, LV_OBJ_FLAG_HIDDEN);
        if (s_speedometer_unit_label)
          lv_obj_add_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
        
        // 안내모드의 안전운행화면(속도계화면)에서는 도로명을 표기하지 않는다.
        if (s_speedometer_road_name_label)
          lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
        if (s_speedometer_road_name_sub_label)
          lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);

        // Also hide primary HUD road name
        if (s_road_name_label)
          lv_obj_add_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
        if (s_road_name_sub_label)
          lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
      }
      
      // HUD 모드일 때 도로명 숨기기 로직 제거 (안전운행 정보 표시 중에도 도로명 유지 요청)
      if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_NAVI) {
          // 구간속도가 보이고 있다면 도로명 숨겨야 함 (공간 중첩)
          bool avr_visible = s_avr_speed_value_label && !lv_obj_has_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);
          if (avr_visible) {
              if (s_road_name_label) lv_obj_add_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
              if (s_road_name_sub_label) lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
          } else {
              // 구간속도가 없으면 도로명 표시 (안전운행 중이어도)
              if (s_road_name_label && lv_label_get_text(s_road_name_label)[0] != '\0') {
                  lv_obj_clear_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
                  if (s_road_name_sub_label && lv_label_get_text(s_road_name_sub_label)[0] != '\0') {
                      lv_obj_clear_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
                  }
                  if (s_speed_mark_unit_label) lv_obj_add_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
              }
          }
      }

      // 하트비트 업데이트 (이미지 표시 후)
      update_heartbeat_lvgl();

      ESP_LOGI(TAG, "Safety_DRV: Image file=%s loaded (decoding in background)",
               img_filename);
      ESP_LOGD(TAG,
               "Safety_DRV: Image positioned at sector=%u (offset_x=%d, "
               "offset_y=%d)",
               entry->sector, offset_x, offset_y);
    } else {
      // Image path unchanged, just ensure image
      // is visible and update properties
      if (s_safety_image != NULL) {
        lv_obj_align(s_safety_image, LV_ALIGN_CENTER, offset_x, offset_y);
      }

      if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_NAVI) {
        if (s_safety_image != NULL) {
          lv_obj_clear_flag(s_safety_image, LV_OBJ_FLAG_HIDDEN);
        }
      } else if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER) {
        if (s_speedometer_safety_image != NULL) {
          lv_obj_clear_flag(s_speedometer_safety_image, LV_OBJ_FLAG_HIDDEN);
        }

        if (s_speedometer_safety_arc != NULL) {
          int limit_speed = data2 > 220 ? 220 : data2;
          // 어린이 보호구역(children.png)일 경우 속도 제한을 30km/h로 고정
          if (strcmp(img_filename, "children.png") == 0) {
            limit_speed = 30;
          }
          s_speedometer_safety_tt_val = (uint8_t)limit_speed;

          ESP_LOGI(TAG, "Speedometer Arc (Else) Update: data2=%u, limit_speed=%d", data2, limit_speed);

          if (limit_speed > 0) {
            int start_angle = (int)((limit_speed / 220.0) * 228.0);
            ESP_LOGI(TAG, "Setting Safety Arc Start Angle: %d (limit: %d km/h)", start_angle, limit_speed);
            
            // 시인성 확보를 위해 두께와 색상을 강제 재설정 (주황색/10px로 복구)
            lv_obj_set_style_arc_width(s_speedometer_safety_arc, 10, LV_PART_INDICATOR);
            lv_obj_set_style_arc_color(s_speedometer_safety_arc, lv_color_hex(0xFF8800), LV_PART_INDICATOR);
            
            lv_arc_set_angles(s_speedometer_safety_arc, start_angle, 228);
            lv_obj_clear_flag(s_speedometer_safety_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(s_speedometer_safety_arc);
            lv_obj_invalidate(s_speedometer_safety_arc);
          } else {
            // 제한속도 표기가 없는 경우 180~220km/h 구간 표시
            int start_angle = (int)((180 / 220.0) * 228.0);
            lv_arc_set_angles(s_speedometer_safety_arc, start_angle, 228);
            lv_obj_clear_flag(s_speedometer_safety_arc, LV_OBJ_FLAG_HIDDEN);
            lv_obj_invalidate(s_speedometer_safety_arc);
          }
        }

        if (s_speedometer_speed_label)
          lv_obj_add_flag(s_speedometer_speed_label, LV_OBJ_FLAG_HIDDEN);
        if (s_speedometer_unit_label)
          lv_obj_add_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
      }
      ESP_LOGD(TAG, "Safety_DRV: Image path unchanged, skipping reload: %s",
               img_path);
    }
  } // end if (!ring_only_mode)

  // Set outer ring color based on data3 (OTA 제외)
  if (s_current_mode == DISPLAY_MODE_OTA) {
    // OTA: 모든 외곽링 숨김
    if (s_circle_ring != NULL)
      lv_obj_add_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
  } else {
    if (data3 == 1) {
      // 외곽 링 객체가 없으면 생성
      ensure_circle_ring_created();
      
      if (s_circle_ring != NULL) {
        // 빨강링 점멸 시작
        if (s_safety_ring_timer == NULL) {
          ESP_LOGI(TAG, "Safety Ring FLASH START: mode=%d, d3=%d", s_current_mode, data3);
          s_safety_ring_flash_count = 1; // 첫 번째 상태 (빨강)
          s_safety_ring_timer = lv_timer_create(safety_ring_timer_cb, 200, NULL);

          lv_obj_set_style_border_color(s_circle_ring, lv_color_hex(0xFF0000), 0);
          lv_obj_set_style_border_width(s_circle_ring, 10, 0); // Red = 10pt
          lv_obj_set_size(s_circle_ring, 461, 461);           // Red diameter = 461
          lv_obj_center(s_circle_ring);
          lv_obj_clear_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(s_circle_ring); // 최상위로
          lv_obj_invalidate(s_circle_ring);
          lv_refr_now(NULL); // 즉시 갱신 강제
        } else {
          // 이미 동작 중이면 리셋하여 다시 2회 반복 시작
          ESP_LOGI(TAG, "Safety Ring FLASH RESET: mode=%d", s_current_mode);
          lv_timer_reset(s_safety_ring_timer);
          s_safety_ring_flash_count = 1;
          lv_obj_set_style_border_color(s_circle_ring, lv_color_hex(0xFF0000), 0);
          lv_obj_set_style_border_width(s_circle_ring, 10, 0); // Red = 10pt
          lv_obj_set_size(s_circle_ring, 461, 461);
          lv_obj_center(s_circle_ring);
          lv_obj_move_foreground(s_circle_ring); 
          lv_obj_invalidate(s_circle_ring);
          lv_refr_now(NULL); // 즉시 갱신 강제
        }
      }
    } else if (s_circle_ring != NULL) {
      // 즉시 중지 및 이전 GPS 상태 링 표시 (data3 = 0 또는 다른 값)
      if (s_safety_ring_timer != NULL) {
        lv_timer_del(s_safety_ring_timer);
        s_safety_ring_timer = NULL;
      }
      if (s_last_gps_status == 0x00) {
        lv_obj_set_style_border_color(s_circle_ring, lv_color_hex(0x0000FF),
                                      0); // Blue
      } else {
        lv_obj_set_style_border_color(s_circle_ring, lv_color_hex(0x00FF00),
                                      0); // Green
      }
      lv_obj_set_style_border_width(s_circle_ring, 5, 0); // Revert to 5pt for GPS status
      lv_obj_set_size(s_circle_ring, 463, 463);           // GPS diameter = 463
      lv_obj_center(s_circle_ring);

      // GPS 링을 숨기는 모드이거나 연결이 끊겼을 때(단, 가상 모드 제외), 과속 경고 점멸 중이 아닐 때만 숨김
      if ((s_current_mode != DISPLAY_MODE_GUIDE || (!s_connected && !s_virt_drive_active)) && s_safety_ring_timer == NULL) {
        lv_obj_add_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
      } else if (s_connected || s_virt_drive_active) {
        lv_obj_clear_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
      }
      lv_obj_invalidate(s_circle_ring);
    }
  }

  // Display safety distance (data4, data5,
  // data6) Calculate distance in meters from
  // data4, data5, data6 각 바이트는 16진수
  // 자리를 표현하며 이를 10진수로 변환하여
  // safety거리값으로 사용 data4 is high byte,
  // data5 is middle byte, data6 is low byte
  uint32_t distance_m =
      ((uint32_t)data4 << 16) | ((uint32_t)data5 << 8) | (uint32_t)data6;

  static uint32_t s_last_distance_m = 0xFFFFFFFF;
  static uint8_t s_last_distance_mode = 0xFF;

  // safety거리값이 20 이하이거나 10000m 이상(10km)인 경우 표기하지 않음
  if (distance_m <= 20 || distance_m >= 10000) {
    if (s_safety_length_value_label != NULL) {
      lv_obj_add_flag(s_safety_length_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_safety_length_unit_label != NULL) {
      lv_obj_add_flag(s_safety_length_unit_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_safety_value_label != NULL) {
      lv_obj_add_flag(s_speedometer_safety_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_safety_unit_label != NULL) {
      lv_obj_add_flag(s_speedometer_safety_unit_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_safety_arc != NULL) {
      // 거리가 가까워져서 텍스트를 지울 때도, 실제 제한 속도(tt_val)가 있다면 해당 구간 표시 유지
      int limit = (s_speedometer_safety_tt_val > 0) ? s_speedometer_safety_tt_val : 180;
      int start_angle = (int)((limit / 220.0) * 228.0);
      
      // 제한 속도가 활성 상태일 때도 주황색/10px 유지
      if (s_speedometer_safety_tt_val > 0) {
          lv_obj_set_style_arc_width(s_speedometer_safety_arc, 10, LV_PART_INDICATOR);
          lv_obj_set_style_arc_color(s_speedometer_safety_arc, lv_color_hex(0xFF8800), LV_PART_INDICATOR);
      } else {
          lv_obj_set_style_arc_width(s_speedometer_safety_arc, 10, LV_PART_INDICATOR);
          lv_obj_set_style_arc_color(s_speedometer_safety_arc, lv_color_hex(0xFF8800), LV_PART_INDICATOR);
      }
      
      lv_arc_set_angles(s_speedometer_safety_arc, start_angle, 228);
      lv_obj_clear_flag(s_speedometer_safety_arc, LV_OBJ_FLAG_HIDDEN);
      lv_obj_invalidate(s_speedometer_safety_arc);
    }
    s_last_distance_m = 0xFFFFFFFF;
    return;
  }

  if (distance_m == s_last_distance_m &&
      s_current_mode == s_last_distance_mode) {
    return;
  }
  s_last_distance_m = distance_m;
  s_last_distance_mode = s_current_mode;

  // Check if HUD labels exist (always expected at boot)
  if (s_safety_length_value_label == NULL ||
      s_safety_length_unit_label == NULL) {
    ESP_LOGW(TAG, "Safety_DRV: HUD safety length labels not created yet");
    // Continue anyway to allow speedometer mode labels to update
  }

  // safety거리값 표시: 1000 미만은 미터, 1000
  // 이상은 킬로미터로 변환 1000=1Km,
  // 1500=1.5km, 900=900m 형식으로 표시
  char value_text[32];
  char unit_text[8];
  if (distance_m < 1000) {
    // 1000 미만: 미터 단위로 그대로 표시 (예:
    // 900m)
    snprintf(value_text, sizeof(value_text), "%lu", (unsigned long)distance_m);
    snprintf(unit_text, sizeof(unit_text), "m");
  } else {
    // 1000 이상: 킬로미터로 변환하여 소수점
    // 한 자리까지 표시 (예: 1Km, 1.5Km)
    float distance_km = distance_m / 1000.0f;
    // 정수인지 확인 (소수점이 0인지)
    if (distance_km == (float)((uint32_t)distance_km)) {
      // 정수인 경우 (예: 1000 = 1Km)
      snprintf(value_text, sizeof(value_text), "%lu",
               (unsigned long)((uint32_t)distance_km));
    } else {
      // 소수점이 있는 경우 소수점 한 자리까지
      // 표시 (예: 1500 = 1.5Km)
      snprintf(value_text, sizeof(value_text), "%.1f", distance_km);
    }
    snprintf(unit_text, sizeof(unit_text), "Km");
  }

  // Update safety거리값 label (25pt, 녹색)
  // 위치: LCD 중앙에서 우측으로 130pt, 위로 9pt 이동
  // 거리계산기준: 마지막 숫자 중앙이 X=130pt
  lv_label_set_text(s_safety_length_value_label, value_text);
  lv_obj_set_style_text_color(s_safety_length_value_label,
                              lv_color_hex(0x00FF00),
                              0); // 녹색
  lv_obj_set_style_text_font(s_safety_length_value_label, &font_kopub_35,
                             0); // 35pt 폰트로 상향 (기존 25pt)

  // Get total width and last char width for alignment
  lv_coord_t value_width =
      lv_txt_get_width(value_text, strlen(value_text), &font_kopub_35, 0,
                       LV_TEXT_FLAG_NONE); // 35pt 기준으로 너비 계산
  size_t val_len = strlen(value_text);
  lv_coord_t last_char_width = 0;
  if (val_len > 0) {
    char last_char_str[2] = {value_text[val_len - 1], '\0'};
    last_char_width =
        lv_txt_get_width(last_char_str, 1, &font_kopub_35, 0,
                         LV_TEXT_FLAG_NONE); // 35pt 기준으로 너비 계산
  }

  // Align: center of last digit at X=130, 위로 9pt (Y=-9)
  lv_obj_align(s_safety_length_value_label, LV_ALIGN_CENTER,
               130 + (last_char_width / 2) - (value_width / 2), -9);
  lv_obj_clear_flag(s_safety_length_value_label, LV_OBJ_FLAG_HIDDEN);

  // Update safety단위 label (25pt, 회색)
  // 위치: safety거리값 오른쪽 끝 숫자에서 5pt 우로 이동
  if (s_safety_length_unit_label != NULL) {
    lv_label_set_text(s_safety_length_unit_label, unit_text);
    lv_obj_set_style_text_color(s_safety_length_unit_label,
                                lv_color_hex(0xCCCCCC),
                                0); // 회색
    lv_obj_set_style_text_font(s_safety_length_unit_label, &font_kopub_25,
                               0); // 25pt 폰트
    lv_obj_align_to(s_safety_length_unit_label, s_safety_length_value_label,
                    LV_ALIGN_OUT_RIGHT_MID, 5, 0); // 값 레이블 우측 5pt
    lv_obj_clear_flag(s_safety_length_unit_label, LV_OBJ_FLAG_HIDDEN);
  }

  // Speedometer Mode Labels update
  if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER) {
    if (s_speedometer_safety_value_label != NULL) {
      lv_label_set_text(s_speedometer_safety_value_label, value_text);

      // Calculate width for last digit center alignment (using kopub_35 to match HUD)
      lv_coord_t v_width = lv_txt_get_width(
          value_text, strlen(value_text), &font_kopub_35, 0, LV_TEXT_FLAG_NONE);
      size_t v_len = strlen(value_text);
      lv_coord_t v_last_char_w = 0;
      if (v_len > 0) {
        char last_char_str[2] = {value_text[v_len - 1], '\0'};
        v_last_char_w = lv_txt_get_width(last_char_str, 1, &font_kopub_35, 0,
                                         LV_TEXT_FLAG_NONE);
      }

      // Align: center of last digit at X=110, 아래로 146pt (Y=146) - Safety 이미지 우측 하단 정렬
      lv_obj_align(s_speedometer_safety_value_label, LV_ALIGN_CENTER,
                   110 + (v_last_char_w / 2) - (v_width / 2), 146);
      lv_obj_set_style_text_color(s_speedometer_safety_value_label, lv_color_hex(0x00FF00), 0); // 초록색

      lv_obj_clear_flag(s_speedometer_safety_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_safety_unit_label != NULL) {
      lv_label_set_text(s_speedometer_safety_unit_label, unit_text);
      lv_obj_align_to(s_speedometer_safety_unit_label,
                      s_speedometer_safety_value_label, LV_ALIGN_OUT_RIGHT_MID,
                      6, 0);
      lv_obj_clear_flag(s_speedometer_safety_unit_label, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

// Safety_DRV display function (called from
// BLE callback) 참고파일:
// "littlefs/safety_drv.csv" data 형식:
// "start, ID, commend, data_length, data1,
// data2, data3, data4, data5, data6, end"
// start, id, comment, data_length, data1이
// 일치하면, 이 함수를 실행한다. 이미지파일은
// "littlefs/image/"에 있다. csv "image"항에
// 내용중 tt는 "data2"를 10진수로 치환하여
// 이미지파일명의 tt에 적용하여 해당 이미지를
// 사용한다. "sector"를 참고하여 LCD에
// 표기한다. 해당 이미지파일 호출과 표기는
// 표기하고있는 이미지가 없거나 변경되었을때만
// 화면을 갱신한다. "data3"가 1이면 LCD외곽에
// 빨강링을 표시하고, 0이면 녹색링을 표시한다.
// data4, data5, data6는 16진수 미터단위의
// 정보다. 각 바이트는 16진수 자리를 표현하며
// 이를 10진수로 변환하여 "safety거리값"으로
// 사용한다. 폰트크기: "safety거리값"은 40pt,
// "safety단위"는 20pt "safety거리값"표시는
// LCD중앙에서 우측으로 110pt 이동, 위로 110pt
// 이동하여 표기한다. "safety단위"표시는
// LCD중앙에서 우측으로 160pt 이동, 위로 110pt
// 이동하여 표기한다. "safety단위"는
// "safety거리값"이 1000보다 작으면 "m", 1000
// 이상이면"Km"로 표기한다. "safety거리값"과
// "safety단위"는 1000=1Km, 1500=1.5km,
// 900=900m 이렇게 표시해줘..
// "safety거리값"색상은 주황색으로 한다.
// "safety단위" 색상은 회색으로 한다. 단,
// "safety거리값"이 20 이하이면
// 표기하지않는다.
static void safety_drive(uint8_t start, uint8_t id, uint8_t commend,
                         uint8_t data_length, uint8_t data1, uint8_t data2,
                         uint8_t data3, uint8_t data4, uint8_t data5,
                         uint8_t data6) {
  /* ESP_LOGI(TAG,
           "safety_drive: called with
     start=0x%02X id=0x%02X cmd=0x%02X "
           "dlen=0x%02X data1=0x%02X
     data2=0x%02X data3=0x%02X data4=0x%02X "
           "data5=0x%02X data6=0x%02X",
           start, id, commend, data_length,
     data1, data2, data3, data4, data5,
           data6); */

  // data1 == 2: 외곽링을 제외한 LCD 에 표기한
  // 모든것을 화면에 표기하지 않는다
  ESP_LOGI(TAG, "Safety_DRV Packet: start=0x%02X id=0x%02X cmd=0x%02X d1=0x%02X d2=0x%02X d3=0x%02X", 
           start, id, commend, data1, data2, data3);

  if (data1 == 0x02) {
    ESP_LOGI(TAG, "Safety_DRV: data1=2 received, "
                  "requesting global clear");
    request_clear_display(0x02);
    return;
  }

  // Find matching Safety_DRV entry
  const safety_data_entry_t *entry =
      find_safety_image_entry(start, id, commend, data_length, data1);
  if (entry == NULL) {
    ESP_LOGW(TAG,
             "Safety_DRV: No matching entry for "
             "start=0x%02X id=0x%02X "
             "cmd=0x%02X dlen=0x%02X data1=0x%02X",
             start, id, commend, data_length, data1);
    return;
  }

  ESP_LOGD(TAG,
           "Safety_DRV: Found matching "
           "entry, image=%s sector=%u",
           entry->image_filename, entry->sector);

  // Request all updates via queue (to avoid
  // LVGL crash in BLE callback) Image, ring
  request_safety_update(start, id, commend, data_length, data1, data2, data3,
                        data4, data5, data6);
}

// Circle drawing function (외곽 링 GPS 상태
// 표시) Format: 19 4D 04 01 data1 2F
// - start=0x19, id=0x4D, commend=0x04,
// data_length=0x01이 일치하면 실행
// - data1=0x00: 외곽 링을 파랑색으로 (GPS
// 수신 불가)
// - data1=0x01: 외곽 링을 녹색으로 (GPS 수신
// 가능) Request circle update (called from
// BLE callback - queues request to LVGL task)
static void request_circle_update(uint8_t start, uint8_t id, uint8_t commend,
                                  uint8_t data_length, uint8_t data1) {
  if (s_circle_update_queue == NULL) {
    ESP_LOGW(TAG, "Circle update queue not "
                  "initialized, cannot "
                  "request circle update");
    return;
  }

  circle_update_request_t req = {.start = start,
                                 .id = id,
                                 .commend = commend,
                                 .data_length = data_length,
                                 .data1 = data1};

  // Filter duplicate requests to prevent
  // queue overflow
  if (s_last_circle_request_valid) {
    if (memcmp(&req, &s_last_circle_request, sizeof(circle_update_request_t)) ==
        0) {
      // Same request as last one, skip to
      // prevent queue overflow
      return;
    }
  }

  memcpy(&s_last_circle_request, &req, sizeof(circle_update_request_t));
  s_last_circle_request_valid = true;

  // Non-blocking queue send to prevent BLE
  // callback blocking
  if (xQueueSend(s_circle_update_queue, &req, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Circle update queue full, dropping "
                  "request (non-blocking)");
  }
}

// Update circle display (called from LVGL
// task - actual LVGL object manipulation)
static void update_circle_display(uint8_t start, uint8_t id, uint8_t commend,
                                  uint8_t data_length, uint8_t data1) {
  // OTA 모드: 모든 외곽링 숨김
  if (s_current_mode == DISPLAY_MODE_OTA) {
    if (s_circle_ring != NULL)
      lv_obj_add_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  // Ensure object exists first
  ensure_circle_ring_created();

  // CLOCK/CLOCK2/ALBUM/SETTING: GPS 링(GPS 상태) 숨김
  // 연결이 끊겼거나 Safety 빨강 점멸 타이머 실행 중이면 건드리지 않음
  // [User Request] 가상 운행 모드(DISPLAY_MODE_GUIDE + virt_active)에서는 허용
  if (s_current_mode != DISPLAY_MODE_GUIDE || (!s_connected && !s_virt_drive_active)) {
    if (s_safety_ring_timer == NULL && s_circle_ring != NULL) {
      lv_obj_add_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
    }
    return;
  }

  // Hide black screen overlay if visible
  // (clear_display 후 다시 표시될 때)
  hide_black_screen_overlay();

  ESP_LOGI(TAG,
           "circle_dwg called: start=0x%02X "
           "id=0x%02X commend=0x%02X "
           "dlen=0x%02X data1=0x%02X",
           start, id, commend, data_length, data1);

  if (start != 0x19 || id != 0x4D || commend != 0x04 || data_length != 0x01) {
    ESP_LOGW(TAG, "circle_dwg: Invalid command format");
    return;
  }

  // Set border color based on data1
  s_last_gps_status = data1;
  if (data1 == 0x00) {
    // Blue border (GPS 검색 중 / 수신 전)
    lv_obj_set_style_border_color(s_circle_ring, lv_color_hex(0x0000FF),
                                  0);                   // 파랑색
    lv_obj_set_style_border_width(s_circle_ring, 5, 0); // 파랑색 = 5pt
    lv_obj_set_size(s_circle_ring, 463, 463);           // 지름 = 463
    lv_obj_center(s_circle_ring);
    ESP_LOGI(TAG, "Circle ring: Blue (GPS 검색 중)");
  } else if (data1 == 0x01) {
    // Green border (GPS 수신 완료/정상)
    lv_obj_set_style_border_color(s_circle_ring, lv_color_hex(0x00FF00),
                                  0);                   // 녹색 (최대 밝기)
    lv_obj_set_style_border_width(s_circle_ring, 5, 0); // 녹색 = 5pt
    lv_obj_set_size(s_circle_ring, 463, 463);           // 지름 = 463
    lv_obj_center(s_circle_ring);
    ESP_LOGI(TAG, "Circle ring: Green (GPS 수신 완료)");
  } else if (data1 == 0x02) {
    // Grey border (Initial / Ready)
    lv_obj_set_style_border_color(s_circle_ring, lv_color_hex(0x808080),
                                  0);                   // 회색
    lv_obj_set_style_border_width(s_circle_ring, 5, 0); // 회색 = 5pt
    lv_obj_set_size(s_circle_ring, 463, 463);           // 지름 = 463
    lv_obj_center(s_circle_ring);
    ESP_LOGI(TAG, "Circle ring: Grey (Initial Ready)");
  } else {
    ESP_LOGW(TAG,
             "circle_dwg: Unknown data1 "
             "value: 0x%02X",
             data1);
    return;
  }

  lv_obj_clear_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
  lv_obj_invalidate(s_circle_ring);
}

// Safety_DRV 링 점멸 타이머 콜백 (0.2초 간격 2회 반복)
static void safety_ring_timer_cb(lv_timer_t *timer) {
  if (s_circle_ring == NULL) {
    s_safety_ring_timer = NULL;
    lv_timer_del(timer);
    return;
  }

  s_safety_ring_flash_count++;

  // 4번째 상태(두 번째 OFF)까지 완료되면 중지
  // 1:Red(시작), 2:Original, 3:Red, 4:Original(끝)
  if (s_safety_ring_flash_count >= 4) {
    // 최종적으로 기존 색상 복구
    lv_color_t original_color = lv_color_hex(0x00FF00);
    if (s_last_circle_request_valid) {
      if (s_last_circle_request.data1 == 0x00) {
        original_color = lv_color_hex(0x0000FF);
      } else if (s_last_circle_request.data1 == 0x01) {
        original_color = lv_color_hex(0x00FF00);
      } else if (s_last_circle_request.data1 == 0x02) {
        original_color = lv_color_hex(0x808080);
      }
    }
    lv_obj_set_style_border_color(s_circle_ring, original_color, 0);
    // Restore Original Width - Back to 5pt for all
    lv_obj_set_style_border_width(s_circle_ring, 5, 0); // 5pt로 복구
    lv_obj_set_size(s_circle_ring, 463, 463);           // 463pt로 복구
    lv_obj_center(s_circle_ring);
    // GPS 링 숨김 모드에서는 타이머 종료 후 링 숨김
    // (CLOCK/CLOCK2/ALBUM/SETTING/VIRTUAL_DRIVE/OTA)
    if (s_current_mode != DISPLAY_MODE_GUIDE) {
      lv_obj_add_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_invalidate(s_circle_ring);

    s_safety_ring_timer = NULL;
    lv_timer_del(timer);
    return;
  }

  // 홀수 번째는 빨강, 짝수 번째는 기존 색상
  if (s_safety_ring_flash_count % 2 == 1) {
    lv_obj_set_style_border_color(s_circle_ring, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_border_width(s_circle_ring, 10, 0); // 빨강일 때 10pt
    lv_obj_set_size(s_circle_ring, 461, 461);           // 빨강일 때 지름 461pt
    lv_obj_center(s_circle_ring);
  } else {
    lv_color_t original_color = lv_color_hex(0x00FF00);
    if (s_last_circle_request_valid) {
      if (s_last_circle_request.data1 == 0x00) {
        original_color = lv_color_hex(0x0000FF);
      } else if (s_last_circle_request.data1 == 0x01) {
        original_color = lv_color_hex(0x00FF00);
      } else if (s_last_circle_request.data1 == 0x02) {
        original_color = lv_color_hex(0x808080);
      }
    }
    lv_obj_set_style_border_color(s_circle_ring, original_color, 0);
    lv_obj_set_style_border_width(s_circle_ring, 5, 0); // 일반 상태 5pt
    lv_obj_set_size(s_circle_ring, 463, 463);           // 일반 상태 463pt
    lv_obj_center(s_circle_ring);
  }
  lv_obj_invalidate(s_circle_ring);
}

// Legacy function name - redirects to request
// function for backward compatibility
/* static void circle_dwg(uint8_t start,
uint8_t id, uint8_t commend, uint8_t
data_length, uint8_t data1) {
  request_circle_update(start, id, commend,
data_length, data1);
} */

// 외곽 링 객체가 없으면 생성하는 헬퍼 함수
static void ensure_circle_ring_created(void) {
  if (s_circle_ring == NULL) {
    s_circle_ring = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_circle_ring, 463, 463);           // 기본 지름 463pt
    // 완전한 원이 되도록 radius를 원형으로 설정
    lv_obj_set_style_radius(s_circle_ring, LV_RADIUS_CIRCLE, 0);
    // 배경은 투명, 테두리만 표시
    lv_obj_set_style_bg_opa(s_circle_ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_circle_ring, 5, 0); // 기본 두께 5pt
    lv_obj_set_style_border_opa(s_circle_ring, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_circle_ring, 0, 0);
    lv_obj_set_scrollbar_mode(s_circle_ring, LV_SCROLLBAR_MODE_OFF);
    lv_obj_center(s_circle_ring); // 화면 중앙 기준 원
    lv_obj_add_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN); // 초기에는 숨김
  }
}

static void hide_black_screen_overlay(void) {
  if (s_black_screen_overlay != NULL) {
    lv_obj_add_flag(s_black_screen_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(s_black_screen_overlay);
  }
}

// 화면 지우기 함수
// Format: 19 4D 05 01 data1 2F
// start=0x19, id=0x4D, commend=0x05,
// data_length=0x01이 일치하면 실행 data1=1:
// TBT방향표시 함수에서 LCD에 표기한 모든것
// 지우기 (TBT 이미지, TBT계산값, TBT단위)
// data1=2: safety_drive 함수에서 LCD에 표기한
// 모든것 지우기 (Safety 이미지, safety거리값,
// safety단위) data1=7: LCD 전체 화면을
// 검정색으로 채우기 data1=8: 목적지정보
// 함수에서 LCD에 표기한 모든것 지우기
// (목적지남은시간값, 목적지남은거리값)
// Request clear display (called from BLE
// callback - queues request to LVGL task)
static void request_clear_display(uint8_t data1) {
  if (s_clear_display_queue == NULL) {
    ESP_LOGW(TAG, "Clear display queue not "
                  "initialized, cannot "
                  "request clear display");
    return;
  }

  clear_display_request_t req = {.data1 = data1};

  // No filter for clear display requests to
  // ensure immediate action (특히 safety
  // 이미지가 안 지워지는 현상을 방지하기 위해
  // 필터링 제거)

  memcpy(&s_last_clear_display_request, &req, sizeof(clear_display_request_t));
  s_last_clear_display_request_valid = true;

  // Non-blocking queue send to prevent BLE
  // callback blocking
  if (xQueueSend(s_clear_display_queue, &req, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Clear display queue full, dropping "
                  "request (non-blocking)");
  }
}

static void align_avr_speed_labels(void);

// Update clear display (called from LVGL task
// - actual LVGL object manipulation)
static void update_clear_display(uint8_t data1) {
  ESP_LOGI(TAG, "clear_display called: data1=0x%02X", data1);

  // Speedometer sub-mode ignore 0x07 (Full Black)
  if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER && data1 == 0x07) {
    ESP_LOGI(TAG, "clear_display: Speedometer mode - ignoring data1=0x07");
    return;
  }

  // 0x07(전체 화면 블랙)을 제외한 요청의 경우
  // 블랙 오버레이를 숨김
  if (data1 != 0x07) {
    hide_black_screen_overlay();
  }

  // TBT방향표시 지우기 로직
  bool do_tbt =
      (data1 == 0x01 || data1 == 0x03 || data1 == 0x05 || data1 == 0x07);
  // safety_drive 지우기 로직
  bool do_safety =
      (data1 == 0x02 || data1 == 0x03 || data1 == 0x06 || data1 == 0x07);
  // 일반속도(speed mark) 지우기 로직
  bool do_normal_speed =
      (data1 == 0x04 || data1 == 0x05 || data1 == 0x06 || data1 == 0x07);
  // 구간속도 지우기 로직
  bool do_avr_speed =
      (data1 == 0x02 || data1 == 0x03 || data1 == 0x06 || data1 == 0x07);
  // 목적지정보 지우기 로직
  bool do_dest = (data1 == 0x07 || data1 == 0x08);

  if (do_tbt) {
    if (s_hud_image != NULL) {
      lv_obj_add_flag(s_hud_image, LV_OBJ_FLAG_HIDDEN);
      // lv_img_set_src(s_hud_image, NULL); // Do NOT clear src to preserve LVGL
      // image cache
    }
    if (s_length_tbt_value_label != NULL) {
      lv_obj_add_flag(s_length_tbt_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_length_tbt_unit_label != NULL) {
      lv_obj_add_flag(s_length_tbt_unit_label, LV_OBJ_FLAG_HIDDEN);
    }
    s_last_image_request_valid = false;
  }

  if (do_safety) {
    if (s_safety_image != NULL) {
      lv_obj_add_flag(s_safety_image, LV_OBJ_FLAG_HIDDEN);
      // lv_img_set_src(s_safety_image, NULL); // Do NOT clear src to preserve
      // LVGL image cache
      ESP_LOGI(TAG, "update_clear_display: "
                    "safety_image hidden and cleared");
    }
    if (s_safety_length_value_label != NULL) {
      lv_obj_add_flag(s_safety_length_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_safety_length_unit_label != NULL) {
      lv_obj_add_flag(s_safety_length_unit_label, LV_OBJ_FLAG_HIDDEN);
    }

    // Clear speedometer mode safety elements
    if (s_speedometer_safety_image != NULL) {
      lv_obj_add_flag(s_speedometer_safety_image, LV_OBJ_FLAG_HIDDEN);
      // lv_img_set_src(s_speedometer_safety_image, NULL); // Preserve cache
    }
    if (s_speedometer_safety_value_label != NULL) {
      lv_obj_add_flag(s_speedometer_safety_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_safety_unit_label != NULL) {
      lv_obj_add_flag(s_speedometer_safety_unit_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_safety_arc != NULL) {
      // 클리어 요청 시에도 현재 설정된 제한 속도가 있으면 그 값을 유지, 없으면 180km 기본값
      int limit = (s_speedometer_safety_tt_val > 0) ? s_speedometer_safety_tt_val : 180;
      int start_angle = (int)((limit / 220.0) * 228.0);
      
      // 안내가 종료되는 상황이므로 원래의 주황색/10px 스타일로 복구
      lv_obj_set_style_arc_width(s_speedometer_safety_arc, 10, LV_PART_INDICATOR);
      lv_obj_set_style_arc_color(s_speedometer_safety_arc, lv_color_hex(0xFF8800), LV_PART_INDICATOR);
      
      lv_arc_set_angles(s_speedometer_safety_arc, start_angle, 228);
      lv_obj_clear_flag(s_speedometer_safety_arc, LV_OBJ_FLAG_HIDDEN);
      lv_obj_invalidate(s_speedometer_safety_arc);
    }

    // Show speedometer mode speed labels when safety image is cleared
    if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER) {
      if (s_speedometer_speed_label != NULL) {
        lv_obj_clear_flag(s_speedometer_speed_label, LV_OBJ_FLAG_HIDDEN);
      }
      
      // 안내모드의 안전운행화면(속도계화면)에서는 도로명을 표기하지 않는다. (단위 km/h 항상 표시)
      if (s_speedometer_road_name_label) lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
      if (s_speedometer_road_name_sub_label) lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
      if (s_speedometer_unit_label) {
        bool avr_visible = (s_speedometer_avr_speed_value_label != NULL && !lv_obj_has_flag(s_speedometer_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN));
        if (!avr_visible) {
          lv_obj_clear_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
        } else {
          lv_obj_add_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
        }
      }
    } else if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_NAVI) {
        // HUD 모드에서도 도로명이 있으면 복구
        bool road_name_active = (s_road_name_label != NULL && 
                                 lv_label_get_text(s_road_name_label)[0] != '\0');
        if (road_name_active) {
            bool avr_visible = s_avr_speed_value_label && !lv_obj_has_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);
            if (!avr_visible) {
                lv_obj_clear_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
                if (s_road_name_sub_label && lv_label_get_text(s_road_name_sub_label)[0] != '\0') {
                    lv_obj_clear_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
                }
                if (s_speed_mark_unit_label) lv_obj_add_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
    s_last_safety_request_valid = false;
  }

  if (do_normal_speed) {
    if (s_normal_speed_value_label != NULL) {
      lv_obj_add_flag(s_normal_speed_value_label, LV_OBJ_FLAG_HIDDEN);
      lv_obj_invalidate(s_normal_speed_value_label);
    }
    if (s_normal_speed_unit_label != NULL) {
      lv_obj_add_flag(s_normal_speed_unit_label, LV_OBJ_FLAG_HIDDEN);
      lv_obj_invalidate(s_normal_speed_unit_label);
    }
    
    // Also hide HUD speed marks
    if (s_speed_mark_value_label != NULL) {
      lv_obj_add_flag(s_speed_mark_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speed_mark_unit_label != NULL) {
      lv_obj_add_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
    }

    // 도로명도 같이 지우기 (유저 요청)
    if (s_road_name_label != NULL) {
      lv_obj_add_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_road_name_sub_label != NULL) {
      lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_road_name_label != NULL) {
      lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_road_name_sub_label != NULL) {
      lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    }
    // 도로명이 지워졌으므로 속도계 단위(km/h) 다시 표시 (단, 구간속도 표시 중이면 제외)
    if (s_speedometer_unit_label != NULL) {
      bool avr_visible = (s_speedometer_avr_speed_value_label != NULL && !lv_obj_has_flag(s_speedometer_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN));
      if (!avr_visible) {
        lv_obj_clear_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
      }
    }
  }

  if (do_avr_speed) {
    if (s_avr_speed_title_label != NULL) {
      lv_obj_add_flag(s_avr_speed_title_label, LV_OBJ_FLAG_HIDDEN);
      lv_obj_invalidate(s_avr_speed_title_label);
    }
    if (s_avr_speed_value_label != NULL) {
      lv_obj_add_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);
      lv_obj_invalidate(s_avr_speed_value_label);
    }
    if (s_avr_speed_unit_label != NULL) {
      lv_obj_add_flag(s_avr_speed_unit_label, LV_OBJ_FLAG_HIDDEN);
      lv_obj_invalidate(s_avr_speed_unit_label);
    }

    // Also clear Start/Average Speed labels in Speedometer mode
    if (s_speedometer_avr_speed_title_label != NULL) {
      lv_obj_add_flag(s_speedometer_avr_speed_title_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_avr_speed_value_label != NULL) {
      lv_obj_add_flag(s_speedometer_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_avr_speed_unit_label != NULL) {
      lv_obj_add_flag(s_speedometer_avr_speed_unit_label, LV_OBJ_FLAG_HIDDEN);
    }

    // 평균속도가 명시적으로 지워졌으므로 도로명 라벨 위치/가시성 즉시 복구
    align_avr_speed_labels();

    // 속도계 모드에서 구간속도가 사라졌으므로 km/h 단위 다시 표시 (안전안내 중이 아닐 때만)
    if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER) {
        bool safety_visible = (s_speedometer_safety_image != NULL && !lv_obj_has_flag(s_speedometer_safety_image, LV_OBJ_FLAG_HIDDEN));
        if (!safety_visible && s_speedometer_unit_label) {
            lv_obj_clear_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
        }
    }
  }

  if (do_dest) {
    if (s_dest_time_value_label != NULL) {
      lv_label_set_text(s_dest_time_value_label, "");
      lv_obj_add_flag(s_dest_time_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_dest_time_hour_unit_label != NULL) {
      lv_label_set_text(s_dest_time_hour_unit_label, "");
      lv_obj_add_flag(s_dest_time_hour_unit_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_dest_time_minute_value_label != NULL) {
      lv_label_set_text(s_dest_time_minute_value_label, "");
      lv_obj_add_flag(s_dest_time_minute_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_dest_time_unit_label != NULL) {
      lv_label_set_text(s_dest_time_unit_label, "");
      lv_obj_add_flag(s_dest_time_unit_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_dest_distance_value_label != NULL) {
      lv_label_set_text(s_dest_distance_value_label, "");
      lv_obj_add_flag(s_dest_distance_value_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_dest_distance_unit_label != NULL) {
      lv_label_set_text(s_dest_distance_unit_label, "");
      lv_obj_add_flag(s_dest_distance_unit_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_dest_image != NULL) {
      lv_obj_add_flag(s_dest_image, LV_OBJ_FLAG_HIDDEN);
      lv_img_set_src(s_dest_image, NULL);
    }
    if (s_dest_label != NULL) {
      lv_obj_add_flag(s_dest_label, LV_OBJ_FLAG_HIDDEN);
    }
    s_current_dest_image_path[0] = '\0';
  }

  if (data1 == 0x01) {
    ESP_LOGI(TAG, "clear_display: TBT방향표시 지움");
  } else if (data1 == 0x02) {
    ESP_LOGI(TAG, "clear_display: safety_drive(외곽링 "
                  "제외) 및 구간속도 지움");
  } else if (data1 == 0x03) {
    ESP_LOGI(TAG, "clear_display: TBT, "
                  "safety_drive, 구간속도 지움");
  } else if (data1 == 0x04) {
    ESP_LOGI(TAG, "clear_display: 일반속도(speed "
                  "mark) 지움");
  } else if (data1 == 0x05) {
    ESP_LOGI(TAG, "clear_display: TBT 및 "
                  "일반속도 지움");
  } else if (data1 == 0x06) {
    ESP_LOGI(TAG, "clear_display: safety_drive(외곽링 "
                  "제외), 구간속도, 일반속도 지움");
  } else if (data1 == 0x07) {
    ESP_LOGI(TAG, "clear_display: 외곽링 "
                  "제외 전체 화면 블랙");
    // 시간/날짜 라벨들도 숨김
    if (s_time_month_label)
      lv_obj_add_flag(s_time_month_label, LV_OBJ_FLAG_HIDDEN);
    if (s_time_month_unit_label)
      lv_obj_add_flag(s_time_month_unit_label, LV_OBJ_FLAG_HIDDEN);
    if (s_time_day_label)
      lv_obj_add_flag(s_time_day_label, LV_OBJ_FLAG_HIDDEN);
    if (s_time_day_unit_label)
      lv_obj_add_flag(s_time_day_unit_label, LV_OBJ_FLAG_HIDDEN);
    if (s_time_hour_label)
      lv_obj_add_flag(s_time_hour_label, LV_OBJ_FLAG_HIDDEN);
    if (s_time_colon1_label)
      lv_obj_add_flag(s_time_colon1_label, LV_OBJ_FLAG_HIDDEN);
    if (s_time_minute_label)
      lv_obj_add_flag(s_time_minute_label, LV_OBJ_FLAG_HIDDEN);
    if (s_time_colon2_label)
      lv_obj_add_flag(s_time_colon2_label, LV_OBJ_FLAG_HIDDEN);
    if (s_time_second_label)
      lv_obj_add_flag(s_time_second_label, LV_OBJ_FLAG_HIDDEN);

    // 블랙 오버레이 표시
    if (s_black_screen_overlay == NULL) {
      s_black_screen_overlay = lv_obj_create(lv_scr_act());
      lv_obj_set_size(s_black_screen_overlay, LCD_H_RES, LCD_V_RES);
      lv_obj_set_style_bg_color(s_black_screen_overlay, lv_color_hex(0x000000),
                                0);
      lv_obj_set_style_bg_opa(s_black_screen_overlay, LV_OPA_COVER, 0);
      lv_obj_set_style_border_width(s_black_screen_overlay, 0, 0);
      lv_obj_set_style_pad_all(s_black_screen_overlay, 0, 0);
      lv_obj_align(s_black_screen_overlay, LV_ALIGN_CENTER, 0, 0);
      lv_obj_clear_flag(s_black_screen_overlay, LV_OBJ_FLAG_SCROLLABLE);
    }
    lv_obj_clear_flag(s_black_screen_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_black_screen_overlay);

    // 외곽링은 블랙 오버레이보다 앞에 있어야
    // 함
    if (s_circle_ring != NULL) {
      lv_obj_move_foreground(s_circle_ring);
    }
  } else if (data1 == 0x08) {
    ESP_LOGI(TAG, "clear_display: 목적지정보 지움");
  } else {
    ESP_LOGW(TAG,
             "clear_display: Unknown data1 "
             "value: 0x%02X",
             data1);
  }
}

// Legacy function name - redirects to request
// function for backward compatibility
/* static void clear_display(uint8_t data1) {
 * request_clear_display(data1); } */

// Request speed update via queue (to avoid
// blocking in BLE callback)
static void request_speed_update(uint8_t start, uint8_t id, uint8_t commend,
                                 uint8_t data_length, uint8_t data1,
                                 uint8_t data2) {
  if (s_speed_update_queue == NULL) {
    ESP_LOGW(TAG, "Speed update queue not initialized, "
                  "cannot request speed update");
    return;
  }

  speed_update_request_t req = {.start = start,
                                .id = id,
                                .commend = commend,
                                .data_length = data_length,
                                .data1 = data1,
                                .data2 = data2};

  // Filter duplicate requests (same request
  // as last one)
  if (s_last_speed_request_valid) {
    if (memcmp(&req, &s_last_speed_request, sizeof(speed_update_request_t)) ==
        0) {
      // Same request as last one, skip to
      // prevent queue overflow
      return;
    }
  }

  // Save last request
  memcpy(&s_last_speed_request, &req, sizeof(speed_update_request_t));
  s_last_speed_request_valid = true;

  // Check queue usage and remove old items if
  // queue is >80% full to maintain buffer
  UBaseType_t queue_size = uxQueueMessagesWaiting(s_speed_update_queue);
  UBaseType_t queue_capacity =
      uxQueueSpacesAvailable(s_speed_update_queue) + queue_size;
  if (queue_capacity > 0) {
    float usage = (float)queue_size / (float)queue_capacity;
    const float CLEANUP_THRESHOLD = 0.80f; // 80% threshold
    if (usage >= CLEANUP_THRESHOLD) {
      // Remove old items until queue usage
      // drops to ~50% to maintain buffer
      speed_update_request_t dummy;
      UBaseType_t target_size = queue_capacity / 2; // Target 50% usage
      UBaseType_t items_to_remove =
          (queue_size > target_size) ? (queue_size - target_size) : 0;
      UBaseType_t removed_count = 0;
      for (UBaseType_t i = 0; i < items_to_remove; i++) {
        if (xQueueReceive(s_speed_update_queue, &dummy, 0) == pdTRUE) {
          removed_count++;
        } else {
          break; // No more items to remove
        }
      }
      if (removed_count > 0) {
        ESP_LOGW(TAG,
                 "Speed update queue "
                 "cleanup: removed %lu old "
                 "items (usage was "
                 "%.1f%%, now %.1f%%)",
                 (unsigned long)removed_count, usage * 100.0f,
                 ((float)(queue_size - removed_count) / (float)queue_capacity) *
                     100.0f);
      }
    }
  }

  // Non-blocking queue send to prevent BLE
  // callback blocking
  if (xQueueSend(s_speed_update_queue, &req, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Speed update queue full, dropping "
                  "request (non-blocking)");
  }
}

// Speed mark function (called from BLE
// callback - now just requests update via
// queue) Format: 19 4D 03 02 00 xx 2F (xx는
// 무시영역, data2를 10진수로 치환하여 speed로
// 활용)
static void speed_mark(uint8_t start, uint8_t id, uint8_t commend,
                       uint8_t data_length, uint8_t data1, uint8_t data2) {
  // Check if this matches: 19 4D 03 02 00 or
  // 19 4D 03 02 01
  if (start != 0x19 || id != 0x4D || commend != 0x03 || data_length != 0x02 ||
      (data1 != 0x00 && data1 != 0x01)) {
    // ESP_LOGW(TAG, "speed_mark: data format mismatch, "
    //               "expected 19 4D 03 02 00 or 01");
    return;
  }

  // Request speed update via queue (to be
  // processed in LVGL task)
  request_speed_update(start, id, commend, data_length, data1, data2);
}

// 구간속도 함수 (Average Speed Mark)
// Format: 19 4D 03 02 01 xx 2F (xx는 무시영역, data2를 10진수로 치환하여
// avr_speed로 활용)
static void avr_speed_mark(uint8_t start, uint8_t id, uint8_t commend,
                           uint8_t data_length, uint8_t data1, uint8_t data2) {

  // Check if this matches: 19 4D 03 02 01
  if (start != 0x19 || id != 0x4D || commend != 0x03 || data_length != 0x02 ||
      data1 != 0x01) {
    return;
  }

  // Request speed update via queue (to be processed in LVGL task)
  request_speed_update(start, id, commend, data_length, data1, data2);
}

// speed_mark update function
// Format: 19 4D 03 02 00 xx 2F (xx는 무시, data2 사용)
// data2를 10진수로 변환하여 속도 표시
static void update_speed_mark(uint8_t data2) {
  // Check if labels exist
  if (s_speed_mark_value_label == NULL || s_speed_mark_unit_label == NULL) {
    return;
  }

  // Convert data2 to decimal speed value
  uint8_t speed = data2;

  // Format speed value as string
  char speed_text[16];
  snprintf(speed_text, sizeof(speed_text), "%u", (unsigned int)speed);

  // Update speed value label (155pt, white, center + 83pt down)
  lv_label_set_text(s_speed_mark_value_label, speed_text);
  lv_obj_set_style_text_color(s_speed_mark_value_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_speed_mark_value_label, &font_ORB_155, 0);
  lv_obj_align(s_speed_mark_value_label, LV_ALIGN_CENTER, 0, 83);
  lv_obj_clear_flag(s_speed_mark_value_label, LV_OBJ_FLAG_HIDDEN);

  // --- HUD 모드 요청: 도로명이나 평균속도가 있으면 단위(km/h) 숨기고, 없으면
  // 표시 ---
  // 도로명이 1행 또는 2행 중 하나라도 활성 상태면(숨겨지지 않았으면) 보고 있는 것으로 판단
  bool road_name_visible =
      (s_road_name_label != NULL &&
       !lv_obj_has_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN));
  if (s_road_name_sub_label && !lv_obj_has_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN)) {
      road_name_visible = true;
  }
  
  bool avr_speed_visible =
      (s_avr_speed_value_label != NULL &&
       !lv_obj_has_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN));

  if (road_name_visible || avr_speed_visible) {
    lv_obj_add_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
  } else {
    // 둘 다 없을 때는 기존 km/h 단위 표시 (164pt 하단)
    lv_label_set_text(s_speed_mark_unit_label, "km/h");
    lv_obj_set_style_text_color(s_speed_mark_unit_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_speed_mark_unit_label, &font_kopub_35, 0);
    lv_obj_align(s_speed_mark_unit_label, LV_ALIGN_CENTER, 0, 164);
    lv_obj_clear_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_invalidate(s_speed_mark_unit_label);

  static uint8_t s_last_logged_speed = 255;
  if (speed != s_last_logged_speed) {
    // ESP_LOGI(TAG, "speed_mark: speed=%u (unit_hidden=%d)", (unsigned
    // int)speed, road_name_visible ? 1 : 0);
    s_last_logged_speed = speed;
  }
}

// Helper to align average speed labels based on current mode
static void align_avr_speed_labels(void) {
  if (!s_avr_speed_title_label || !s_avr_speed_value_label ||
      !s_avr_speed_unit_label)
    return;

  if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_NAVI) {
    // Original Dynamic Alignment for HUD
    lv_obj_update_layout(s_avr_speed_value_label);
    lv_coord_t val_w = lv_obj_get_width(s_avr_speed_value_label);
    char *txt = lv_label_get_text(s_avr_speed_value_label);
    int char_count = strlen(txt);
    lv_coord_t char_w = (char_count > 0) ? (val_w / char_count) : 0;

    // Check if average speed is actively shown
    bool avr_visible =
        !lv_obj_has_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);

    // If visible, set average speed to 170 (since both alternate exactly at
    // 170)
    lv_coord_t avr_y = 170;
    lv_obj_align(s_avr_speed_value_label, LV_ALIGN_CENTER, (val_w - char_w) / 2,
                 avr_y);
    lv_obj_align_to(s_avr_speed_title_label, s_avr_speed_value_label,
                    LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_align_to(s_avr_speed_unit_label, s_avr_speed_value_label,
                    LV_ALIGN_OUT_RIGHT_MID, 1, 0);

    // Dynamically HIDE Road Name label when average speed is shown to prevent
    // overlap
    if (s_road_name_label) {
      if (avr_visible) {
        lv_obj_add_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
        if (s_road_name_sub_label)
          lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
      } else {
        const char *current_road = lv_label_get_text(s_road_name_label);
        if (current_road && strlen(current_road) > 0) {
          lv_obj_clear_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
          if (s_road_name_sub_label &&
              strlen(lv_label_get_text(s_road_name_sub_label)) > 0) {
            lv_obj_clear_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
          }
        }
        lv_obj_align(s_road_name_label, LV_ALIGN_TOP_MID, 0,
                     388); // Same Y as avg speed, first line center at 403pt
      }
    }
  } else if (s_current_mode == DISPLAY_MODE_CLOCK && s_clock_option == 1) {
    // Clock Mode 2 Alignment
    // Title: Center - 120pt left, 50pt up
    lv_obj_align(s_avr_speed_title_label, LV_ALIGN_CENTER, -120, -50);
    // Value: Center - 120pt left
    lv_obj_align(s_avr_speed_value_label, LV_ALIGN_CENTER, -120, 0);
    // Unit: After value
    lv_obj_align_to(s_avr_speed_unit_label, s_avr_speed_value_label,
                    LV_ALIGN_OUT_RIGHT_BOTTOM, 5, 0);
  }

  // Ensure visibility above duplicates if any
  lv_obj_move_foreground(s_avr_speed_title_label);
  lv_obj_move_foreground(s_avr_speed_value_label);
  lv_obj_move_foreground(s_avr_speed_unit_label);
}

static size_t utf8_strlen_simple(const char *s) {
  size_t count = 0;
  while (*s) {
    if ((*s & 0xC0) != 0x80)
      count++;
    s++;
  }
  return count;
}

// Update speed label (called from LVGL task)
// NOTE: This function should only be called
// from LVGL handler task
static void update_speed_label(uint8_t data1, uint8_t speed) {
  // Remove generic drawing from here, move explicitly to SPEEDOMETER mode
  // condition.
  hide_black_screen_overlay();
  // [User Request] 가상 운행 모드에서는 연결이 없어도 업데이트 허용
  if (!s_connected && !s_virt_drive_active) return;

  if (s_current_mode != DISPLAY_MODE_GUIDE &&
      s_current_mode != DISPLAY_MODE_CLOCK)
    return;

  char speed_str[16];
  snprintf(speed_str, sizeof(speed_str), "%u", speed);

  if (data1 == 0x00) {
    // [일반속도] 155pt 흰색 (중앙 83pt 하), KM/H 35pt 흰색 (중앙 164pt 하)

    // HUD mode display
    if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_NAVI) {
      update_speed_mark(speed);
    }

    // Speedometer mode display
    if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER) {
      if (s_speedometer_needle_line != NULL) {
        static int s_last_drawn_speed = -1;
        int draw_speed = speed > 220 ? 220 : speed;

        if (draw_speed != s_last_drawn_speed) {
          s_last_drawn_speed = draw_speed;
          double angle_deg = 156.0 + (draw_speed / 220.0) * 228.0;
          double angle_rad = angle_deg * M_PI / 180.0;

          int center_x = LCD_H_RES / 2;
          int center_y = LCD_V_RES / 2;
          int radius = (LCD_H_RES < LCD_V_RES ? LCD_H_RES : LCD_V_RES) / 2 - 20;
          int needle_len = radius * 0.85;
          int tail_len = -40; // Tail extending through center

          double cos_a = cos(angle_rad);
          double sin_a = sin(angle_rad);
          double cos_p = cos(angle_rad + M_PI / 2.0);
          double sin_p = sin(angle_rad + M_PI / 2.0);

          // Tapered needle using 5 points and 5px line width to fill the center
          // (10px) and tip (5px)
          // 1. Tip
          s_speedometer_needle_points[0].x =
              center_x + (int)(needle_len * cos_a);
          s_speedometer_needle_points[0].y =
              center_y + (int)(needle_len * sin_a);
          // 2. Center-Top (offset 2.5px)
          s_speedometer_needle_points[1].x = center_x + (int)(2.5 * cos_p);
          s_speedometer_needle_points[1].y = center_y + (int)(2.5 * sin_p);
          // 3. Tail
          s_speedometer_needle_points[2].x = center_x + (int)(tail_len * cos_a);
          s_speedometer_needle_points[2].y = center_y + (int)(tail_len * sin_a);
          // 4. Center-Bottom (offset -2.5px)
          s_speedometer_needle_points[3].x = center_x + (int)(-2.5 * cos_p);
          s_speedometer_needle_points[3].y = center_y + (int)(-2.5 * sin_p);
          // 5. Back to Tip
          s_speedometer_needle_points[4] = s_speedometer_needle_points[0];

          lv_line_set_points(s_speedometer_needle_line,
                             s_speedometer_needle_points, 5);
        }
      }

      // Check if safety image is currently visible
      bool safety_visible = false;
      if (s_speedometer_safety_image != NULL) {
        safety_visible =
            !lv_obj_has_flag(s_speedometer_safety_image, LV_OBJ_FLAG_HIDDEN);
      }

      static int s_speedometer_last_label_speed = -1;
      static bool s_speedometer_last_safety_visible = false;

      if (speed != s_speedometer_last_label_speed ||
          safety_visible != s_speedometer_last_safety_visible) {
        s_speedometer_last_label_speed = speed;
        s_speedometer_last_safety_visible = safety_visible;

        if (!safety_visible && s_speedometer_speed_label &&
            s_speedometer_unit_label) {
          lv_label_set_text(s_speedometer_speed_label, speed_str);
          lv_obj_clear_flag(s_speedometer_speed_label, LV_OBJ_FLAG_HIDDEN);

          // 안내모드의 안전운행화면(속도계화면)에서는 도로명을 표기하지 않는다.
          // 단, 구간속도가 표시 중이면 km/h 단위를 숨겨 겹침 방지
          bool avr_visible = (s_speedometer_avr_speed_value_label != NULL && 
                              !lv_obj_has_flag(s_speedometer_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN));
          if (!avr_visible) {
              lv_obj_clear_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
          } else {
              lv_obj_add_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
          }
        } else if (safety_visible && s_speedometer_speed_label) {
          // Keep the label text updated even if hidden, so it's correct when
          // restored
          lv_label_set_text(s_speedometer_speed_label, speed_str);
        }
      }
    }
  } else if (data1 == 0x01) {
    if (s_avr_speed_title_label && s_avr_speed_value_label &&
        s_avr_speed_unit_label) {
      if (speed == 0) {
        // data2 내용이 "0"이면 표기하지 않는다
        lv_obj_add_flag(s_avr_speed_title_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_avr_speed_unit_label, LV_OBJ_FLAG_HIDDEN);

        // HUD 모드라면 현재속도 단위(KM/H)를 다시 표시하되, 도로명 활성화 시엔
        // 무시
        if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_NAVI && s_speed_mark_unit_label) {
          bool road_name_visible =
              (s_road_name_label != NULL &&
               !lv_obj_has_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN));
          if (!road_name_visible) {
            lv_obj_clear_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
          }
        }

        // 도로명 라벨 위치 원상복구를 위해 align 호출
        align_avr_speed_labels();
      } else {
        // 텍스트 설정
        lv_label_set_text(s_avr_speed_title_label, "구간속도");
        lv_label_set_text(s_avr_speed_value_label, speed_str);
        lv_label_set_text(s_avr_speed_unit_label, "km/h");

        lv_obj_clear_flag(s_avr_speed_title_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_avr_speed_unit_label, LV_OBJ_FLAG_HIDDEN);

        // HUD 모드라면 현재속도 단위(KM/H)를 숨김
        if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_NAVI && s_speed_mark_unit_label) {
          lv_obj_add_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
        }

        // 새로운 스타일 적용 (사용자 요청: 구간속도 25pt 회색, speed 40pt 녹색,
        // km/h 25pt 회색)
        lv_obj_set_style_text_font(s_avr_speed_title_label, &font_kopub_25, 0);
        lv_obj_set_style_text_color(s_avr_speed_title_label,
                                    lv_color_hex(0xCCCCCC), 0); // 회색

        lv_obj_set_style_text_font(s_avr_speed_value_label, &font_kopub_40, 0);
        lv_obj_set_style_text_color(s_avr_speed_value_label,
                                    lv_color_hex(0x00FF00), 0); // 녹색

        lv_obj_set_style_text_font(s_avr_speed_unit_label, &font_kopub_25, 0);
        lv_obj_set_style_text_color(s_avr_speed_unit_label,
                                    lv_color_hex(0xCCCCCC), 0); // 회색

        // Align labels based on mode
        // Align labels based on mode
        align_avr_speed_labels();

        // 상위 레이어로 이동 (다른 요소에 가려지지 않게)
        lv_obj_move_foreground(s_avr_speed_title_label);
        lv_obj_move_foreground(s_avr_speed_value_label);
        lv_obj_move_foreground(s_avr_speed_unit_label);
      }
    }

    // --- Speedometer Mode Average Speed Update ---
    if (s_speedometer_avr_speed_title_label &&
        s_speedometer_avr_speed_value_label &&
        s_speedometer_avr_speed_unit_label) {
      if (speed == 0) {
        lv_obj_add_flag(s_speedometer_avr_speed_title_label,
                        LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_speedometer_avr_speed_value_label,
                        LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_speedometer_avr_speed_unit_label, LV_OBJ_FLAG_HIDDEN);

        // 구간속도가 사라지면 km/h 단위 다시 표시 (안전안내 중이 아닐 때만)
        bool safety_visible = (s_speedometer_safety_image != NULL && !lv_obj_has_flag(s_speedometer_safety_image, LV_OBJ_FLAG_HIDDEN));
        if (!safety_visible && s_speedometer_unit_label) {
            lv_obj_clear_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
        }
      } else {
        lv_label_set_text(s_speedometer_avr_speed_value_label, speed_str);
        lv_obj_clear_flag(s_speedometer_avr_speed_title_label,
                          LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_speedometer_avr_speed_value_label,
                          LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_speedometer_avr_speed_unit_label,
                          LV_OBJ_FLAG_HIDDEN);

        // Dynamic Alignment:
        // [Title] 10pt [Value] 5pt [Unit]
        // Requirement:
        // "구간속도" 문자는 화면 중앙에서 위로 40pt , 우측으로 70pt 이동
        // "구간속도 단위는 20pt폰트로 변경

        // Apply style (25pt Grey / 40pt Green / 20pt Grey)
        lv_obj_set_style_text_font(s_speedometer_avr_speed_title_label,
                                   &font_kopub_25, 0);
        lv_obj_set_style_text_color(s_speedometer_avr_speed_title_label,
                                    lv_color_hex(0x808080), 0);
        lv_obj_set_style_text_font(s_speedometer_avr_speed_value_label,
                                   &font_kopub_40, 0);
        lv_obj_set_style_text_color(s_speedometer_avr_speed_value_label,
                                    lv_color_hex(0x00FF00), 0);
        lv_obj_set_style_text_font(s_speedometer_avr_speed_unit_label,
                                   &font_kopub_20, 0);
        lv_obj_set_style_text_color(s_speedometer_avr_speed_unit_label,
                                    lv_color_hex(0x808080), 0);

        // Speedometer mode: Align to HUD-equivalent position
        // HUD labels alternate with road name at Y=170 (rel to center)
        // Road name is at TOP_MID, 0, 388 (466/2 = 233. 388-233 = 155 approx)
        // Let's use Y=170 for consistency with HUD's avr_y logic
        
        lv_obj_update_layout(s_speedometer_avr_speed_value_label);
        lv_coord_t val_w = lv_obj_get_width(s_speedometer_avr_speed_value_label);
        char *txt = lv_label_get_text(s_speedometer_avr_speed_value_label);
        size_t char_count = utf8_strlen_simple(txt);
        lv_coord_t char_w = (char_count > 0) ? (val_w / char_count) : 0;

        lv_obj_align(s_speedometer_avr_speed_value_label, LV_ALIGN_CENTER, (val_w - char_w) / 2, 186);
        lv_obj_align_to(s_speedometer_avr_speed_title_label, s_speedometer_avr_speed_value_label,
                        LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_align_to(s_speedometer_avr_speed_unit_label, s_speedometer_avr_speed_value_label,
                        LV_ALIGN_OUT_RIGHT_MID, 1, 0);
        
        // Show unit by default for HUD parity
        lv_obj_clear_flag(s_speedometer_avr_speed_unit_label, LV_OBJ_FLAG_HIDDEN);

        // Hide speedometer road names when average speed is shown
        if (s_speedometer_road_name_label) lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
        if (s_speedometer_road_name_sub_label) lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
        
        // 구간속도 표시 중에는 km/h 단위를 숨겨 겹침 방지
        if (s_speedometer_unit_label) lv_obj_add_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
      }
    }
  }
}

// Update time display from system time
// 각 부분별로 색상이 다르므로 개별 label 사용
// 날짜: MM(하늘색) 월(회색) dd(하늘색)
// 일(회색)
static void update_time_display(void) {
  if (!s_time_initialized) return;

  struct tm timeinfo;
  time_t now;
  time(&now);
  localtime_r(&now, &timeinfo);

  int hour = (int)timeinfo.tm_hour;
  int min = (int)timeinfo.tm_min;
  int sec = (int)timeinfo.tm_sec;
  int month = (int)timeinfo.tm_mon + 1;
  int day = (int)timeinfo.tm_mday;

  // 1. Background update for Clock Mode (Analog)
  if (s_clock_canvas) {
    draw_analog_clock(hour, min, sec);
  }

  // 2. Update Digital Labels (Standby Mode)
  if (s_time_month_label && s_time_month_unit_label &&
      s_time_day_label && s_time_day_unit_label &&
      s_time_hour_label && s_time_minute_label && s_time_second_label) {
      
    char buf[16];
    
    snprintf(buf, sizeof(buf), "%d", month);
    lv_label_set_text(s_time_month_label, buf);
    lv_label_set_text(s_time_month_unit_label, "월");

    snprintf(buf, sizeof(buf), "%d", day);
    lv_label_set_text(s_time_day_label, buf);
    lv_label_set_text(s_time_day_unit_label, "일");

    snprintf(buf, sizeof(buf), "%02d", hour);
    lv_label_set_text(s_time_hour_label, buf);
    if (s_time_colon1_label) lv_label_set_text(s_time_colon1_label, ":");

    snprintf(buf, sizeof(buf), "%02d", min);
    lv_label_set_text(s_time_minute_label, buf);
    if (s_time_colon2_label) lv_label_set_text(s_time_colon2_label, ":");

    snprintf(buf, sizeof(buf), "%02d", sec);
    lv_label_set_text(s_time_second_label, buf);

    // Invalidate everything to be sure
    lv_obj_invalidate(s_time_month_label);
    lv_obj_invalidate(s_time_day_label);
    lv_obj_invalidate(s_time_hour_label);
    lv_obj_invalidate(s_time_minute_label);
    lv_obj_invalidate(s_time_second_label);
  }
}

static bool s_time_display_update_required = false;

// Time update timer callback
static void time_update_timer_cb(void *arg) {
  s_time_display_update_required = true;
}

// Time set function
// Time set function
// Format: 19 4D 09 0E data1 data2 ... data14
// 2F start, ID, commend, data_length = "19 4D
// 09 0E"일 때 실행 data1~data14는
// yyyyMMddHHmmss 형식의 UTF-8 바이트를
// 16진수로 변환한 값 BLE 연결 시 1번만
// 수신되고, 이후에는 내부 클럭으로 동작 time
// set 패킷 메시지는 /sdcard/time.txt에
// 순차적으로 저장
static void time_set(const uint8_t *data, size_t data_len) {
  // Check minimum length: start(1) + id(1) +
  // commend(1) + data_length(1) + data(14) +
  // end(1) = 19 bytes
  if (data_len < 19) {
    ESP_LOGW(TAG,
             "time_set: packet too short "
             "(%zu bytes)",
             data_len);
    return;
  }

  // Check packet header: start, ID, commend,
  // data_length = "19 4D 09 0E"
  if (data[0] != 0x19 || data[1] != 0x4D || data[2] != 0x09 ||
      data[3] != 0x0E) {
    ESP_LOGW(TAG, "time_set: invalid packet "
                  "header (expected 19 4D 09 0E)");
    return;
  }

  // BLE 연결 시 1번만 수신 제한 제거 (수신 시마다 적용)
  /* if (s_time_initialized) {
    ESP_LOGI(TAG, "time_set: time already "
                  "initialized, ignoring packet");
    return;
  } */

  // Extract data1~data14 (14 bytes for
  // yyyyMMddHHmmss) Each byte is a UTF-8
  // encoded digit (0-9 = 0x30-0x39)
  // data1~data14에서 추출 (인덱스 4부터
  // 17까지)
  if (data[4] < 0x30 || data[4] > 0x39 || // Check if valid
                                          // ASCII digit
      data[5] < 0x30 || data[5] > 0x39 || data[6] < 0x30 || data[6] > 0x39 ||
      data[7] < 0x30 || data[7] > 0x39 || data[8] < 0x30 || data[8] > 0x39 ||
      data[9] < 0x30 || data[9] > 0x39 || data[10] < 0x30 || data[10] > 0x39 ||
      data[11] < 0x30 || data[11] > 0x39 || data[12] < 0x30 ||
      data[12] > 0x39 || data[13] < 0x30 || data[13] > 0x39 ||
      data[14] < 0x30 || data[14] > 0x39 || data[15] < 0x30 ||
      data[15] > 0x39 || data[16] < 0x30 || data[16] > 0x39 ||
      data[17] < 0x30 || data[17] > 0x39) {
    ESP_LOGW(TAG, "time_set: invalid time "
                  "data (not ASCII digits)");
    return;
  }

  // Parse yyyyMMddHHmmss from data1~data14
  // data[4]~data[7]: year (yyyy)
  // data[8]~data[9]: month (MM)
  // data[10]~data[11]: day (dd)
  // data[12]~data[13]: hour (HH)
  // data[14]~data[15]: minute (mm)
  // data[16]~data[17]: second (ss)
  int year = (data[4] - '0') * 1000 + (data[5] - '0') * 100 +
             (data[6] - '0') * 10 + (data[7] - '0');
  int month = (data[8] - '0') * 10 + (data[9] - '0');
  int day = (data[10] - '0') * 10 + (data[11] - '0');
  int hour = (data[12] - '0') * 10 + (data[13] - '0');
  int min = (data[14] - '0') * 10 + (data[15] - '0');
  int sec = (data[16] - '0') * 10 + (data[17] - '0');

  // Validate parsed values
  if (year < 2000 || year > 2099 || month < 1 || month > 12 || day < 1 ||
      day > 31 || hour > 23 || min > 59 || sec > 59) {
    ESP_LOGW(TAG,
             "time_set: invalid time values "
             "(year=%d month=%d day=%d hour=%d "
             "min=%d sec=%d)",
             year, month, day, hour, min, sec);
    return;
  }

  // Set system time
  struct tm timeinfo = {0};
  timeinfo.tm_year = year - 1900;
  timeinfo.tm_mon = month - 1;
  timeinfo.tm_mday = day;
  timeinfo.tm_hour = hour;
  timeinfo.tm_min = min;
  timeinfo.tm_sec = sec;
  timeinfo.tm_isdst = -1; // Let system determine DST

  struct timeval tv = {0};
  tv.tv_sec = mktime(&timeinfo);
  tv.tv_usec = 0;

  if (settimeofday(&tv, NULL) != 0) {
    ESP_LOGE(TAG, "time_set: settimeofday failed");
    return;
  }

  ESP_LOGI(TAG,
           "time_set: time set to "
           "%04d-%02d-%02d %02d:%02d:%02d",
           year, month, day, hour, min, sec);

  // 부팅 후 첫 번째 시간 수신 시에만 대기 모드 전환 예약
  // s_time_initialized가 true가 되기 전에 체크해야 첫 번째만 트리거됨
  if (!s_time_initialized && s_current_mode != DISPLAY_MODE_OTA) {
    s_boot_clock_trigger = true;
  }

  // Mark time as initialized (첫 수신 체크 후 세팅)
  s_time_initialized = true;

  // Enable packet logging from now on
  s_logging_enabled = true;
  ESP_LOGI(TAG, "Packet logging enabled (time data received)");

  // Update display immediately (request via flag to be thread-safe)
  s_time_display_update_required = true;

  // Create periodic timer to update time
  // display every second (if not already
  // created)
  if (s_time_update_timer == NULL) {
    const esp_timer_create_args_t timer_args = {
        .callback = &time_update_timer_cb, .name = "time_update"};
    esp_err_t ret = esp_timer_create(&timer_args, &s_time_update_timer);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG,
               "time_set: failed to create "
               "timer: %s",
               esp_err_to_name(ret));
      return;
    }

    // Start periodic timer (1 second
    // interval)
    ret = esp_timer_start_periodic(s_time_update_timer,
                                   1000000); // 1 second in microseconds
    if (ret != ESP_OK) {
      ESP_LOGE(TAG,
               "time_set: failed to start "
               "timer: %s",
               esp_err_to_name(ret));
      esp_timer_delete(s_time_update_timer);
      s_time_update_timer = NULL;
      return;
    }
    ESP_LOGI(TAG, "time_set: periodic time "
                  "update timer started");
  }

  // Update brightness immediately based on new time (only if in Auto mode)
  if (s_brightness_level == 0) {
      update_auto_brightness(true);
  }
}

// 목적지정보 함수
// Format: 19 4D 0A 05 data1 data2 data3 data4
// data5 2F start, ID, commend, data_length =
// 19 4D 0A 05 가 일치하면 실행 data1~data2:
// 16진수로 남은시간 분단위 → 10진수로
// 치환하여 "목적지남은시간값"으로 적용
// data3~data5: 16진수로 남은거리 미터단위 →
// 10진수로 치환하여 "목적지남은거리값"으로
// 적용 "목적지남은시간값"이 60이하이면
// 분단위로 표기하고, 61이상이면 시,분 단위로
// 표기한다. "목적지남은거리값"이 1000보다
// 작으면 m단위로 표기하고, 1000이상이면
// km단위로 표기한다. "목적지남은시간값"은
// LCD중앙에 위로 5pt, 오른쪽 끝 기준으로
// 좌측으로 90pt 이동하여 표기한다.
// "목적지남은거리값"은 LCD중앙에 아래로 45pt,
// 오른쪽 끝 기준으로 좌측으로 90pt 이동하여
// 표기한다. "목적지남은시간값" 폰트크기는
// 30pt이고 노랑색으로 한다.
// "목적지남은거리값" 폰트크기는 30pt이고
// 하늘색으로 한다. go_to.bmp 이미지를
// LCD중앙에서 위로 80pt, 좌로 150pt 이동하여
// 표기한다. go_to.bmp가 이미 표시되고있으면
// 표기하지 않는다. 이미지파일은
// "littlefs/image/"에 있다. Request
// destination update (called from BLE
// callback - queues request to LVGL task)
static void request_destination_update(uint8_t data1, uint8_t data2,
                                       uint8_t data3, uint8_t data4,
                                       uint8_t data5) {
  if (s_destination_update_queue == NULL) {
    ESP_LOGW(TAG, "Destination update queue not "
                  "initialized, cannot request "
                  "destination update");
    return;
  }

  destination_update_request_t req = {.data1 = data1,
                                      .data2 = data2,
                                      .data3 = data3,
                                      .data4 = data4,
                                      .data5 = data5};

  // Filter duplicate requests to prevent
  // queue overflow
  if (s_last_destination_request_valid) {
    if (memcmp(&req, &s_last_destination_request,
               sizeof(destination_update_request_t)) == 0) {
      // Same request as last one, skip to
      // prevent queue overflow
      return;
    }
  }

  memcpy(&s_last_destination_request, &req,
         sizeof(destination_update_request_t));
  s_last_destination_request_valid = true;

  // Check queue usage and remove old items if
  // queue is >80% full to maintain buffer
  UBaseType_t queue_size = uxQueueMessagesWaiting(s_destination_update_queue);
  UBaseType_t queue_capacity =
      uxQueueSpacesAvailable(s_destination_update_queue) + queue_size;
  if (queue_capacity > 0) {
    float usage = (float)queue_size / (float)queue_capacity;
    const float CLEANUP_THRESHOLD = 0.80f; // 80% threshold
    if (usage >= CLEANUP_THRESHOLD) {
      // Remove old items until queue usage
      // drops to ~50% to maintain buffer
      destination_update_request_t dummy;
      UBaseType_t target_size = queue_capacity / 2; // Target 50% usage
      UBaseType_t items_to_remove =
          (queue_size > target_size) ? (queue_size - target_size) : 0;
      UBaseType_t removed_count = 0;
      for (UBaseType_t i = 0; i < items_to_remove; i++) {
        if (xQueueReceive(s_destination_update_queue, &dummy, 0) == pdTRUE) {
          removed_count++;
        } else {
          break; // No more items to remove
        }
      }
      if (removed_count > 0) {
        ESP_LOGW(TAG,
                 "Destination update queue "
                 "cleanup: removed %lu old items "
                 "(usage was %.1f%%, now %.1f%%)",
                 (unsigned long)removed_count, usage * 100.0f,
                 ((float)(queue_size - removed_count) / (float)queue_capacity) *
                     100.0f);
      }
    }
  }

  // Non-blocking queue send to prevent BLE
  // callback blocking
  if (xQueueSend(s_destination_update_queue, &req, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Destination update queue full, "
                  "dropping request (non-blocking)");
  }

  // Update destination info (called from LVGL
  // task - actual LVGL object manipulation)
}


static void update_road_name_label(const char *road_name) {
  // [User Request] 가상 운행 모드에서는 연결이 없어도 업데이트 허용
  if (!s_connected && !s_virt_drive_active) return;

  if (s_road_name_label == NULL) {
    ESP_LOGW(TAG, "Road Name: s_road_name_label is NULL!");
    return;
  }

  LVGL_LOCK(); // Ensure thread safety

  // Reset long modes and widths for fresh update
  lv_label_set_long_mode(s_road_name_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(s_road_name_label, LV_SIZE_CONTENT);
  if (s_road_name_sub_label) {
    lv_label_set_long_mode(s_road_name_sub_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_road_name_sub_label, LV_SIZE_CONTENT);
  }
  if (s_speedometer_road_name_label) {
    lv_label_set_long_mode(s_speedometer_road_name_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_speedometer_road_name_label, LV_SIZE_CONTENT);
  }
  if (s_speedometer_road_name_sub_label) {
    lv_label_set_long_mode(s_speedometer_road_name_sub_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_speedometer_road_name_sub_label, LV_SIZE_CONTENT);
  }

  if (road_name == NULL || strlen(road_name) == 0) {
    ESP_LOGI(TAG, "Road Name: Received empty string, hiding labels");
    lv_obj_add_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
    if (s_road_name_sub_label) lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    
    if (s_speedometer_road_name_label) {
      lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_road_name_sub_label) {
      lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    }

    // 도로명이 사라지면 km/h 단위 다시 표시 (HUD 모드)
    if (s_speed_mark_unit_label) {
      bool avr_speed_visible =
          (s_avr_speed_value_label != NULL &&
           !lv_obj_has_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN));
      if (!avr_speed_visible) {
        lv_obj_clear_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(s_speed_mark_unit_label);
      }
    }

    // 도로명이 사라지면 속도계 모드 단위(km/h) 다시 표시
    if (s_speedometer_unit_label) {
      lv_obj_clear_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
    }
    
    LVGL_UNLOCK();
    return;
  }

  ESP_LOGI(TAG, "Road Name: Updating label to '%s'", road_name);

  // Road Name Logic: Split by \n if logic applies, otherwise check for manual \n
  char line1[128] = {0};
  char line2[128] = {0};
  bool has_split = false;

  // Existing Line Break Logic: "로" + "숫자" + "길" -> Insert \n
  char processed_name[256];
  strncpy(processed_name, road_name, sizeof(processed_name) - 1);
  processed_name[sizeof(processed_name) - 1] = '\0';

  const char *ro_search_start = processed_name;
  while (1) {
    char *ro_ptr = strstr((char *)ro_search_start, "\xEB\xA1\x9C"); // "로"
    if (!ro_ptr)
      break;

    char *pattern_ptr = ro_ptr + 3; // "로" 이후
    while (*pattern_ptr == ' ')
      pattern_ptr++;

    if (*pattern_ptr >= '0' && *pattern_ptr <= '9') {
      char *digit_start = pattern_ptr;
      while (*pattern_ptr >= '0' && *pattern_ptr <= '9')
        pattern_ptr++;

      if (strncmp(pattern_ptr, "\xEA\xB8\xB8", 3) == 0) { // "길"
        size_t head_len = digit_start - processed_name;
        while (head_len > 0 && processed_name[head_len - 1] == ' ')
          head_len--;

        snprintf(line1, sizeof(line1), "%.*s", (int)head_len, processed_name);
        snprintf(line2, sizeof(line2), "%s", digit_start);
        has_split = true;
        break;
      }
    }
    ro_search_start = ro_ptr + 3;
  }

  // If not split by pattern, check for existing \n or just use as line1
  if (!has_split) {
    char *newline_ptr = strchr(processed_name, '\n');
    if (newline_ptr) {
      size_t line1_len = newline_ptr - processed_name;
      snprintf(line1, sizeof(line1), "%.*s", (int)line1_len, processed_name);
      snprintf(line2, sizeof(line2), "%s", newline_ptr + 1);
      has_split = true;
    } else {
      strncpy(line1, processed_name, sizeof(line1) - 1);
      line2[0] = '\0';
      has_split = false;
    }
  }

  // Set texts
  lv_label_set_text(s_road_name_label, line1);
  if (s_speedometer_road_name_label) lv_label_set_text(s_speedometer_road_name_label, line1);
  
  if (has_split) {
    if (s_road_name_sub_label) {
      lv_label_set_text(s_road_name_sub_label, line2);
      lv_obj_clear_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_speedometer_road_name_sub_label) {
      lv_label_set_text(s_speedometer_road_name_sub_label, line2);
      lv_obj_clear_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    }
  } else {
    if (s_road_name_sub_label) lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    if (s_speedometer_road_name_sub_label) lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
  }

  // Apply Scrolling (11 chars for top, 5 chars for bottom)
  // 30pt font: 11 chars ~= 250px, 5 chars ~= 115px (approximate)
  const lv_coord_t top_limit_width = 250;
  const lv_coord_t bottom_limit_width = 115;

  if (utf8_strlen_simple(line1) >= 11) {
    lv_obj_set_width(s_road_name_label, top_limit_width);
    lv_label_set_long_mode(s_road_name_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    if (s_speedometer_road_name_label) {
      lv_obj_set_width(s_speedometer_road_name_label, top_limit_width);
      lv_label_set_long_mode(s_speedometer_road_name_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    }
  }

  if (has_split && utf8_strlen_simple(line2) >= 5) {
    if (s_road_name_sub_label) {
      lv_obj_set_width(s_road_name_sub_label, bottom_limit_width);
      lv_label_set_long_mode(s_road_name_sub_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    }
    if (s_speedometer_road_name_sub_label) {
      lv_obj_set_width(s_speedometer_road_name_sub_label, bottom_limit_width);
      lv_label_set_long_mode(s_speedometer_road_name_sub_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    }
  }

  // 가시성 및 위치 제어
  if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_NAVI) {
    lv_obj_set_parent(s_road_name_label, s_hud_screen);
    lv_obj_move_foreground(s_road_name_label);
    if (s_road_name_sub_label) {
        lv_obj_set_parent(s_road_name_sub_label, s_hud_screen);
        lv_obj_move_foreground(s_road_name_sub_label);
    }
    
    lv_obj_align(s_road_name_label, LV_ALIGN_TOP_MID, 0, 388);
    if (s_road_name_sub_label) lv_obj_align(s_road_name_sub_label, LV_ALIGN_TOP_MID, 0, 423);
    
    // 도로명 색상을 약간 어두운 노란색으로 변경 (210, 210, 0)
    lv_obj_set_style_text_color(s_road_name_label, lv_color_make(210, 210, 0), 0);
    if (s_road_name_sub_label) lv_obj_set_style_text_color(s_road_name_sub_label, lv_color_make(210, 210, 0), 0);

    bool avr_visible = s_avr_speed_value_label && !lv_obj_has_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);
    // 안전운행 정보 표시 중에도 도로명을 표기하도록 로직 수정 (avr_visible만 체크)
    if (!avr_visible) {
      lv_obj_clear_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
      if (has_split && s_road_name_sub_label) lv_obj_clear_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
      
      if (s_speed_mark_unit_label) {
        lv_obj_add_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
      }
    } else {
      lv_obj_add_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
      if (s_road_name_sub_label) lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    }
    // Disable speedometer labels
    if (s_speedometer_road_name_label) lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
    if (s_speedometer_road_name_sub_label) lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);

  } else if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER) {
    // 안내모드의 안전운행화면(속도계화면)에서는 도로명을 표기하지 않는다.
    if (s_speedometer_road_name_label) lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
    if (s_speedometer_road_name_sub_label) lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    // Disable primary HUD labels
    lv_obj_add_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
    if (s_road_name_sub_label) lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);
    if (s_road_name_sub_label) lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
    if (s_speedometer_road_name_label) lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
    if (s_speedometer_road_name_sub_label) lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
  }

  LVGL_UNLOCK();
}

// Update destination info (called from LVGL task)
// Update destination info (called from LVGL task)
static void update_destination_info(uint8_t data1, uint8_t data2, uint8_t data3,
                                    uint8_t data4, uint8_t data5) {
  // Hide black screen overlay if visible
  hide_black_screen_overlay();

  // [User Request] 가상 운행 모드에서는 연결이 없어도 업데이트 허용
  if (!s_connected && !s_virt_drive_active) return;

  // Check if labels exist
  if (s_dest_time_value_label == NULL || s_dest_time_hour_unit_label == NULL ||
      s_dest_time_minute_value_label == NULL ||
      s_dest_time_unit_label == NULL || s_dest_distance_value_label == NULL ||
      s_dest_distance_unit_label == NULL) {
    return;
  }

  // Calculate remaining time in minutes from data1(High), data2(Low)
  uint16_t time_minutes = ((uint16_t)data1 << 8) | (uint16_t)data2;

  // Calculate remaining distance in meters (data3:High, data4:Mid, data5:Low)
  // Applied but not displayed as per recent context
  uint32_t distance_m =
      ((uint32_t)data3 << 16) | ((uint32_t)data4 << 8) | (uint32_t)data5;
  (void)distance_m; // Suppress unused warning

  char time_value_text[32];
  char time_hour_text[16] = "";
  char time_minute_text[16] = "";
  bool show_hours = false;

  if (time_minutes <= 60) {
    snprintf(time_value_text, sizeof(time_value_text), "%u",
             (unsigned int)time_minutes);
    show_hours = false;
  } else {
    uint16_t hours = time_minutes / 60;
    uint16_t minutes = time_minutes % 60;
    snprintf(time_hour_text, sizeof(time_hour_text), "%u", (unsigned int)hours);
    snprintf(time_minute_text, sizeof(time_minute_text), "%u",
             (unsigned int)minutes);
    show_hours = true;
  }

  if (show_hours) {
    // 61분 이상: "H시간 MM분"

    // 1시간 30분 형태인 경우 전체 문장의 너비를 계산하여 중앙 정렬 (기준점:
    // -175)
    lv_coord_t w_hour_val =
        lv_txt_get_width(time_hour_text, strlen(time_hour_text), &font_kopub_35,
                         0, LV_TEXT_FLAG_NONE);
    lv_coord_t w_hour_unit = lv_txt_get_width(
        "시간", strlen("시간"), &font_kopub_25, 0, LV_TEXT_FLAG_NONE);
    lv_coord_t w_min_val =
        lv_txt_get_width(time_minute_text, strlen(time_minute_text),
                         &font_kopub_35, 0, LV_TEXT_FLAG_NONE);
    lv_coord_t w_min_unit = lv_txt_get_width("분", strlen("분"), &font_kopub_25,
                                             0, LV_TEXT_FLAG_NONE);

    // 전체 너비 = 각 요소 너비 + 간격(1pt * 3)
    lv_coord_t total_width =
        w_hour_val + 1 + w_hour_unit + 1 + w_min_val + 1 + w_min_unit;

    // 시작점 계산: 기준점(-150)이 전체 너비의 중앙이 되도록 함
    // start_x (전체 문장의 왼쪽 끝) = -150 - (total_width / 2)
    lv_coord_t start_x =
        -150 - (total_width / 2); // 좌측 150pt로 이동 (기존 -100)

    // 1. Hour Value: 시작점에서 자신의 너비 절반만큼 오른쪽으로 정렬
    lv_label_set_text(s_dest_time_value_label, time_hour_text);
    lv_obj_set_style_text_color(s_dest_time_value_label, lv_color_hex(0x00FF00),
                                0); // 밝은 초록색 (Bright Green)
    lv_obj_set_style_text_font(s_dest_time_value_label, &font_kopub_35, 0);
    lv_obj_align(s_dest_time_value_label, LV_ALIGN_CENTER,
                 start_x + (w_hour_val / 2), -15);
    lv_obj_clear_flag(s_dest_time_value_label, LV_OBJ_FLAG_HIDDEN);

    // 2. Hour Unit "시간" (회색, 25pt)
    lv_label_set_text(s_dest_time_hour_unit_label, "시간");
    lv_obj_set_style_text_color(s_dest_time_hour_unit_label, lv_color_white(),
                                0); // 흰색
    lv_obj_set_style_text_font(s_dest_time_hour_unit_label, &font_kopub_25, 0);

    // 3. Minute Value (녹색, 35pt)
    lv_label_set_text(s_dest_time_minute_value_label, time_minute_text);
    lv_obj_set_style_text_color(s_dest_time_minute_value_label,
                                lv_color_hex(0x00FF00), 0); // 밝은 초록색 (Bright Green)
    lv_obj_set_style_text_font(s_dest_time_minute_value_label, &font_kopub_35,
                               0);

    // 4. Minute Unit "분" (회색, 25pt)
    lv_label_set_text(s_dest_time_unit_label, "분");
    lv_obj_set_style_text_color(s_dest_time_unit_label, lv_color_white(),
                                0); // 흰색
    lv_obj_set_style_text_font(s_dest_time_unit_label, &font_kopub_25, 0);

    // Align Hour Unit: Right of Hour Value (1pt spacing)
    lv_obj_align_to(s_dest_time_hour_unit_label, s_dest_time_value_label,
                    LV_ALIGN_OUT_RIGHT_MID, 1, 0);
    lv_obj_clear_flag(s_dest_time_hour_unit_label, LV_OBJ_FLAG_HIDDEN);

    // Align Minute Value: Right of Hour Unit (1pt spacing)
    lv_obj_align_to(s_dest_time_minute_value_label, s_dest_time_hour_unit_label,
                    LV_ALIGN_OUT_RIGHT_MID, 1, 0);
    lv_obj_clear_flag(s_dest_time_minute_value_label, LV_OBJ_FLAG_HIDDEN);

    // Align Minute Unit: Right of Minute Value (1pt spacing)
    lv_obj_align_to(s_dest_time_unit_label, s_dest_time_minute_value_label,
                    LV_ALIGN_OUT_RIGHT_MID, 1, 0);
    lv_obj_clear_flag(s_dest_time_unit_label, LV_OBJ_FLAG_HIDDEN);

  } else {
    // 60분 이하: "MM분"

    // 30분 형태인 경우 전체 문장의 너비를 계산하여 중앙 정렬 (기준점: -175)
    lv_coord_t w_val =
        lv_txt_get_width(time_value_text, strlen(time_value_text),
                         &font_kopub_35, 0, LV_TEXT_FLAG_NONE);
    lv_coord_t w_unit = lv_txt_get_width("분", strlen("분"), &font_kopub_25, 0,
                                         LV_TEXT_FLAG_NONE);

    // 전체 너비 = 값 너비 + 간격(1pt) + 단위 너비
    lv_coord_t total_width = w_val + 1 + w_unit;

    // 시작점 계산: 기준점(-150)이 전체 너비의 중앙이 되도록 함
    lv_coord_t start_x =
        -150 - (total_width / 2); // 좌측 150pt로 이동 (기존 -100)

    // 1. Minute Value (using time value label) (밝은 초록색, 30pt)
    lv_label_set_text(s_dest_time_value_label, time_value_text);
    lv_obj_set_style_text_color(s_dest_time_value_label, lv_color_hex(0x00FF00), 0); // 밝은 초록색 (Bright Green)
    lv_obj_set_style_text_font(s_dest_time_value_label, &font_kopub_35, 0);
    lv_obj_align(s_dest_time_value_label, LV_ALIGN_CENTER,
                 start_x + (w_val / 2), -15);
    lv_obj_clear_flag(s_dest_time_value_label, LV_OBJ_FLAG_HIDDEN);

    // 2. Minute Unit "분" (회색, 30pt)
    lv_label_set_text(s_dest_time_unit_label, "분");
    lv_obj_set_style_text_color(s_dest_time_unit_label, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(s_dest_time_unit_label, &font_addr_30, 0);

    // Hide Unused
    lv_obj_add_flag(s_dest_time_hour_unit_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_dest_time_minute_value_label, LV_OBJ_FLAG_HIDDEN);

    // Align Minute Unit: Right of Minute Value (1pt spacing)
    lv_obj_align_to(s_dest_time_unit_label, s_dest_time_value_label,
                    LV_ALIGN_OUT_RIGHT_MID, 1, 0);
    lv_obj_clear_flag(s_dest_time_unit_label, LV_OBJ_FLAG_HIDDEN);
  }

  // Display "도착지까지" Label
  // Position: 문자 첫글자 기준으로 중앙에서 위로 50pt, 좌로 190pt
  // Color: 회색 (Gray)
  if (s_dest_label) {
    lv_label_set_text(s_dest_label, "소요시간");
    lv_obj_set_style_text_color(s_dest_label, lv_color_make(180, 160, 0),
                                0); // 어두운 노랑색 (Dark Yellow)
    // Use 30pt font as requested
    lv_obj_set_style_text_font(s_dest_label, &font_addr_30, 0);

    // Calculate width with 30pt font
    lv_coord_t label_width =
        lv_txt_get_width("소요시간", strlen("소요시간"), &font_addr_30, 0,
                         LV_TEXT_FLAG_NONE);

    // Align Left edge at -190
    // Center X = TargetLeftX + Width/2 = -190 + Width/2
    lv_obj_align(s_dest_label, LV_ALIGN_CENTER, -190 + (label_width / 2), -53);

    lv_obj_clear_flag(s_dest_label, LV_OBJ_FLAG_HIDDEN);
  }

  // Display Remaining Distance - REQUESTED TO BE HIDDEN (Step 286)
  // 3. Hide Distance Info
  if (s_dest_distance_value_label)
    lv_obj_add_flag(s_dest_distance_value_label, LV_OBJ_FLAG_HIDDEN);
  if (s_dest_distance_unit_label)
    lv_obj_add_flag(s_dest_distance_unit_label, LV_OBJ_FLAG_HIDDEN);

  // 4. Hide Image - REQUESTED TO BE HIDDEN (Step 286)
  if (s_dest_image)
    lv_obj_add_flag(s_dest_image, LV_OBJ_FLAG_HIDDEN);
}

// Request image update via queue (to avoid
// stack overflow in ble_tx_task)
static void request_image_update(uint8_t start, uint8_t id, uint8_t commend,
                                 uint8_t data_length, uint8_t data1,
                                 uint8_t data2, bool force_update,
                                 bool has_distance, uint8_t data3,
                                 uint8_t data4, uint8_t data5) {
  if (s_image_update_queue == NULL) {
    ESP_LOGW(TAG, "Image update queue not initialized, "
                  "cannot request image update");
    return;
  }

  image_update_request_t req = {.start = start,
                                .id = id,
                                .commend = commend,
                                .data_length = data_length,
                                .data1 = data1,
                                .data2 = data2,
                                .force_update = force_update,
                                .has_distance = has_distance,
                                .data3 = data3,
                                .data4 = data4,
                                .data5 = data5};

  // Filter duplicate requests (same request
  // as last one, unless force_update)
  if (!force_update && s_last_image_request_valid) {
    if (memcmp(&req, &s_last_image_request, sizeof(image_update_request_t)) ==
        0) {
      // Same request as last one, skip to
      // prevent queue overflow
      return;
    }
  }

  // Save last request
  memcpy(&s_last_image_request, &req, sizeof(image_update_request_t));
  s_last_image_request_valid = true;

  // Check queue usage and remove old items if
  // queue is >80% full to maintain buffer
  UBaseType_t queue_size = uxQueueMessagesWaiting(s_image_update_queue);
  UBaseType_t queue_capacity =
      uxQueueSpacesAvailable(s_image_update_queue) + queue_size;
  if (queue_capacity > 0) {
    float usage = (float)queue_size / (float)queue_capacity;
    const float CLEANUP_THRESHOLD = 0.80f; // 80% threshold
    if (usage >= CLEANUP_THRESHOLD) {
      // Remove old items until queue usage
      // drops to ~50% to maintain buffer
      image_update_request_t dummy;
      UBaseType_t target_size = queue_capacity / 2; // Target 50% usage
      UBaseType_t items_to_remove =
          (queue_size > target_size) ? (queue_size - target_size) : 0;
      UBaseType_t removed_count = 0;
      for (UBaseType_t i = 0; i < items_to_remove; i++) {
        if (xQueueReceive(s_image_update_queue, &dummy, 0) == pdTRUE) {
          removed_count++;
        } else {
          break; // No more items to remove
        }
      }
      /* if (removed_count > 0) {
        ESP_LOGW(TAG,
                 "Image update queue cleanup:
      removed %lu old items (usage was "
                 "%.1f%%, now %.1f%%)",
                 (unsigned long)removed_count,
      usage * 100.0f,
                 ((float)(queue_size -
      removed_count) / (float)queue_capacity)
      * 100.0f);
      } */
    }
  }

  // Non-blocking queue send to prevent BLE
  // callback blocking
  if (xQueueSend(s_image_update_queue, &req, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Image update queue full, dropping "
                  "request (non-blocking)");
  }
}

// Update TBT distance labels (called from
// LVGL task) NOTE: This function should only
// be called from LVGL handler task
static void update_tbt_distance_labels(uint8_t data3, uint8_t data4,
                                       uint8_t data5) {
  // Check if labels exist
  if (s_length_tbt_value_label == NULL || s_length_tbt_unit_label == NULL) {
    ESP_LOGW(TAG, "TBT방향표시: TBT length "
                  "labels not created yet");
    return;
  }

  // Calculate distance in meters from data3,
  // data4, data5 각 바이트는 16진수 자리를
  // 표현하며 이를 10진수로 변환하여
  // TBT계산값으로 사용 data3 is high byte,
  // data4 is middle byte, data5 is low byte
  uint32_t distance_m =
      ((uint32_t)data3 << 16) | ((uint32_t)data4 << 8) | (uint32_t)data5;

  // TBT계산값이 0이면 표기하지 않음
  if (distance_m == 0) {
    lv_obj_add_flag(s_length_tbt_value_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_length_tbt_unit_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(s_length_tbt_value_label);
    lv_obj_invalidate(s_length_tbt_unit_label);
    return;
  }

  // TBT계산값 표시: 1000 미만은 미터, 1000
  // 이상은 킬로미터로 변환 1000=1Km,
  // 1500=1.5km, 900=900m 형식으로 표시
  char value_text[32];
  char unit_text[8];
  if (distance_m < 1000) {
    // 1000 미만: 미터 단위로 그대로 표시 (예:
    // 900m)
    snprintf(value_text, sizeof(value_text), "%lu", (unsigned long)distance_m);
    snprintf(unit_text, sizeof(unit_text), "m");
  } else {
    // 1000 이상: 킬로미터로 변환하여 소수점
    // 한 자리까지 표시 (예: 1Km, 1.5Km)
    float distance_km = distance_m / 1000.0f;
    // 정수인지 확인 (소수점이 0인지)
    if (distance_km == (float)((uint32_t)distance_km)) {
      // 정수인 경우 (예: 1000 = 1Km)
      snprintf(value_text, sizeof(value_text), "%lu",
               (unsigned long)((uint32_t)distance_km));
    } else {
      // 소수점이 있는 경우 소수점 한 자리까지
      // 표시 (예: 1500 = 1.5Km)
      snprintf(value_text, sizeof(value_text), "%.1f", distance_km);
    }
    snprintf(unit_text, sizeof(unit_text), "Km");
  }

  // Update TBT계산값 label (40pt, 녹색)
  // 위치: LCD 중앙에서 위로 166pt 이동, 마지막 숫자 중심 기준
  lv_label_set_text(s_length_tbt_value_label, value_text);
  lv_obj_set_style_text_color(s_length_tbt_value_label, lv_color_hex(0x00FF00),
                              0); // 녹색
  lv_obj_set_style_text_font(s_length_tbt_value_label, &font_kopub_40,
                             0); // 40pt 폰트

  // Calculate width to align right edge at center
  lv_coord_t value_width = lv_txt_get_width(
      value_text, strlen(value_text), &font_kopub_40, 0, LV_TEXT_FLAG_NONE);
  // Align: right edge at X=0 (center), so center of label at X = -width/2
  lv_obj_align(s_length_tbt_value_label, LV_ALIGN_CENTER, -value_width / 2,
               -166); // 중앙에서 위로 166pt, 마지막 숫자 중심
  lv_obj_clear_flag(s_length_tbt_value_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_invalidate(s_length_tbt_value_label);

  // Update TBT단위 label (30pt, 회색)
  // 위치: TBT계산값 오른쪽 끝 숫자에서 3pt 우로 이동
  lv_label_set_text(s_length_tbt_unit_label, unit_text);
  lv_obj_set_style_text_color(s_length_tbt_unit_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_length_tbt_unit_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_obj_align_to(s_length_tbt_unit_label, s_length_tbt_value_label,
                  LV_ALIGN_OUT_RIGHT_MID, 3, 0); // 값 레이블 우측 3pt
  lv_obj_clear_flag(s_length_tbt_unit_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_invalidate(s_length_tbt_unit_label);
}

// Safety 거리 표시 함수
// data4, data5, data6는 16진수 미터단위 정보
// 각 바이트는 16진수 자리를 표현하며 이를 10진수로 변환하여 safety거리값으로
// 사용 NOTE: This function should only be called from LVGL handler task
#if 0
static void update_safety_distance_labels(uint8_t data4, uint8_t data5,
                                          uint8_t data6) {
  // Check if labels exist
  if (s_safety_length_value_label == NULL ||
      s_safety_length_unit_label == NULL) {
    ESP_LOGW(TAG, "Safety_DRV: safety length labels not created yet");
    return;
  }

  // Calculate distance in meters from data4, data5, data6
  // data4 is high byte, data5 is middle byte, data6 is low byte
  uint32_t distance_m =
      ((uint32_t)data4 << 16) | ((uint32_t)data5 << 8) | (uint32_t)data6;

  // safety거리값이 20 이하이면 표기하지 않음
  if (distance_m <= 20) {
    lv_obj_add_flag(s_safety_length_value_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_safety_length_unit_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(s_safety_length_value_label);
    lv_obj_invalidate(s_safety_length_unit_label);
    return;
  }

  // safety거리값 표시: 1000 미만은 미터, 1000 이상은 킬로미터로 변환
  // 1000=1Km, 1500=1.5km, 900=900m 형식으로 표시
  char value_text[32];
  char unit_text[8];
  if (distance_m < 1000) {
    // 1000 미만: 미터 단위로 그대로 표시 (예: 900m)
    snprintf(value_text, sizeof(value_text), "%lu", (unsigned long)distance_m);
    snprintf(unit_text, sizeof(unit_text), "m");
  } else {
    // 1000 이상: 킬로미터로 변환하여 소수점 한 자리까지 표시 (예: 1Km, 1.5Km)
    float distance_km = distance_m / 1000.0f;
    // 정수인지 확인 (소수점이 0인지)
    if (distance_km == (float)((uint32_t)distance_km)) {
      // 정수인 경우 (예: 1000 = 1Km)
      snprintf(value_text, sizeof(value_text), "%lu",
               (unsigned long)((uint32_t)distance_km));
    } else {
      // 소수점이 있는 경우 소수점 한 자리까지 표시 (예: 1500 = 1.5Km)
      snprintf(value_text, sizeof(value_text), "%.1f", distance_km);
    }
    snprintf(unit_text, sizeof(unit_text), "Km");
  }

  // Update safety거리값 label (35pt, 회색)
  // 위치: LCD 중앙에서 좌측으로 120pt, 위로 37pt 이동, 마지막 숫자 중심 기준
  lv_label_set_text(s_safety_length_value_label, value_text);
  lv_obj_set_style_text_color(s_safety_length_value_label,
                              lv_color_white(), 0); // 흰색
  lv_obj_set_style_text_font(s_safety_length_value_label, &font_kopub_35,
                             0); // 35pt 폰트

  // Calculate width to align right edge at X=-120
  lv_coord_t value_width = lv_txt_get_width(value_text, strlen(value_text),
                                            &font_kopub_35, 0, LV_TEXT_FLAG_NONE);
  // Align: right edge at X=-120, so center of label at X = -120 - width/2
  lv_obj_align(s_safety_length_value_label, LV_ALIGN_CENTER,
               -120 - value_width / 2,
               -37); // 중앙에서 좌측 120pt, 위로 37pt, 마지막 숫자 중심
  lv_obj_clear_flag(s_safety_length_value_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_invalidate(s_safety_length_value_label);

  // Update safety단위 label (25pt, 회색)
  // 위치: safety거리값 오른쪽 끝 숫자에서 1pt 우로 이동
  lv_label_set_text(s_safety_length_unit_label, unit_text);
  lv_obj_set_style_text_color(s_safety_length_unit_label,
                              lv_color_white(), 0); // 흰색
  lv_obj_set_style_text_font(s_safety_length_unit_label, &font_kopub_25,
                             0); // 25pt 폰트
  lv_obj_align_to(s_safety_length_unit_label, s_safety_length_value_label,
                  LV_ALIGN_OUT_RIGHT_MID, 1, 0); // 값 레이블 우측 1pt
  lv_obj_clear_flag(s_safety_length_unit_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_invalidate(s_safety_length_unit_label);
}
#endif

// Update HUD image based on TBT entry
// (TBT방향표시) 참고파일: "littlefs/TBT.csv"
// start, id, commend, data_length, data1,
// data2가 일치하면 실행 이미지파일은
// "littlefs/image/"에 있음 CSV "image"항에
// 해당 이미지를 사용 "sector"를 참고하여
// LCD에 표기 표시는 표기하고 있는 이미지가
// 없거나 변경되었을 때만 화면을 갱신
// force_update: true면 경로가 같아도 이미지를
// 다시 로드 (모드 전환 시 사용) NOTE: This
// function should only be called from LVGL
// handler task to avoid stack overflow
static void update_hud_image_for_data(const image_data_entry_t *entry,
                                      uint8_t data2, bool force_update) {
  if (entry == NULL) {
    ESP_LOGW(TAG, "update_hud_image_for_data:"
                  " entry is NULL");
    return;
  }

  // Build full image path first to check for
  // duplicates
  char img_filename[64];
  strncpy(img_filename, entry->image_filename, sizeof(img_filename) - 1);
  img_filename[sizeof(img_filename) - 1] = '\0';

  char img_path[128];
  // Use LVGL VFS path (S: drive) for consistency with Safety images
  snprintf(img_path, sizeof(img_path), "S:/littlefs/image/%s", img_filename);

  // Early exit if the image is already
  // displayed and no force update is
  // requested This is the most common case
  // during high-frequency TBT updates
  // (distance changes only)
  if (strcmp(img_path, s_current_image_path) == 0 && !force_update) {
    // BUG FIX: Even if path is same, if image is hidden (e.g. by clear
    // command), we must unhide it to show it again with the distance update.
    if (s_hud_image != NULL &&
        lv_obj_has_flag(s_hud_image, LV_OBJ_FLAG_HIDDEN)) {
      lv_obj_clear_flag(s_hud_image, LV_OBJ_FLAG_HIDDEN);
      lv_obj_invalidate(s_hud_image);
    }
    return;
  }

  // Hide black screen overlay if visible
  hide_black_screen_overlay();

  if (s_hud_image == NULL) {
    ESP_LOGW(TAG, "s_hud_image is NULL, "
                  "cannot update image");
    return;
  }

  if (s_current_mode != DISPLAY_MODE_GUIDE || s_guide_sub_mode != GUIDE_SUB_NAVI) {
    /* ESP_LOGW(TAG,
             "Not in HUD mode (current "
             "mode=%d), cannot update image",
             s_current_mode); */
    return;
  }

  if (force_update && strcmp(img_path, s_current_image_path) == 0) {
    ESP_LOGI(TAG, "HUD: Force update requested, "
                  "refreshing image visibility");
    lv_obj_clear_flag(s_hud_image, LV_OBJ_FLAG_HIDDEN);
    return;
  }

  ESP_LOGI(TAG,
           "HUD: Image change detected: %s "
           "-> %s (sector=%u)",
           s_current_image_path, img_path, entry->sector);

  // Update current image path
  strncpy(s_current_image_path, img_path, sizeof(s_current_image_path) - 1);
  s_current_image_path[sizeof(s_current_image_path) - 1] = '\0';

  // Note: CSV size information is ignored -
  // images are displayed at original size
  // (Previously parsed CSV size field, but
  // now using original image dimensions)

  // Sector offset will be calculated later
  // when setting position

  // Removed lv_timer_handler() to prevent
  // potential reentrancy/hangs during heavy
  // image decoding tasks.

  ESP_LOGI(TAG,
           "HUD: Starting image load: %s "
           "(sector=%u)",
           img_path, entry->sector);

  // Hide image first to prevent white
  // rectangle flash during image swap
  lv_obj_add_flag(s_hud_image, LV_OBJ_FLAG_HIDDEN);

  // Clear previous image source before
  // loading new one to free memory
  update_heartbeat_lvgl();
  lv_img_set_src(s_hud_image, NULL);
  update_heartbeat_lvgl();
  // Removed redundant lv_timer_handler() to
  // prevent potential reentrancy/hangs
  vTaskDelay(pdMS_TO_TICKS(10)); // Allow time for memory cleanup

  // Set image source to load original size
  // (no size constraint) NOTE:
  // lv_img_set_src() may take a long time for
  // PNG decoding
  int64_t set_src_start_us = esp_timer_get_time();
  lv_img_set_src(s_hud_image, img_path);
  int64_t set_src_end_us = esp_timer_get_time();
  int64_t set_src_duration_ms = (set_src_end_us - set_src_start_us) / 1000;
  ESP_LOGI(TAG,
           "HUD: lv_img_set_src() took %lld "
           "ms for %s",
           set_src_duration_ms, img_path);
  lv_obj_invalidate(s_hud_image);

  // LVGL은 이미지를 비동기로 디코딩하므로,
  // 명시적인 디코딩 루프가 필요 없음 이미지를
  // 설정하고 invalidate만 하면 LVGL이
  // 자동으로 백그라운드에서 디코딩함 하트비트
  // 업데이트 (이미지 설정 후)
  update_heartbeat_lvgl();

  // Show image (LVGL이 백그라운드에서
  // 디코딩하는 동안에도 표시 가능)
  lv_obj_clear_flag(s_hud_image, LV_OBJ_FLAG_HIDDEN);

  // 하트비트 업데이트 (이미지 표시 후)
  update_heartbeat_lvgl();

  ESP_LOGI(TAG,
           "HUD: Image file=%s loaded "
           "(decoding in background)",
           img_path);

  // Set position using align (more reliable
  // than set_pos) Calculate offset from
  // center based on sector
  lv_coord_t offset_x = 0;
  lv_coord_t offset_y = 0;

  switch (entry->sector) {
  case 0: // LCD 중앙에서 위로 66pt 이동
    offset_x = 0;
    offset_y = -66;
    break;
  case 12: // LCD 중앙에서 위로 80pt, 우측으로 130pt 이동
    offset_x = 130;
    offset_y = -80;
    break;
  case 6: // LCD 중앙에서 아래로 100pt 이동
    offset_x = 0;
    offset_y = 100;
    break;
  case 3: // LCD 중앙에서 우측으로 100pt 이동
    offset_x = 100;
    offset_y = 0;
    break;
  case 9: // LCD 중앙에서 좌측으로 150pt 이동
    offset_x = -150;
    offset_y = 0;
    break;
  default:
    offset_x = 0;
    offset_y = 0;
    break;
  }

  // Use align to center the image, then apply
  // offset
  lv_obj_align(s_hud_image, LV_ALIGN_CENTER, offset_x, offset_y);
  lv_obj_invalidate(s_hud_image);

  // Ensure image is visible after loading
  lv_obj_clear_flag(s_hud_image, LV_OBJ_FLAG_HIDDEN);
  lv_obj_invalidate(s_hud_image);

  // Final watchdog reset
  // Log removed - only HUD TX data is logged
}

// Reusable line buffer for LVGL flush
// (allocated once in PSRAM, AI_DRV reference)
static uint16_t *s_flush_chunk_buf = NULL;

static inline uint16_t lcd_be16(uint16_t v) {
  // SH8601 QSPI expects big-endian RGB565 in
  // AI_DRV (byte-swapped).
  return (uint16_t)((v >> 8) | (v << 8));
}

// AI_DRV init sequence (QSPI), expressed with
// static buffers to avoid stack lifetime
// issues
static const uint8_t s_cmd_fe_data[] = {0x00};
static const uint8_t s_cmd_c4_data[] = {0x80};
static const uint8_t s_cmd_3a_data[] = {0x55};
static const uint8_t s_cmd_36_data[] = {0x00}; // RGB order
static const uint8_t s_cmd_35_data[] = {0x00};
static const uint8_t s_cmd_53_data[] = {0x20};
static const uint8_t s_cmd_2a_data[] = {0x00, 0x06, 0x01, 0xD7};
static const uint8_t s_cmd_2b_data[] = {0x00, 0x00, 0x01, 0xD1};

static const sh8601_lcd_init_cmd_t s_lcd_init_cmds[] = {
    {0xFE, s_cmd_fe_data, sizeof(s_cmd_fe_data), 0},
    {0xC4, s_cmd_c4_data, sizeof(s_cmd_c4_data), 0},
    {0x3A, s_cmd_3a_data, sizeof(s_cmd_3a_data), 0},
    {0x36, s_cmd_36_data, sizeof(s_cmd_36_data), 0},
    {0x35, s_cmd_35_data, sizeof(s_cmd_35_data), 0},
    {0x53, s_cmd_53_data, sizeof(s_cmd_53_data), 0},
    {0x2A, s_cmd_2a_data, sizeof(s_cmd_2a_data), 0},
    {0x2B, s_cmd_2b_data, sizeof(s_cmd_2b_data), 600},
    {0x11, NULL, 0, 120}, // Sleep Out
    // 0x29 (Display On) removed from here - called explicitly in lvgl_init
};

// Consider the device "actively
// communicating" if we've seen any real
// traffic recently. We update this on
// READ/WRITE/CONF and also on successful
// indication TX.
#define BLE_ACTIVITY_WINDOW_MS (3000)
static TickType_t s_last_ble_activity_tick = 0;
// static bool s_client_started = false; // Legacy residue

// Deleted legacy Bluedroid notification logic residue

void log_ble_packet(const uint8_t *data, size_t len, const char *prefix) {
    if (data == NULL || len < 2 || data[0] != 0x19 || data[len - 1] != 0x2F) {
        return;
    }
    
    uint8_t id = data[1];
    uint8_t cmd = data[2];

    // 1. 패킷 설명(desc) 분석
    const char *desc = "알 수 없는 명령";
    if (len >= 3 && data[0] == 0x19 && (data[1] == 0x4D || data[1] == 0x4E)) {
        uint8_t mcmd = data[2];
        switch (mcmd) {
            case 0x01: desc = "TBT 방향 정보"; break;
            case 0x02: desc = "안전운행(Safety) 정보"; break;
            case 0x03: desc = "주행 속도(Speed) 정보"; break;
            case 0x04: desc = "외곽 링 상태(00:파랑, 01:초록)"; break;
            case 0x05: desc = "화면 클리어(Clear) 명령"; break;
            case 0x06: desc = (len >= 5 && data[4] == 0x03) ? "(펌웨어 정보 문의)" : "화면 밝기 조회(RX)"; break;
            case 0x07: desc = "화면 밝기 설정"; break;
            case 0x09: desc = "시간 설정(Time Set)"; break;
            case 0x0A: desc = "목적지 남은 정보"; break;
            case 0x0B: desc = (len >= 4 && data[3] == 0x02) ? "밝기 설정값 통보(TX)" : "목적지 정보 지움"; break;
            case 0x0C: desc = (len >= 5 && data[4] == 0x01) ? "(내비기능 시작)" : (len >= 5 && data[4] == 0x00) ? "(속도계 모드 시작)" : "(모델명과 펌웨어 버전 송신)"; break;
            case 0x0D: desc = (data[1] == 0x4E || data[1] == 0x4D) ? "시간 업데이트 요청(TX)" : "목적지 도착 알림(RX)"; break;
            case 0x0E: desc = "도로명 정보(Road Name)"; break;
            case 0xFF: desc = "Keep-Alive Heartbeat (TX)"; break;
        }
    } else if (len >= 3 && data[0] == 0x19 && data[1] == 0x59) {
        desc = "설정/동기화 데이터(0x59)";
    } else if (len >= 3 && data[0] == 0x19 && data[1] == 0x50) {
        uint8_t mcmd = data[2];
        switch (mcmd) {
            case 0x01: desc = "이미지 정보 전달(RX)"; break;
            case 0x02: desc = "이미지 데이터 블록(RX)"; break;
            case 0x03: desc = "이미지 전송 결과(Seq)(TX)"; break;
            case 0x04: desc = "이미지 업로드 완료(TX)"; break;
            case 0x05: desc = "이미지 삭제 명령(RX)"; break;
            case 0x06: desc = "이미지 자동 모드 설정(RX)"; break;
            case 0x07: desc = "이미지 상태 요청(TX)"; break;
            case 0x08: desc = "이미지 상태 전달(RX)"; break;
            case 0x11: desc = "이미지 정보 수신 응답(TX)"; break;
        }
    } else if (len >= 3 && data[0] == 0x19 && data[1] == 0x4F) {
        uint8_t mcmd = data[2];
        switch (mcmd) {
            case 0x01: desc = "업데이트정보 수신"; break;
            case 0x02: desc = "업데이트 요청(REQ)(TX)"; break;
            case 0x03: desc = "펌웨어 데이터(Data)(RX)"; break;
            case 0x04: desc = "데이터 수신 응답(ACK)(TX)"; break;
            case 0x05: desc = "업데이트 완료 통보(TX)"; break;
        }
    }

    // 2. 시간 및 터미널 출력용 문자열 생성
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm timeinfo;
    localtime_r(&tv.tv_sec, &timeinfo);

    // 3. 뮤텍스 보호 하에 static 버퍼 사용하여 HEX 문자열 생성 및 출력
    if (s_pkt_log_mutex == NULL) {
        s_pkt_log_mutex = xSemaphoreCreateMutex();
    }

    if (xSemaphoreTake(s_pkt_log_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // [MOD] Silence only very heavy binary data flows if requested, 
        // but now explicitly enabling 0x50 for tracking
        bool is_bulk_data = (id == 0x4F && cmd == 0x03); 
        
        static char hex_str[800];
        memset(hex_str, 0, sizeof(hex_str));
        int pos = 0;
        
        // 0x50 02(IMAGE DATA) and 0x4F 03(FW DATA) get 10-byte truncation
        size_t limit = len - 1;
        bool truncated = false;
        if (((id == 0x50 && cmd == 0x02) || (id == 0x4F && cmd == 0x03)) && len > 11) {
            limit = 11; // index 1 ~ 10 (10 bytes)
            truncated = true;
        }

        for (size_t i = 1; i < limit && pos < (sizeof(hex_str) - 8); i++) {
            pos += snprintf(hex_str + pos, sizeof(hex_str) - pos, "%02X ", data[i]);
        }
        if (truncated) {
            strcat(hex_str, "... ");
        }

        if (!is_bulk_data || (id == 0x50 && cmd == 0x02)) { // Show image data even if bulk
            ESP_LOGW(TAG, "PKT_LOG [%02d:%02d:%02d] [%s] %s// %s", 
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, 
                     (prefix ? prefix : "??"), hex_str, desc);
        }
        xSemaphoreGive(s_pkt_log_mutex);
    }
}

void update_album_option_from_ble(uint8_t mode) {
  s_album_option = (mode == 0) ? 0 : 1;
  save_nvs_settings();
  LVGL_LOCK();
  update_setting_ui_labels();
  LVGL_UNLOCK();
  ESP_LOGI(TAG, "Album mode update from BLE: %s", (mode == 0)?"AUTO":"MANUAL");
}

/* ---------------------------------------------------------------------------
   On-screen clock
   ---------------------------------------------------------------------------
 */

// Font data removed - now using LVGL fonts

// LVGL display flush callback - sends pixel
// buffer to LCD (AI_DRV reference)

// LCD transfer completion callback
static bool lvgl_on_flush_ready_cb(esp_lcd_panel_io_handle_t panel_io,
                                   esp_lcd_panel_io_event_data_t *edata,
                                   void *user_ctx) {
  BaseType_t high_task_wakeup = pdFALSE;
  if (s_lcd_flush_sem) {
    xSemaphoreGiveFromISR(s_lcd_flush_sem, &high_task_wakeup);
  }
  return high_task_wakeup == pdTRUE;
}

static void lvgl_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area,
                            lv_color_t *color_p) {
  if (s_lcd_panel == NULL) {
    lv_disp_flush_ready(disp_drv);
    return;
  }

  int32_t width = (area->x2 - area->x1 + 1);
  int32_t height = (area->y2 - area->y1 + 1);

  // Safety clamp to prevent out-of-bounds coordinates (especially with
  // sh8601 offset)
  if (area->x1 + width > LCD_H_RES)
    width = LCD_H_RES - area->x1;
  if (area->y1 + height > LCD_V_RES)
    height = LCD_V_RES - area->y1;

  if (width <= 0 || height <= 0) {
    lv_disp_flush_ready(disp_drv);
    return;
  }

  // Process in chunks to avoid SPI
  // transaction limits (AI_DRV reference)
  int lines_remaining = height;
  int y_start = area->y1;
  const lv_color_t *src_ptr = color_p;

  // Allocate reusable buffer for one chunk on
  // first use (Internal RAM for DMA reliability)
  // Allocate reusable buffer for one chunk on
  // first use (Try PSRAM first to save internal RAM for Wi-Fi)
  // Allocate reusable buffer for one chunk on
  // first use (Internal RAM for DMA reliability)
  // Reverted to Internal RAM but smaller chunk size
  if (!s_flush_chunk_buf) {
    size_t max_pixels = LCD_H_RES * LCD_FLUSH_CHUNK_HEIGHT;
    s_flush_chunk_buf = (uint16_t *)heap_caps_malloc(
        max_pixels * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    if (!s_flush_chunk_buf) {
      ESP_LOGE(TAG, "LVGL: Flush buffer alloc failed (Internal)");
      lv_disp_flush_ready(disp_drv);
      return;
    }
  }

  while (lines_remaining > 0) {
    int lines_to_copy = (lines_remaining > LCD_FLUSH_CHUNK_HEIGHT)
                            ? LCD_FLUSH_CHUNK_HEIGHT
                            : lines_remaining;

    // Byte swap and copy to chunk buffer
    // (AI_DRV reference: (c >> 8) | (c << 8))
    int pixel_count = width * lines_to_copy;
    for (int i = 0; i < pixel_count; i++) {
      uint16_t c = src_ptr[i].full;
      s_flush_chunk_buf[i] = (c >> 8) | (c << 8);
    }

    // Draw chunk (with retry if queue is full)
    esp_err_t ret;
    int retry = 0;
    do {
      ret = esp_lcd_panel_draw_bitmap(s_lcd_panel, area->x1, y_start,
                                      area->x1 + width, y_start + lines_to_copy,
                                      s_flush_chunk_buf);
      if (ret != ESP_OK) {
        // If queue is full, wait for any previous transfer to complete
        if (s_lcd_flush_sem) {
          xSemaphoreTake(s_lcd_flush_sem, pdMS_TO_TICKS(10));
        } else {
          vTaskDelay(1);
        }
        retry++;
      }
    } while (ret != ESP_OK && retry < 20);

    if (ret == ESP_OK && s_lcd_flush_sem) {
      // Wait for THIS chunk to complete to maintain sync and prevent overflow
      xSemaphoreTake(s_lcd_flush_sem, pdMS_TO_TICKS(100));
    }

    y_start += lines_to_copy;
    src_ptr += pixel_count;
    lines_remaining -= lines_to_copy;

    update_heartbeat_lvgl();
  }

  lv_disp_flush_ready(disp_drv);
}

// Initialize LCD panel (QSPI SH8601)
static esp_err_t lcd_init_panel(void) {
  if (s_lcd_panel != NULL) {
    return ESP_OK; // Already initialized
  }

  if (s_lcd_flush_sem == NULL) {
    s_lcd_flush_sem = xSemaphoreCreateBinary();
  }
  if (s_lvgl_mutex == NULL) {
    s_lvgl_mutex = xSemaphoreCreateRecursiveMutex();
  }

  // Initialize LCD_VCI_EN pin (GPIO 18) - Power enable for LCD
  gpio_config_t vci_en_conf = {
      .pin_bit_mask = (1ULL << PIN_NUM_LCD_VCI_EN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t ret = gpio_config(&vci_en_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LCD: Failed to configure LCD_VCI_EN pin (%s)",
             esp_err_to_name(ret));
    return ret;
  }

  // Set LCD_VCI_EN to HIGH to enable LCD power
  gpio_set_level(PIN_NUM_LCD_VCI_EN, 1);
  ESP_LOGI(TAG, "LCD: LCD_VCI_EN (GPIO %d) set to HIGH", PIN_NUM_LCD_VCI_EN);

  // Wait for LCD power to stabilize
  vTaskDelay(pdMS_TO_TICKS(100));

  // Increase image cache size to 4 to utilize PSRAM effectively.
  // Each 466x466 RGB565 image takes ~434KB. 4 images = ~1.7MB.
  lv_img_cache_set_size(4);
  // QSPI-only init (mirrors AI_DRV). No
  // fallback to avoid masking wiring/board
  // issues. DMA는 SPI_DMA_CH_AUTO로
  // 활성화되어 있어 QSPI 전송 시 CPU 개입을
  // 최소화합니다. max_transfer_sz를 충분히
  // 크게 설정하여 큰 전송도 DMA로 처리되도록
  // 합니다.
  spi_bus_config_t buscfg = {
      .sclk_io_num = PIN_NUM_LCD_PCLK,
      .data0_io_num = PIN_NUM_LCD_DATA0,
      .data1_io_num = PIN_NUM_LCD_DATA1,
      .data2_io_num = PIN_NUM_LCD_DATA2,
      .data3_io_num = PIN_NUM_LCD_DATA3,
      .max_transfer_sz =
          LCD_H_RES * LCD_V_RES * 3 * 2, // 충분한 크기로 설정하여 DMA
                                         // 사용 보장
      .flags = SPICOMMON_BUSFLAG_QUAD,
  };
  ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LCD: QSPI bus init failed (%s)", esp_err_to_name(ret));
    return ret;
  }

  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config =
      (esp_lcd_panel_io_spi_config_t)SH8601_PANEL_IO_QSPI_CONFIG(
          PIN_NUM_LCD_CS, lvgl_on_flush_ready_cb, NULL);
  // Increase SPI queue depth to prevent "spi
  // transmit (queue) color failed" errors
  // Default is 10, increase to 30 for better performance during mode switching
  io_config.trans_queue_depth = 30;
  io_config.pclk_hz = 20 * 1000 * 1000;

  ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config,
                                 &io_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LCD: new_panel_io failed (%s)", esp_err_to_name(ret));
    spi_bus_free(LCD_HOST);
    return ret;
  }

  // Save IO handle for brightness control
  s_lcd_io_handle = io_handle;

  sh8601_vendor_config_t vendor_config = {
      .init_cmds = s_lcd_init_cmds,
      .init_cmds_size =
          (uint16_t)(sizeof(s_lcd_init_cmds) / sizeof(s_lcd_init_cmds[0])),
      .flags = {.use_qspi_interface = 1},
  };
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = PIN_NUM_LCD_RST,
      .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
      .bits_per_pixel = LCD_BITS_PER_PIXEL,
      .vendor_config = &vendor_config,
  };

  ret = esp_lcd_new_panel_sh8601(io_handle, &panel_config, &s_lcd_panel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LCD: new_panel_sh8601 failed (%s)", esp_err_to_name(ret));
    esp_lcd_panel_io_del(io_handle);
    spi_bus_free(LCD_HOST);
    return ret;
  }

  ret = esp_lcd_panel_reset(s_lcd_panel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LCD: panel_reset failed (%s)", esp_err_to_name(ret));
    return ret;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
  ret = esp_lcd_panel_init(s_lcd_panel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LCD: panel_init failed (%s)", esp_err_to_name(ret));
    return ret;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
  // Display On and Brightness setting moved to lvgl_init
  // to prevent brown/green flash before first frame is ready.

  ESP_LOGI(TAG, "LCD: panel initialized (Display Off, Backlight Off)");
  return ESP_OK;
}

// Font loader helper
static lv_font_t *load_font_from_fs(const char *font_name) {
  char path[128];
  lv_font_t *font = NULL;

  // Try multiple paths with LVGL POSIX drive letter ('L:')
  const char *base_paths[] = {"L:/", "L:/fonts/", "L:/flash_data/"};

  for (int i = 0; i < sizeof(base_paths) / sizeof(base_paths[0]); i++) {
    snprintf(path, sizeof(path), "%s%s.bin", base_paths[i], font_name);
    font = lv_font_load(path);
    if (font) {
      ESP_LOGI(TAG, "Font: Successfully loaded %s from %s", font_name, path);
      return font;
    }
  }

  ESP_LOGE(TAG, "Font: Failed to load %s from any expected path", font_name);
  return NULL;
}

// Load all required fonts
static void load_all_fonts(void) {
  ESP_LOGI(TAG, "Font: Starting bulk font loading...");
  s_font_kopub_20 = load_font_from_fs("font_kopub_20"); vTaskDelay(pdMS_TO_TICKS(20));
  s_font_kopub_25 = load_font_from_fs("font_kopub_25"); vTaskDelay(pdMS_TO_TICKS(20));

  s_font_kopub_35 = load_font_from_fs("font_kopub_35"); vTaskDelay(pdMS_TO_TICKS(20));
  s_font_kopub_40 = load_font_from_fs("font_kopub_40"); vTaskDelay(pdMS_TO_TICKS(20));
  s_font_orb_100 = load_font_from_fs("font_orb_100");  vTaskDelay(pdMS_TO_TICKS(20));
  s_font_orb_155 = load_font_from_fs("font_orb_155");  vTaskDelay(pdMS_TO_TICKS(20));
  
  /* ESP_LOGI(TAG, "Font: Attempting to load gman_188...");
  s_font_gman_188 = load_font_from_fs("font_gman_188"); vTaskDelay(pdMS_TO_TICKS(20)); */
  
  ESP_LOGI(TAG, "Font: Attempting to load addr_30...");
  s_font_addr_30 = load_font_from_fs("font_addr_30");   vTaskDelay(pdMS_TO_TICKS(20));



  // Link font_gman_188 as fallback for font_orb_155 if both are available
  /* if (s_font_orb_155 && s_font_gman_188) {
    s_font_orb_155->fallback = s_font_gman_188;
    ESP_LOGI(TAG, "Font: Linked font_gman_188 as fallback for font_orb_155");
  } */
  ESP_LOGI(TAG, "Font: All fonts processed.");
}

// LVGL tick callback (AI_DRV reference: ESP
// timer)
static void lvgl_tick_cb(void *arg) {
  (void)arg;
  lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// LVGL log callback to route LVGL logs to
// ESP-IDF logging
#if LV_USE_LOG
static void lvgl_log_cb(const char *buf) { ESP_LOGW(TAG, "LVGL: %s", buf); }
#endif

// Initialize LVGL and create UI elements
// (AI_DRV reference)
static esp_err_t lvgl_init(void) {
  // Initialize LVGL
  lv_init();

  // Register LVGL log callback to see decoder
  // errors
#if LV_USE_LOG
  lv_log_register_print_cb(lvgl_log_cb);
#endif

  // Initialize POSIX file system driver for
  // LVGL (to access LittleFS files) ESP-IDF VFS
  // provides POSIX interface, so we can use it
  // directly
#if LV_USE_FS_POSIX
  lv_fs_posix_init();
  ESP_LOGI(TAG,
           "LVGL: POSIX file system driver "
           "initialized (letter='%c', path='%s')",
           (char)LV_FS_POSIX_LETTER, LV_FS_POSIX_PATH);
#endif

  // Load fonts from LittleFS before UI creation
  load_all_fonts();

  // Initialize PNG decoder (uses lodepng
  // library, allocates memory via
  // lvgl_psram_malloc)
#if LV_USE_PNG
  extern void lv_png_init(void);
  lv_png_init();
  ESP_LOGI(TAG, "LVGL: PNG decoder initialized "
                "(lodepng library, uses PSRAM "
                "via custom allocator)");
#else
  ESP_LOGW(TAG, "LVGL: PNG decoder not enabled "
                "(LV_USE_PNG is not set)");
#endif

#if LV_USE_GIF
  // extern void lv_gif_init(void);
  // lv_gif_init();
  ESP_LOGI(TAG, "LVGL: GIF decoder (built-in) enabled");
#endif

#if LV_USE_BMP
  extern void lv_bmp_init(void);
  lv_bmp_init();
  ESP_LOGI(TAG, "LVGL: BMP decoder initialized");
#endif

#if LV_USE_SJPG
  extern void lv_split_jpeg_init(void);
  lv_split_jpeg_init();
  ESP_LOGI(TAG, "LVGL: SJPG (JPG) decoder initialized");
#endif

  // Register 'S' drive for SD card / custom
  // paths
  static lv_fs_drv_t sd_fs_drv;
  lv_fs_drv_init(&sd_fs_drv);
  sd_fs_drv.letter = 'S';
  sd_fs_drv.open_cb = lv_fs_open_sd;
  sd_fs_drv.close_cb = lv_fs_close_sd;
  sd_fs_drv.read_cb = lv_fs_read_sd;
  sd_fs_drv.seek_cb = lv_fs_seek_sd;
  sd_fs_drv.tell_cb = lv_fs_tell_sd;
  lv_fs_drv_register(&sd_fs_drv);
  ESP_LOGI(TAG, "LVGL: 'S:' drive registered "
                "for custom path access");

  // Allocate display buffers in PSRAM (AI_DRV
  // reference: full screen buffer)
  size_t buf_size = LCD_H_RES * LVGL_BUFFER_HEIGHT * sizeof(lv_color_t);
  s_disp_buf1 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
  s_disp_buf2 = (lv_color_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);

  if (!s_disp_buf1 || !s_disp_buf2) {
    ESP_LOGE(TAG, "LVGL: Failed to allocate buffers");
    if (s_disp_buf1)
      free(s_disp_buf1);
    if (s_disp_buf2)
      free(s_disp_buf2);
    return ESP_ERR_NO_MEM;
  }
  // Zero-initialize buffers to black
  memset(s_disp_buf1, 0, buf_size);
  memset(s_disp_buf2, 0, buf_size);

  // Initialize display draw buffer
  lv_disp_draw_buf_init(&s_disp_draw_buf, s_disp_buf1, s_disp_buf2,
                        LCD_H_RES * LVGL_BUFFER_HEIGHT);

  // Initialize display driver
  lv_disp_drv_init(&s_disp_drv);
  s_disp_drv.hor_res = LCD_H_RES;
  s_disp_drv.ver_res = LCD_V_RES;
  s_disp_drv.flush_cb = lvgl_disp_flush;
  s_disp_drv.draw_buf = &s_disp_draw_buf;
  // 화면 깨짐 방지를 위해 전체 리프레시 사용
  // (AI_DRV reference)
  s_disp_drv.full_refresh = 1;
  // Fix for small widget size: Force DPI to a
  // lower value (AI_DRV reference)
  s_disp_drv.dpi = 130;

  s_disp = lv_disp_drv_register(&s_disp_drv);

  // Set image cache size for better JPG/PNG decoding performance
  // Cache up to 4 decoded images in memory. Each 466x466 RGB565 image takes ~434KB. 4 images = ~1.7MB.
  lv_img_cache_set_size(4);
  ESP_LOGI(TAG, "LVGL: Image cache set to 4 entries (aggressive PSRAM usage)");

  // Create screens for different modes
  s_boot_screen = lv_obj_create(NULL);
  s_hud_screen = lv_obj_create(NULL);
  s_speedometer_screen = lv_obj_create(NULL);
  s_clock_screen = lv_obj_create(NULL);
  s_album_screen = lv_obj_create(NULL);
  s_setting_screen = lv_obj_create(NULL);
  s_ota_screen = lv_obj_create(NULL);

  lv_obj_set_style_bg_color(s_boot_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(s_boot_screen, LV_OPA_COVER, 0); 
  lv_obj_set_style_bg_color(s_hud_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_color(s_speedometer_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_color(s_clock_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_color(s_album_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_color(s_setting_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_color(s_ota_screen, lv_color_black(), 0);

  // Load HUD screen as default (so lv_scr_act() returns it for following
  // creations)
  lv_scr_load(s_hud_screen);

  // Immediately render black screen to prevent green flash on boot
  lv_timer_handler();
  lv_refr_now(NULL);

  // Now that a black frame is ready, turn ON display and set brightness
  esp_lcd_panel_disp_on_off(s_lcd_panel, true);
  vTaskDelay(pdMS_TO_TICKS(100));
  set_lcd_brightness(s_brightness_level, true); // [User Request] Use restored NVS level instead of hardcoded 0

  ESP_LOGI(TAG, "LVGL: Black screen rendered and panel active");

  // Create intro image object immediately to prevent green flash
  s_intro_image = lv_img_create(lv_scr_act());
  lv_obj_add_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN); // Initially hidden
  lv_obj_center(s_intro_image);
  ESP_LOGI(TAG, "LVGL: Intro image object created (hidden)");

  // Initialize Touch
  if (init_touch() == ESP_OK) {
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read_cb;
    indev_drv.gesture_limit = 30; // 픽셀 이동 기준 감도 향상
    indev_drv.gesture_min_velocity = 10;
    s_touch_indev = lv_indev_drv_register(&indev_drv);
  }

  // Create UI for Clock and Album modes (background ready)
  create_boot_ui();
  create_clock_ui();
  create_clock2_ui();      // Create Clock 2
  create_speedometer_ui(); // Create Speedometer
  create_album_ui();
  create_setting_ui();
  create_ota_ui();

  // Create HYD TX message label (중앙에서
  // 50pt 아래, 30pt 크기) - 앱->ESP32
  s_hyd_msg_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_hyd_msg_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_hyd_msg_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_obj_align(s_hyd_msg_label, LV_ALIGN_CENTER, 0,
               50); // 중앙에서 50pt 아래
  lv_label_set_text(s_hyd_msg_label, "");
  lv_obj_set_style_text_align(s_hyd_msg_label, LV_TEXT_ALIGN_CENTER, 0);

  // Create HUD TX message label (중앙에서
  // 100pt 아래, 30pt 크기) - ESP32->앱 (FFF1
  // 채널)
  s_hud_tx_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_hud_tx_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_hud_tx_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_obj_align(s_hud_tx_label, LV_ALIGN_CENTER, 0,
               100); // 중앙에서 100pt 아래
  lv_label_set_text(s_hud_tx_label, "");
  lv_obj_set_style_text_align(s_hud_tx_label, LV_TEXT_ALIGN_CENTER, 0);

  // Create ESP timer for LVGL tick (AI_DRV
  // reference)
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = lvgl_tick_cb,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "lvgl_tick",
      .skip_unhandled_events = false};
  static esp_timer_handle_t lvgl_tick_timer;
  esp_err_t ret = esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LVGL: Failed to create tick timer");
    return ret;
  }
  ret = esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LVGL: Failed to start tick timer");
    return ret;
  }

  // Create HUD mode large label (중앙, 큰
  // 폰트) - HUD 모드용
  s_hud_tx_label_large = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_hud_tx_label_large, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_hud_tx_label_large, &lv_font_montserrat_48,
                             0); // 48pt 폰트
  lv_obj_center(s_hud_tx_label_large);
  lv_label_set_text(s_hud_tx_label_large, "");
  lv_obj_set_style_text_align(s_hud_tx_label_large, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_add_flag(s_hud_tx_label_large,
                  LV_OBJ_FLAG_HIDDEN); // 초기에는 숨김

  // Create HUD mode image (중앙) - HUD 모드용
  s_hud_image = lv_img_create(s_hud_screen);
  lv_obj_center(s_hud_image);
  // 이미지 소스 설정 (LittleFS 경로)
  // POSIX 드라이버 사용 시 드라이브 레터
  // 형식: "A:/image/arrow_0.bmp" LVGL이
  // 드라이브 레터를 인식하고 나머지 경로만
  // 드라이버에 전달 드라이버는
  // LV_FS_POSIX_PATH("/littlefs")와 결합하여
  // "/littlefs/image/arrow_0.bmp" 생성 이미지
  // 소스는 LittleFS 마운트 후에 설정됨
  // (init_littlefs() 후) 여기서는 객체만
  // 생성하고 소스는 설정하지 않음
  lv_obj_add_flag(s_hud_image,
                  LV_OBJ_FLAG_HIDDEN); // 초기에는 숨김

  // Create Safety length value label
  // (safety거리값, 25pt 폰트, 녹색)
  // safety거리값은 LCD 중앙에서 우측으로
  // 130pt 이동, 위로 9pt 이동
  s_safety_length_value_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_safety_length_value_label,
                              lv_color_hex(0x00FF00),
                              0); // 녹색
  lv_obj_set_style_text_font(s_safety_length_value_label, &font_kopub_35,
                             0); // 35pt 폰트로 상향 (기존 25pt)
  lv_obj_align(s_safety_length_value_label, LV_ALIGN_CENTER, 130,
               -9); // 중앙에서 우측으로 130pt, 위로 9pt
  lv_label_set_text(s_safety_length_value_label, "");
  lv_obj_set_style_text_align(s_safety_length_value_label, LV_TEXT_ALIGN_CENTER,
                              0);
  lv_obj_add_flag(s_safety_length_value_label,
                  LV_OBJ_FLAG_HIDDEN); // 초기에는 숨김

  // Create Safety length unit label
  // (safety단위, 25pt 폰트, 회색)
  s_safety_length_unit_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_safety_length_unit_label,
                              lv_color_hex(0xCCCCCC),
                              0); // 회색
  lv_obj_set_style_text_font(s_safety_length_unit_label, &font_kopub_25,
                             0); // 25pt 폰트
  lv_obj_align_to(s_safety_length_unit_label, s_safety_length_value_label,
                  LV_ALIGN_OUT_RIGHT_MID, 5, 0); // 값 레이블 우측 5pt
  lv_label_set_text(s_safety_length_unit_label, "");
  lv_obj_set_style_text_align(s_safety_length_unit_label, LV_TEXT_ALIGN_LEFT,
                              0);
  lv_obj_add_flag(s_safety_length_unit_label,
                  LV_OBJ_FLAG_HIDDEN); // 초기에는 숨김

  // Create Normal Speed labels
  s_normal_speed_value_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_normal_speed_value_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_normal_speed_value_label, &font_ORB_155, 0);
  lv_obj_align(s_normal_speed_value_label, LV_ALIGN_CENTER, 0, 120);
  lv_obj_add_flag(s_normal_speed_value_label, LV_OBJ_FLAG_HIDDEN);

  s_normal_speed_unit_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_normal_speed_unit_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_normal_speed_unit_label, &font_kopub_35, 0);
  lv_obj_align(s_normal_speed_unit_label, LV_ALIGN_CENTER, 0, 190);
  lv_label_set_text(s_normal_speed_unit_label, "km/h");
  lv_obj_add_flag(s_normal_speed_unit_label, LV_OBJ_FLAG_HIDDEN);

  // Create Average Speed labels
  s_avr_speed_title_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_avr_speed_title_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_avr_speed_title_label, &font_kopub_20,
                             0); // 20pt 한글 폰트
  lv_label_set_text(s_avr_speed_title_label, "구간속도");
  lv_obj_align(s_avr_speed_title_label, LV_ALIGN_CENTER, 70,
               200); // LCD 중앙에서 아래로 200pt
  lv_obj_add_flag(s_avr_speed_title_label, LV_OBJ_FLAG_HIDDEN);

  s_avr_speed_value_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_avr_speed_value_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_avr_speed_value_label, &font_kopub_25,
                             0); // 25pt 폰트
  lv_obj_align(s_avr_speed_value_label, LV_ALIGN_CENTER, 130,
               200); // LCD 중앙에서 아래로 200pt
  lv_obj_add_flag(s_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);

  s_avr_speed_unit_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_avr_speed_unit_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_avr_speed_unit_label, &font_kopub_20,
                             0); // 20pt 폰트
  lv_label_set_text(s_avr_speed_unit_label, "km/h");
  lv_obj_align_to(s_avr_speed_unit_label, s_avr_speed_value_label,
                  LV_ALIGN_OUT_RIGHT_BOTTOM, 2, 0);
  lv_obj_add_flag(s_avr_speed_unit_label, LV_OBJ_FLAG_HIDDEN);

  // Create Time date labels (30pt 폰트) -
  // "MM월dd일" 위치: LCD 중앙에서 우측으로
  // 100pt, 아래로 10pt 이동 MM, dd, 월, 일:
  // 모두 회색

  // MM (회색, 숫자이므로 montserrat 폰트)
  s_time_month_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_time_month_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_time_month_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_obj_align(s_time_month_label, LV_ALIGN_CENTER, 100,
               10); // 기준 위치: 중앙에서 우측으로
                    // 100pt, 아래로 10pt
  lv_label_set_text(s_time_month_label, "");
  lv_obj_set_style_text_align(s_time_month_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_time_month_label, LV_OBJ_FLAG_HIDDEN);

  // 월 (회색, 한글이므로 malrang 폰트)
  s_time_month_unit_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_time_month_unit_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_time_month_unit_label, &font_kopub_35,
                             0); // 30pt 한글 폰트
  lv_label_set_text(s_time_month_unit_label, "");
  lv_obj_set_style_text_align(s_time_month_unit_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_time_month_unit_label, LV_OBJ_FLAG_HIDDEN);

  // dd (회색, 숫자이므로 montserrat 폰트)
  s_time_day_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_time_day_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_time_day_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_label_set_text(s_time_day_label, "");
  lv_obj_set_style_text_align(s_time_day_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_time_day_label, LV_OBJ_FLAG_HIDDEN);

  // 일 (회색, 한글이므로 malrang 폰트)
  s_time_day_unit_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_time_day_unit_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_time_day_unit_label, &font_kopub_35,
                             0); // 30pt 한글 폰트
  lv_label_set_text(s_time_day_unit_label, "");
  lv_obj_set_style_text_align(s_time_day_unit_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_time_day_unit_label, LV_OBJ_FLAG_HIDDEN);

  // Create Time time labels (30pt 폰트) -
  // "HH:mm:ss" 위치: LCD 중앙에서 우측으로
  // 100pt, 아래로 50pt 이동 HH, mm, ss, ::
  // 모두 회색

  // HH (회색)
  s_time_hour_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_time_hour_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_time_hour_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_obj_align(s_time_hour_label, LV_ALIGN_CENTER, 100,
               50); // 기준 위치: 중앙에서 우측으로
                    // 100pt, 아래로 50pt
  lv_label_set_text(s_time_hour_label, "");
  lv_obj_set_style_text_align(s_time_hour_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_time_hour_label, LV_OBJ_FLAG_HIDDEN);

  // : (회색)
  s_time_colon1_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_time_colon1_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_time_colon1_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_label_set_text(s_time_colon1_label, "");
  lv_obj_set_style_text_align(s_time_colon1_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_time_colon1_label, LV_OBJ_FLAG_HIDDEN);

  // mm (회색)
  s_time_minute_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_time_minute_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_time_minute_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_label_set_text(s_time_minute_label, "");

  // --- HUD Screen (NAVI) Speed Labels ---
  s_speed_mark_value_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_font(s_speed_mark_value_label, &font_ORB_155, 0);
  lv_obj_set_style_text_color(s_speed_mark_value_label, lv_color_white(), 0);
  lv_obj_align(s_speed_mark_value_label, LV_ALIGN_CENTER, 0, 83);
  lv_label_set_text(s_speed_mark_value_label, "0");
  lv_obj_add_flag(s_speed_mark_value_label, LV_OBJ_FLAG_HIDDEN);

  s_speed_mark_unit_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_font(s_speed_mark_unit_label, &font_kopub_35, 0);
  lv_obj_set_style_text_color(s_speed_mark_unit_label, lv_color_white(), 0);
  lv_obj_align(s_speed_mark_unit_label, LV_ALIGN_CENTER, 0, 164);
  lv_label_set_text(s_speed_mark_unit_label, "km/h");
  lv_obj_add_flag(s_speed_mark_unit_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_text_align(s_time_minute_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_time_minute_label, LV_OBJ_FLAG_HIDDEN);

  // : (회색)
  s_time_colon2_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_time_colon2_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_time_colon2_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_label_set_text(s_time_colon2_label, "");
  lv_obj_set_style_text_align(s_time_colon2_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_time_colon2_label, LV_OBJ_FLAG_HIDDEN);

  // ss (회색)
  s_time_second_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(s_time_second_label, lv_color_white(),
                              0); // 흰색
  lv_obj_set_style_text_font(s_time_second_label, &font_kopub_35,
                             0); // 30pt 폰트
  lv_label_set_text(s_time_second_label, "");
  lv_obj_set_style_text_align(s_time_second_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_time_second_label, LV_OBJ_FLAG_HIDDEN);

  // Create 목적지남은시간값 label (30pt 폰트,
  // 노랑색) 위치: LCD 중앙에서 위로 5pt,
  // 레이블 중심 기준으로 좌측으로 150pt
  s_dest_time_value_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_dest_time_value_label,
                              lv_color_hex(0x00FF00),
                              0); // 밝은 초록색 (Bright Green)
  lv_obj_set_style_text_font(s_dest_time_value_label, &font_kopub_35, 0);
  // Create 목적지남은시간 "시간" 단위 label (30pt 폰트, 회색)
  s_dest_time_hour_unit_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_dest_time_hour_unit_label, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_set_style_text_font(s_dest_time_hour_unit_label, &font_addr_30, 0);
  lv_label_set_text(s_dest_time_hour_unit_label, "");
  lv_obj_set_style_text_align(s_dest_time_hour_unit_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_dest_time_hour_unit_label, LV_OBJ_FLAG_HIDDEN);

  // Create 목적지남은시간 분 숫자 label 
  s_dest_time_minute_value_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_dest_time_minute_value_label, lv_color_hex(0x00FF00), 0); // 밝은 초록색 (Bright Green)
  lv_obj_set_style_text_font(s_dest_time_minute_value_label, &font_kopub_35, 0);
  lv_label_set_text(s_dest_time_minute_value_label, "");
  lv_obj_set_style_text_align(s_dest_time_minute_value_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_dest_time_minute_value_label, LV_OBJ_FLAG_HIDDEN);

  // Create "소요시간" label
  s_dest_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_dest_label, lv_color_make(180, 160, 0), 0); // 어두운 노랑색 (Dark Yellow)
  lv_obj_set_style_text_font(s_dest_label, &font_addr_30, 0);
  lv_obj_align(s_dest_label, LV_ALIGN_CENTER, -100, -53);
  lv_label_set_text(s_dest_label, "소요시간");
  lv_obj_add_flag(s_dest_label, LV_OBJ_FLAG_HIDDEN);

  // Road Name Labels (Yellow)
  s_road_name_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_road_name_label, lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_text_font(s_road_name_label, &font_addr_30, 0);
  lv_obj_set_style_text_align(s_road_name_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(s_road_name_label, LV_ALIGN_TOP_MID, 0, 388);
  lv_label_set_text(s_road_name_label, "");
  lv_obj_add_flag(s_road_name_label, LV_OBJ_FLAG_HIDDEN);

  s_road_name_sub_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_road_name_sub_label, lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_text_font(s_road_name_sub_label, &font_addr_30, 0);
  lv_obj_set_style_text_align(s_road_name_sub_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(s_road_name_sub_label, LV_ALIGN_TOP_MID, 0, 423);
  lv_label_set_text(s_road_name_sub_label, "");
  lv_obj_add_flag(s_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);

  // TBT Distance Labels (NAVI)
  s_length_tbt_value_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_length_tbt_value_label, lv_color_hex(0x00FF00), 0);
  lv_obj_set_style_text_font(s_length_tbt_value_label, &font_kopub_40, 0);
  lv_label_set_text(s_length_tbt_value_label, "");
  lv_obj_add_flag(s_length_tbt_value_label, LV_OBJ_FLAG_HIDDEN);

  s_length_tbt_unit_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_length_tbt_unit_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_length_tbt_unit_label, &font_kopub_35, 0);
  lv_label_set_text(s_length_tbt_unit_label, "");
  lv_obj_add_flag(s_length_tbt_unit_label, LV_OBJ_FLAG_HIDDEN);

  // Create 목적지남은시간단위 label
  s_dest_time_unit_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_dest_time_unit_label, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_set_style_text_font(s_dest_time_unit_label, &font_addr_30, 0);
  lv_label_set_text(s_dest_time_unit_label, "");
  lv_obj_set_style_text_align(s_dest_time_unit_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_dest_time_unit_label, LV_OBJ_FLAG_HIDDEN);

  // Create 목적지남은거리값 label (하늘색)
  s_dest_distance_value_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_dest_distance_value_label, lv_color_make(135, 206, 235), 0);
  lv_obj_set_style_text_font(s_dest_distance_value_label, &font_kopub_35, 0);
  lv_obj_align(s_dest_distance_value_label, LV_ALIGN_LEFT_MID, 103, 25);
  lv_label_set_text(s_dest_distance_value_label, "");
  lv_obj_set_style_text_align(s_dest_distance_value_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_dest_distance_value_label, LV_OBJ_FLAG_HIDDEN);

  // Create 목적지남은거리단위 label (회색) - "m", "km"
  s_dest_distance_unit_label = lv_label_create(s_hud_screen);
  lv_obj_set_style_text_color(s_dest_distance_unit_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(s_dest_distance_unit_label, &lv_font_montserrat_20, 0);
  lv_label_set_text(s_dest_distance_unit_label, "");
  lv_obj_set_style_text_align(s_dest_distance_unit_label, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_add_flag(s_dest_distance_unit_label, LV_OBJ_FLAG_HIDDEN);

  return ESP_OK;
}
















static void update_display_mode_ui(display_mode_t mode) {
  // Determine if we are in a "functioning" state (App connected or Virtual Drive)
  bool is_active_state = s_connected || s_virt_drive_active;

  // [Fix] Intro Image Overlap Bug:
  // Hide intro image if we are not in BOOT mode, OR if connected, OR if QR Registration is shown.
  if (s_intro_image) {
    if (mode == DISPLAY_MODE_BOOT && !is_active_state && !s_boot_reg_shown) {
      lv_obj_clear_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN);
      lv_obj_move_foreground(s_intro_image);
    } else {
      lv_obj_add_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN);
    }
  }

  switch (mode) {
  case DISPLAY_MODE_BOOT:
    if (lv_scr_act() != s_boot_screen) lv_scr_load(s_boot_screen);
    // [Fix] BOOT 모드에서는 시계와 날짜를 절대 표시하지 않음 (로고 전용)
    if (s_boot_time_label) lv_obj_add_flag(s_boot_time_label, LV_OBJ_FLAG_HIDDEN);
    if (s_boot_date_label) lv_obj_add_flag(s_boot_date_label, LV_OBJ_FLAG_HIDDEN);

    // 10초 타임아웃이 지났고 앱 연결 전이라면 안내 라벨 표시
    // [Fix] iOS 자동 재연결 대응: s_connected 대신 실제 앱 명령 수신 여부로만 판단
    if (s_boot_reg_shown && !s_hud_seen_first_cmd) {
      if (s_boot_reg_title_label) lv_obj_clear_flag(s_boot_reg_title_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_reg_val_label) lv_obj_clear_flag(s_boot_reg_val_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_install_title_label) lv_obj_clear_flag(s_boot_install_title_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_install_sub_label) lv_obj_clear_flag(s_boot_install_sub_label, LV_OBJ_FLAG_HIDDEN);
    } else {
      if (s_boot_reg_title_label) lv_obj_add_flag(s_boot_reg_title_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_reg_val_label) lv_obj_add_flag(s_boot_reg_val_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_install_title_label) lv_obj_add_flag(s_boot_install_title_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_install_sub_label) lv_obj_add_flag(s_boot_install_sub_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_install_qr) lv_obj_add_flag(s_boot_install_qr, LV_OBJ_FLAG_HIDDEN);
    }
    break;

  case DISPLAY_MODE_GUIDE:
    hide_black_screen_overlay();
    if (s_intro_image) lv_obj_add_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN);

      if (s_guide_sub_mode == GUIDE_SUB_NAVI) {
        ESP_LOGI(TAG, "[DISPLAY] Switching to NAVI (HUD) Screen");
      if (s_road_name_label) {
        lv_obj_set_parent(s_road_name_label, s_hud_screen);
        lv_obj_move_foreground(s_road_name_label);
      }
      align_avr_speed_labels();
      lv_scr_load(s_hud_screen);
    } 
    else if (s_guide_sub_mode == GUIDE_SUB_SPEEDOMETER) {
      ESP_LOGI(TAG, "[DISPLAY] Switching to SPEEDOMETER Screen");
      // 안내모드의 안전운행화면(속도계화면)에서는 도로명을 표기하지 않는다.
      if (s_speedometer_road_name_label) lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
      if (s_speedometer_road_name_sub_label) lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
      
      // 구간속도가 표시 중이면 km/h 단위를 숨김
      bool avr_visible = (s_speedometer_avr_speed_value_label != NULL && !lv_obj_has_flag(s_speedometer_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN));
      if (!avr_visible) {
          if (s_speedometer_unit_label) lv_obj_clear_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
      } else {
          if (s_speedometer_unit_label) lv_obj_add_flag(s_speedometer_unit_label, LV_OBJ_FLAG_HIDDEN);
      }
      lv_scr_load(s_speedometer_screen);
    } 
    else {
      // [STANDBY] 안내 모드 내의 대기 상태 -> 시계와 날짜 표시
      if (lv_scr_act() != s_boot_screen) lv_scr_load(s_boot_screen);
      if (s_intro_image) lv_obj_add_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN);

      if (s_boot_time_label) {
        lv_obj_clear_flag(s_boot_time_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_boot_time_label);
      }
      if (s_boot_date_label) {
        lv_obj_clear_flag(s_boot_date_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(s_boot_date_label);
      }
      // 로고 및 QR 안내는 숨김
      if (s_boot_reg_title_label) lv_obj_add_flag(s_boot_reg_title_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_reg_val_label) lv_obj_add_flag(s_boot_reg_val_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_install_title_label) lv_obj_add_flag(s_boot_install_title_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_install_sub_label) lv_obj_add_flag(s_boot_install_sub_label, LV_OBJ_FLAG_HIDDEN);
      if (s_boot_install_qr) lv_obj_add_flag(s_boot_install_qr, LV_OBJ_FLAG_HIDDEN);
    }
    break;

    case DISPLAY_MODE_CLOCK:
    {
      bool use_clock2 = (s_clock_option == 1);
      if (s_clock_option == 2) {
        // [User Request] 재부팅 시마다 스타일이 바뀔 수 있도록 랜덤 오프셋 적용
        if (s_auto_clock_offset == -1) {
          s_auto_clock_offset = (int)(esp_random() % 2);
        }
        
        time_t now; struct tm t; time(&now); localtime_r(&now, &t);
        // 오프셋을 더해 시간 기반 전환의 기준점을 부팅 시마다 다르게 설정
        use_clock2 = ((t.tm_hour + s_auto_clock_offset) % 2 == 0);
      }
      if (!use_clock2) {
        lv_scr_load(s_clock_screen);
      } else {
        lv_scr_load(s_clock2_screen);
      }
      
      // Load current time immediately to avoid invisible hands on startup
      time_t now; struct tm t; time(&now); localtime_r(&now, &t);
      if (!use_clock2) draw_analog_clock(t.tm_hour, t.tm_min, t.tm_sec);
      else draw_analog_clock2(t.tm_hour, t.tm_min, t.tm_sec);
    }
    align_avr_speed_labels();
    break;

  case DISPLAY_MODE_ALBUM:
    if (s_image_count == 0) scan_intro_images();
    
    if (s_image_count > 0) {
      // 앨범 파일이 있으면 정상 재생
      if (s_current_image_index >= s_image_count) s_current_image_index = 0;
      
      if (s_album_guide_label) lv_obj_add_flag(s_album_guide_label, LV_OBJ_FLAG_HIDDEN);
      load_image_from_sd(0);
    } else {
      // 앨범 파일이 하나도 없으면 guide.jpg 표시
      if (s_album_guide_label) {
        lv_obj_clear_flag(s_album_guide_label, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_album_guide_label, "앱에서 사진을 보내주세요!");
        lv_obj_move_foreground(s_album_guide_label);
      }
      
      // guide.jpg 존재 여부 확인 후 로드 (다양한 경로 체크)
      const char *guide_paths[] = {
          "/littlefs/Photo/guide.jpg", "/littlefs/photo/guide.jpg",
          "/littlefs/flash_data/Photo/guide.jpg", "/littlefs/flash_data/photo/guide.jpg",
          "/littlefs/image/guide.jpg"
      };
      
      bool loaded = false;
      struct stat st;
      for (int i = 0; i < 5; i++) {
        if (stat(guide_paths[i], &st) == 0) {
          char lfs_src[64];
          snprintf(lfs_src, sizeof(lfs_src), "S:%s", guide_paths[i]);
          lv_img_set_src(s_album_img, lfs_src);
          ESP_LOGI(TAG, "Album: Fallback guide loaded from %s", guide_paths[i]);
          loaded = true;
          break;
        }
      }
      if (!loaded) {
        ESP_LOGW(TAG, "Album: guide.jpg NOT FOUND in any expected paths.");
      }
    }

    lv_scr_load_anim(s_album_screen, LV_SCR_LOAD_ANIM_FADE_ON, 50, 0, false);
    break;

  case DISPLAY_MODE_SETTING:
    // [User Request] Always start from SETUP 1 when entering setting mode
    s_setting_page_index = 1;
    if (s_setting_page1_obj) lv_obj_clear_flag(s_setting_page1_obj, LV_OBJ_FLAG_HIDDEN);
    if (s_setting_page2_obj) lv_obj_add_flag(s_setting_page2_obj, LV_OBJ_FLAG_HIDDEN);
    if (s_setting_title_label) lv_label_set_text(s_setting_title_label, "SETUP 1");
    
    update_setting_ui_labels(); // Refresh values
    lv_scr_load(s_setting_screen);
    if (s_intro_image) lv_obj_add_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN);
    break;

  case DISPLAY_MODE_OTA:
    lv_scr_load_anim(s_ota_screen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
    break;
  default: break;
  }

  // --- Centralized Circle Ring Visibility Control (Top Layer) ---
  if (s_circle_ring) {
    if (mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode != GUIDE_SUB_STANDBY) {
      // Guide mode (Speedometer/NAVI) shows ring for GPS status (is_active_state)
      if (is_active_state) lv_obj_clear_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
      else lv_obj_add_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
    } else {
      // Standby, Clock, Album, Setting, etc: ONLY show ring if a safety alert (flashing) is active
      if (s_safety_ring_timer != NULL) lv_obj_clear_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
      else lv_obj_add_flag(s_circle_ring, LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (s_last_safety_request_valid && mode == DISPLAY_MODE_GUIDE) {
    const safety_data_entry_t *entry = find_safety_image_entry(
        s_last_safety_request.start, s_last_safety_request.id,
        s_last_safety_request.commend, s_last_safety_request.data_length,
        s_last_safety_request.data1);
    if (entry) {
      s_current_safety_image_path[0] = '\0';
      update_safety_image_for_data(
          entry, s_last_safety_request.data2, s_last_safety_request.data3,
          s_last_safety_request.data4, s_last_safety_request.data5,
          s_last_safety_request.data6);
    }
  }
}

// OTA 모드 진입 전 불필요한 태스크 정리 (OTA 후 리부팅하므로 안전)
// update_post_handler에서 실제 업데이트 시작 시 호출됨
void prepare_for_ota(void) {
  ESP_LOGI(TAG, "Preparing for OTA: cleaning up states (but keeping tasks for "
                "stability)...");

  // vTaskDelete is dangerous because if a task is killed while holding a mutex
  // (like LVGL_LOCK), it causes a permanent deadlock. Since we have ~100KB RAM,
  // keeping tasks is safer.

  /*
  if (s_ble_tx_task_handle) { vTaskDelete(s_ble_tx_task_handle);
  s_ble_tx_task_handle = NULL; } if (s_virt_drive_task_handle) {
  vTaskDelete(s_virt_drive_task_handle); s_virt_drive_task_handle = NULL; } if
  (s_monitor_task_handle) { vTaskDelete(s_monitor_task_handle);
  s_monitor_task_handle = NULL; } if (s_button_task_handle) {
  vTaskDelete(s_button_task_handle); s_button_task_handle = NULL; } if
  (s_lcd_task_handle) { vTaskDelete(s_lcd_task_handle); s_lcd_task_handle =
  NULL; }
  */

  ESP_LOGI(TAG, "OTA Preparation done. Tasks kept for safety.");
}

// === RF Certification Test Implementation ===


// 모드 전환 함수
static void switch_display_mode(display_mode_t new_mode) {
  if (new_mode >= DISPLAY_MODE_MAX) {
    ESP_LOGW(TAG, "Invalid display mode: %d", new_mode);
    return;
  }

  // [Fix] 부트 모드 재진입 차단: 최초 부팅 이후에는 다시는 로고/QR 화면으로 돌아갈 수 없음
  if (new_mode == DISPLAY_MODE_BOOT && s_current_mode != DISPLAY_MODE_BOOT) {
    ESP_LOGI(TAG, "BOOT mode entry blocked (One-time only). Redirecting to internal STANDBY.");
    new_mode = DISPLAY_MODE_GUIDE;
    s_guide_sub_mode = GUIDE_SUB_STANDBY;
  }

  // == [User Request] Block ALL external switches while in specialized modes ==
  // Only manual (touch) interaction is allowed to exit these modes once entered.
  if (!s_is_manual_mode_switch && s_current_mode != DISPLAY_MODE_BOOT) {
    if (s_current_mode == DISPLAY_MODE_CLOCK || 
        s_current_mode == DISPLAY_MODE_ALBUM ||
        s_current_mode == DISPLAY_MODE_SETTING) {
      if (new_mode != s_current_mode) {
        ESP_LOGI(TAG, "Mode switch blocked: Current mode (%d) locked. Manual interaction required.", s_current_mode);
        return;
      }
    }
  }

  LVGL_LOCK();
  if (s_current_mode == DISPLAY_MODE_OTA && new_mode != DISPLAY_MODE_OTA) {
    stop_wifi_ota_mode();
  }

  s_current_mode = new_mode;
  save_nvs_settings();

  if (new_mode == DISPLAY_MODE_GUIDE) {
    s_last_base_mode = new_mode;
  }

  lv_img_cache_invalidate_src(NULL);
  update_display_mode_ui(new_mode);

  // 새로운 모드가 OTA 모드라면 OTA 모드 시작
  if (new_mode == DISPLAY_MODE_OTA) {
    ESP_LOGI(TAG, "Starting Wi-Fi OTA mode...");
    start_wifi_ota_mode();
  }

  // Force immediate refresh to prevent artifacts
  lv_refr_now(NULL);
  LVGL_UNLOCK();

  const char *mode_names[] = {"BOOT", "GUIDE", "CLOCK", "ALBUM", "SETTING", "OTA"};
  if (new_mode < DISPLAY_MODE_MAX) {
    ESP_LOGI(TAG, "Display mode switched to: %s (%d)", mode_names[new_mode],
             new_mode);
  } else {
    ESP_LOGW(TAG, "Display mode switched to invalid mode: %d", new_mode);
  }
}

// 하드웨어에 직접 밝기 값을 적용하는 내부 함수
static void apply_hw_brightness(uint8_t level) {
  if (s_lcd_io_handle == NULL || level < 1 || level > 5)
    return;
  // 5 is brightest, 1 is darkest
  uint8_t brightness_values[] = {0x00, 0x49, 0x6D, 0x92, 0xB6, 0xFF};
  uint8_t brightness = brightness_values[level];
  uint32_t brightness_cmd = (0x02 << 24) | (0x51 << 8);

  LVGL_LOCK();
  esp_lcd_panel_io_tx_param(s_lcd_io_handle, brightness_cmd, &brightness, 1);
  LVGL_UNLOCK();
}

static void update_auto_brightness(bool force) {
  if (!force && s_brightness_level != 0)
    return; // Not in Auto mode and not forced

  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);

  int month = timeinfo.tm_mon; // 0-11
  int current_min = timeinfo.tm_hour * 60 + timeinfo.tm_min;

  uint16_t sunrise = s_monthly_sun_times[month].sunrise_min;
  uint16_t sunset = s_monthly_sun_times[month].sunset_min;

  uint8_t target_level = 5; // Default: Brightest (Day)
  if (current_min < sunrise) {
    target_level = 1; // Night: Darkest
  } else if (current_min < sunrise + 30) {
    target_level = 3;
  } else if (current_min < sunset - 30) {
    target_level = 5; // Day
  } else if (current_min < sunset) { // Between Sunset-30m and Sunset -> Level 3
    target_level = 3;
  } else if (current_min < sunset + 30) {
    target_level = 3;
  } else {
    target_level = 1; // Night
  }

  static uint8_t s_last_applied_auto = 0;
  ESP_LOGI(SYS_MON_TAG,
             "Internal Free: %u KB, Min Free: %u KB | PSRAM Free: %u KB, Min "
             "Free: %u KB",
             (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned int)
                 (heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned int)(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM) /
                            1024));
  if (force || target_level != s_last_applied_auto) {
    apply_hw_brightness(target_level);
    s_last_applied_auto = target_level;
    ESP_LOGI(TAG,
             "Auto Brightness Applied: Level %d (Month: %d, Time: %02d:%02d)",
             target_level, month + 1, timeinfo.tm_hour, timeinfo.tm_min);
  }
}

// LCD 밝기 제어 함수 (0=Auto, 1=최대, 5=최소)
static void set_lcd_brightness(uint8_t level, bool notify) {
  if (level > 5)
    return;
  s_brightness_level = level;

  if (level == 0) {
    update_auto_brightness(true);
  } else {
    apply_hw_brightness(level);
  }

  // Send brightness update to app for synchronization only if notify is true
  if (notify && s_connected && s_hud_notify_enabled) {
    uint8_t resp[7] = {0x19, 0x4E, 0x0B, 0x02, level, level, 0x2F};
    hud_send_notify_bytes(resp, sizeof(resp));
  }

  // Update Settings UI if it exists
  LVGL_LOCK();
  update_setting_ui_labels();
  LVGL_UNLOCK();

  ESP_LOGI(TAG, "Brightness level set to: %d (%s)", 
           level, 
           level == 0 ? "Auto" : (level == 5 ? "Max" : (level == 1 ? "Min" : "Manual")));
}

// 밝기 레벨을 다음 단계로 순환 (A->1->2->3->4->5->A)
static void switch_to_next_brightness(void) {
  // Ordered sequence: A(0) -> 5 -> 4 -> 3 -> 2 -> 1 -> A(0)
  if (s_brightness_level == 0) s_brightness_level = 5;
  else if (s_brightness_level == 1) s_brightness_level = 0;
  else s_brightness_level--;
  
  set_lcd_brightness(s_brightness_level, true);
}

// Settings UI Click Handlers
// --- Settings UI Safe Touch Logic ---
static uint32_t s_last_setting_swipe_tick = 0;
static uint32_t s_setting_btn_press_tick = 0;

static void handle_safe_btn_event(lv_event_t *e, void (*action)(void)) {
  lv_event_code_t code = lv_event_get_code(e);
  
  if (code == LV_EVENT_PRESSED) {
    s_setting_btn_press_tick = esp_log_timestamp();
  } 
  else if (code == LV_EVENT_RELEASED) {
    uint32_t duration = esp_log_timestamp() - s_setting_btn_press_tick;
    // 0.1초 이상 누르고 있다가 델 때만 동작 (스와이프 오인 방지 및 빠른 반응)
    if (duration >= 100) {
      if (action) action();
    } else {
      ESP_LOGI("TOUCH", "Button press ignored (duration: %d ms < 100ms)", duration);
    }
  }
}

static void do_brightness_up(void) {
  // Sequence: 1->2->3->4->5->A(0) (Stops at A)
  if (s_brightness_level == 5) set_lcd_brightness(0, true);
  else if (s_brightness_level != 0) set_lcd_brightness(s_brightness_level + 1, true);
}
static void brightness_up_event_cb(lv_event_t *e) {
  handle_safe_btn_event(e, do_brightness_up);
}

static void do_brightness_down(void) {
  // Sequence: A(0)->5->4->3->2->1 (Stops at 1)
  if (s_brightness_level == 0) set_lcd_brightness(5, true);
  else if (s_brightness_level > 1) set_lcd_brightness(s_brightness_level - 1, true);
}
static void brightness_down_event_cb(lv_event_t *e) {
  handle_safe_btn_event(e, do_brightness_down);
}

static void do_album_up(void) {
  if (s_album_option > 0) {
    s_album_option = 0; // A(0)로 변경
    update_setting_ui_labels();
  }
}
static void album_up_event_cb(lv_event_t *e) {
  handle_safe_btn_event(e, do_album_up);
}

static void do_album_down(void) {
  if (s_album_option < 1) {
    s_album_option = 1; // M(1)로 변경
    update_setting_ui_labels();
  }
}
static void album_down_event_cb(lv_event_t *e) {
  handle_safe_btn_event(e, do_album_down);
}

static void do_clock_up(void) {
  if (s_clock_option > 0) {
    s_clock_option--;
    update_setting_ui_labels();
  }
}
static void clock_up_event_cb(lv_event_t *e) {
  handle_safe_btn_event(e, do_clock_up);
}

static void do_clock_down(void) {
  if (s_clock_option < 2) {
    s_clock_option++;
    update_setting_ui_labels();
  }
}
static void clock_down_event_cb(lv_event_t *e) {
  handle_safe_btn_event(e, do_clock_down);
}

static void do_save_settings(void) {
  save_nvs_settings();
  set_lcd_brightness(s_brightness_level, true);
  if (s_setting_save_label) {
    lv_label_set_text(s_setting_save_label, "저장됨");
  }
}
static void save_btn_event_cb(lv_event_t *e) {
  handle_safe_btn_event(e, do_save_settings);
}

// Helper to update all setting labels and button states
void update_setting_ui_labels(void) {
  char buf[8];

  // 1. Brightness
  if (s_setting_bright_val_label) {
    if (s_brightness_level == 0)
      lv_label_set_text(s_setting_bright_val_label, "A");
    else {
      snprintf(buf, sizeof(buf), "%d", s_brightness_level);
      lv_label_set_text(s_setting_bright_val_label, buf);
    }

    // UP 버튼: A(0)에 도달하면 비활성화 + 화살표 숨김 + 원형 표시
    if (s_brightness_level == 0) {
      lv_obj_clear_flag(s_setting_btn_up, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_bright_up)
        lv_obj_add_flag(s_setting_line_bright_up, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_bright_up)
        lv_obj_clear_flag(s_setting_circ_bright_up, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(s_setting_btn_up, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_bright_up)
        lv_obj_clear_flag(s_setting_line_bright_up, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_bright_up)
        lv_obj_add_flag(s_setting_circ_bright_up, LV_OBJ_FLAG_HIDDEN);
    }

    // DOWN 버튼: 1단계에 도달하면 비활성화 + 화살표 숨김 + 원형 표시
    if (s_brightness_level == 1) {
      lv_obj_clear_flag(s_setting_btn_down, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_bright_dn)
        lv_obj_add_flag(s_setting_line_bright_dn, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_bright_dn)
        lv_obj_clear_flag(s_setting_circ_bright_dn, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(s_setting_btn_down, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_bright_dn)
        lv_obj_clear_flag(s_setting_line_bright_dn, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_bright_dn)
        lv_obj_add_flag(s_setting_circ_bright_dn, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // 2. Album Option
  if (s_setting_album_val_label) {
    if (s_album_option == 0)
      lv_label_set_text(s_setting_album_val_label, "A");
    else {
      lv_label_set_text(s_setting_album_val_label, "M");
    }

    if (s_album_option == 0) {
      if (s_setting_album_btn_up) lv_obj_clear_flag(s_setting_album_btn_up, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_album_up)
        lv_obj_add_flag(s_setting_line_album_up, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_album_up)
        lv_obj_clear_flag(s_setting_circ_album_up, LV_OBJ_FLAG_HIDDEN);
    } else {
      if (s_setting_album_btn_up) lv_obj_add_flag(s_setting_album_btn_up, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_album_up)
        lv_obj_clear_flag(s_setting_line_album_up, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_album_up)
        lv_obj_add_flag(s_setting_circ_album_up, LV_OBJ_FLAG_HIDDEN);
    }

    if (s_album_option == 1) {
      if (s_setting_album_btn_down) lv_obj_clear_flag(s_setting_album_btn_down, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_album_dn)
        lv_obj_add_flag(s_setting_line_album_dn, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_album_dn)
        lv_obj_clear_flag(s_setting_circ_album_dn, LV_OBJ_FLAG_HIDDEN);
    } else {
      if (s_setting_album_btn_down) lv_obj_add_flag(s_setting_album_btn_down, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_album_dn)
        lv_obj_clear_flag(s_setting_line_album_dn, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_album_dn)
        lv_obj_add_flag(s_setting_circ_album_dn, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // 3. Clock Option
  if (s_setting_clock_val_label) {
    if (s_clock_option == 0)
      lv_label_set_text(s_setting_clock_val_label, "1");
    else if (s_clock_option == 1)
      lv_label_set_text(s_setting_clock_val_label, "2");
    else
      lv_label_set_text(s_setting_clock_val_label, "A");

    if (s_clock_option == 0) {
      lv_obj_add_flag(s_setting_clock_btn_up, LV_OBJ_FLAG_CLICKABLE);
      lv_obj_clear_flag(s_setting_clock_btn_up, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_clock_up)
        lv_obj_add_flag(s_setting_line_clock_up, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_clock_up)
        lv_obj_clear_flag(s_setting_circ_clock_up, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(s_setting_clock_btn_up, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_clock_up)
        lv_obj_clear_flag(s_setting_line_clock_up, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_clock_up)
        lv_obj_add_flag(s_setting_circ_clock_up, LV_OBJ_FLAG_HIDDEN);
    }

    if (s_clock_option == 2) {
      lv_obj_add_flag(s_setting_clock_btn_down, LV_OBJ_FLAG_CLICKABLE);
      lv_obj_clear_flag(s_setting_clock_btn_down, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_clock_dn)
        lv_obj_add_flag(s_setting_line_clock_dn, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_clock_dn)
        lv_obj_clear_flag(s_setting_circ_clock_dn, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(s_setting_clock_btn_down, LV_OBJ_FLAG_CLICKABLE);
      if (s_setting_line_clock_dn)
        lv_obj_clear_flag(s_setting_line_clock_dn, LV_OBJ_FLAG_HIDDEN);
      if (s_setting_circ_clock_dn)
        lv_obj_add_flag(s_setting_circ_clock_dn, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

static void setting_page_cb(lv_event_t *e) {
  (void)e;
  uint32_t now = esp_log_timestamp();
  if (now - s_last_setting_swipe_tick < 500) {
    ESP_LOGI("TOUCH", "Setting swipe throttled");
    return;
  }
  s_last_setting_swipe_tick = now;

  s_setting_page_index = (s_setting_page_index % 2) + 1; // Only 2 pages now
  lv_obj_add_flag(s_setting_page1_obj, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(s_setting_page2_obj, LV_OBJ_FLAG_HIDDEN);

  if (s_setting_page_index == 1) {
    lv_obj_clear_flag(s_setting_page1_obj, LV_OBJ_FLAG_HIDDEN);
    if (s_setting_title_label) lv_label_set_text(s_setting_title_label, "SETUP 1");
  } else {
    lv_obj_clear_flag(s_setting_page2_obj, LV_OBJ_FLAG_HIDDEN);
    if (s_setting_title_label) lv_label_set_text(s_setting_title_label, "SETUP 2");
  }
}

// 다음 모드로 전환
/* static void switch_to_next_mode(void) {
  display_mode_t next_mode =
      (display_mode_t)((s_current_mode + 1) %
DISPLAY_MODE_MAX);
  switch_display_mode(next_mode);
} */

// 버튼 인터럽트 핸들러
static void IRAM_ATTR button_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(s_button_queue, &gpio_num, NULL);
}

// 버튼 태스크 (디바운싱 처리)
static void button_task(void *arg) {
  uint32_t io_num;
  TickType_t last_press_time = 0;
  const TickType_t debounce_delay = pdMS_TO_TICKS(200); // 200ms 디바운싱

  // 하트비트 초기화
  update_heartbeat_button();

  while (1) {
    // 하트비트 업데이트 (큐 대기 중에도
    // 업데이트)
    update_heartbeat_button();

    if (xQueueReceive(s_button_queue, &io_num, portMAX_DELAY)) {
      TickType_t current_time = xTaskGetTickCount(); // 태스크
                                                     // 컨텍스트에서는
                                                     // xTaskGetTickCount
                                                     // 사용

      // 디바운싱: 최근 200ms 이내에 눌림이
      // 있었으면 무시
      if (current_time - last_press_time < debounce_delay) {
        continue;
      }

      last_press_time = current_time;

      // 버튼이 눌렸는지 확인 (LOW = 눌림)
      if (gpio_get_level(io_num) == 0) {
        // 버튼이 눌렸을 때만 모드 전환
        vTaskDelay(pdMS_TO_TICKS(50));     // 짧은 딜레이로 안정화
        if (gpio_get_level(io_num) == 0) { // 여전히 눌려있으면
          ESP_LOGI(TAG, "Boot button pressed, "
                        "switching brightness");
          switch_to_next_brightness();
        }
      }
    }
  }
}

// 버튼 초기화 함수
static esp_err_t init_boot_button(void) {
  // 버튼 큐 생성
  s_button_queue = xQueueCreate(10, sizeof(uint32_t));
  if (s_button_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create button queue");
    return ESP_ERR_NO_MEM;
  }

  // GPIO 설정
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_NEGEDGE, // Falling edge
                                      // (버튼 눌림)
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
      .pull_up_en = GPIO_PULLUP_ENABLE, // 내부 풀업
                                        // 활성화
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure boot "
                  "button GPIO");
    return ret;
  }

  // 인터럽트 핸들러 등록
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG,
             "Failed to install GPIO ISR "
             "service: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ret = gpio_isr_handler_add(BOOT_BUTTON_GPIO, button_isr_handler,
                             (void *)BOOT_BUTTON_GPIO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add GPIO ISR handler: %s", esp_err_to_name(ret));
    return ret;
  }

  // 버튼 태스크 생성 (스택 크기 증가: LVGL
  // 함수 호출 시 많은 스택 필요)
  s_button_task_handle = NULL;
  xTaskCreate(button_task, "button_task", 4096, NULL, 5, &s_button_task_handle);

  ESP_LOGI(TAG, "Boot button initialized (GPIO%d)", BOOT_BUTTON_GPIO);
  return ESP_OK;
}

// Monitor display buffers and queues usage
static void monitor_buffers_and_queues(void) {
  const float WARNING_THRESHOLD = 0.80f; // 80% 사용률 경고

  // 1. 큐 사용률 체크
  if (s_image_update_queue != NULL) {
    UBaseType_t queue_size = uxQueueMessagesWaiting(s_image_update_queue);
    UBaseType_t queue_capacity =
        uxQueueSpacesAvailable(s_image_update_queue) + queue_size;
    if (queue_capacity > 0) {
      float usage = (float)queue_size / (float)queue_capacity;
      if (usage >= WARNING_THRESHOLD) {
        ESP_LOGI(SYS_MON_TAG,
                 "[버퍼/큐 모니터] 이미지 "
                 "업데이트 큐 사용률: %.1f%% "
                 "(%d/%d)",
                 usage * 100.0f, queue_size, queue_capacity);
      }
    }
  }

  if (s_safety_update_queue != NULL) {
    UBaseType_t queue_size = uxQueueMessagesWaiting(s_safety_update_queue);
    UBaseType_t queue_capacity =
        uxQueueSpacesAvailable(s_safety_update_queue) + queue_size;
    if (queue_capacity > 0) {
      float usage = (float)queue_size / (float)queue_capacity;
      if (usage >= WARNING_THRESHOLD) {
        ESP_LOGW(SYS_MON_TAG,
                 "[버퍼/큐 모니터] Safety "
                 "업데이트 큐 사용률: %.1f%% "
                 "(%d/%d) - "
                 "경고!",
                 usage * 100.0f, queue_size, queue_capacity);
      }
    }
  }

  if (s_button_queue != NULL) {
    UBaseType_t queue_size = uxQueueMessagesWaiting(s_button_queue);
    UBaseType_t queue_capacity =
        uxQueueSpacesAvailable(s_button_queue) + queue_size;
    if (queue_capacity > 0) {
      float usage = (float)queue_size / (float)queue_capacity;
      if (usage >= WARNING_THRESHOLD) {
        ESP_LOGW(SYS_MON_TAG,
                 "[버퍼/큐 모니터] 버튼 큐 "
                 "사용률: %.1f%% (%d/%d) - 경고!",
                 usage * 100.0f, queue_size, queue_capacity);
      }
    }
  }

  // 2. LVGL 디스플레이 버퍼 체크 (메모리
  // 사용량 확인)
  if (s_disp_buf1 != NULL && s_disp_buf2 != NULL) {
    size_t buf_size = LCD_H_RES * LVGL_BUFFER_HEIGHT * sizeof(lv_color_t);
    // PSRAM 사용량 확인 (전체 PSRAM 대비)
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);

    if (total_psram > 0) {
      float psram_usage = 1.0f - ((float)free_psram / (float)total_psram);
      if (psram_usage >= WARNING_THRESHOLD) {
        ESP_LOGW(SYS_MON_TAG,
                 "[버퍼/큐 모니터] PSRAM 사용률: "
                 "%.1f%% (여유: %zu/%zu bytes) "
                 "- 경고!",
                 psram_usage * 100.0f, free_psram, total_psram);
      }
    }

    // 버퍼 크기 정보 (디버깅용, 경고는 아님)
    ESP_LOGD(TAG,
             "[버퍼/큐 모니터] LVGL 버퍼 크기: "
             "%zu bytes x 2 = %zu bytes",
             buf_size, buf_size * 2);
  }
}

// System health monitoring - track task
// states and stack usage
static void monitor_system_health(void) {
  ESP_LOGI(SYS_MON_TAG, "=== 시스템 상태 모니터 ===");

  // 1. 주요 태스크의 스택 사용량 체크
  // BTC_TASK: BLE stack task. Size from Kconfig.
  TaskHandle_t btc_task_handle = xTaskGetHandle("BTC_TASK");

  const char *task_names[] = {
      "lvgl_handler", "button_task", "monitor_task", "ble_tx",
      "lcd_task",    "BTC_TASK"};
  TaskHandle_t task_handles[] = {
      s_lvgl_task_handle, s_button_task_handle, s_monitor_task_handle,
      s_ble_tx_task_handle, s_lcd_task_handle,
      btc_task_handle};
  // stack_sizes should match the values used in xTaskCreate
  const UBaseType_t stack_sizes[] = {
      9216, 4096, 4096, 6144, 4096, 4096};

  for (int i = 0; i < 6; i++) {
    if (task_handles[i] != NULL) {
      UBaseType_t stack_high_water =
          uxTaskGetStackHighWaterMark(task_handles[i]);
      eTaskState state = eTaskGetState(task_handles[i]);

      const char *state_str;
      switch (state) {
      case eRunning:
        state_str = "Running";
        break;
      case eReady:
        state_str = "Ready";
        break;
      case eBlocked:
        state_str = "Blocked";
        break;
      case eSuspended:
        state_str = "Suspended";
        break;
      case eDeleted:
        state_str = "Deleted";
        break;
      case eInvalid:
        state_str = "Invalid";
        break;
      default:
        state_str = "Unknown";
        break;
      }

      float stack_usage = 0.0f;
      if (stack_sizes[i] > 0) {
        // High water mark is the minimum free stack EVER observed.
        // So usage = (Total - MinFree) / Total
        stack_usage = ((float)(stack_sizes[i] - stack_high_water) /
                       (float)stack_sizes[i]) *
                      100.0f;
      }

      ESP_LOGI(SYS_MON_TAG, "태스크: %-16s | 상태: %-10s | 스택: %.1f%% (여유: %lu/%lu)",
               task_names[i], state_str, stack_usage,
               (unsigned long)stack_high_water, (unsigned long)stack_sizes[i]);

      // 스택 사용률이 90% 이상이면 경고
      if (stack_usage >= 90.0f) {
        ESP_LOGW(SYS_MON_TAG,
                 "[경고] 태스크 '%s' 스택 사용률 %.1f%% - 오버플로우 위험!",
                 task_names[i], stack_usage);
      }
    }
  }

  // 총 태스크 수 확인
  UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
  ESP_LOGI(SYS_MON_TAG, "총 태스크 수: %d", (int)num_tasks);

  // 2. INTERNAL RAM 상태 체크
  size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t min_free_internal =
      heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  size_t largest_internal_block =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  ESP_LOGI(
      SYS_MON_TAG,
      "INTERNAL RAM: 여유=%zu, 최소여유=%zu, 최대블록=%zu",
      free_internal, min_free_internal, largest_internal_block);

  if (free_internal < 30720) { // 30KB
    ESP_LOGW(SYS_MON_TAG, "[Warning] Internal RAM Low! Free: %zu",
             free_internal);
  }

  // 3. PSRAM 상태
  size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t min_psram = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);

  ESP_LOGI(SYS_MON_TAG, "PSRAM RAM:    여유=%zu, 최소여유=%zu", free_psram,
           min_psram);

  if (free_psram < 524288) { // 512KB (Intro GIF decoding headroom)
    ESP_LOGW(SYS_MON_TAG, "[Warning] PSRAM RAM Low! Free: %zu", free_psram);
  }
}

// Heartbeat tracking for critical tasks
static volatile TickType_t s_last_heartbeat_lvgl = 0;
static volatile TickType_t s_last_heartbeat_ble = 0;
static volatile TickType_t s_last_heartbeat_button = 0;

// Heartbeat update functions (called from
// respective tasks)
void update_heartbeat_lvgl(void) {
  s_last_heartbeat_lvgl = xTaskGetTickCount();
}

void update_heartbeat_ble(void) { s_last_heartbeat_ble = xTaskGetTickCount(); }

void update_heartbeat_button(void) {
  s_last_heartbeat_button = xTaskGetTickCount();
}

// Check if tasks are alive (heartbeat
// monitoring)
static void check_task_heartbeats(void) {
  TickType_t current_tick = xTaskGetTickCount();
  const TickType_t heartbeat_timeout = pdMS_TO_TICKS(40000); // 40초로 연장
                                                             // (SD 부하 고려)

  if (s_last_heartbeat_lvgl > 0) {
    TickType_t elapsed = current_tick - s_last_heartbeat_lvgl;
    if (elapsed > heartbeat_timeout) {
      ESP_LOGE(TAG,
               "[시스템 모니터] 경고: LVGL 태스크 "
               "하트비트 타임아웃! 마지막 "
               "하트비트: %lu ms 전",
               (unsigned long)(elapsed * portTICK_PERIOD_MS));
    }
  }

  if (s_last_heartbeat_ble > 0) {
    TickType_t elapsed = current_tick - s_last_heartbeat_ble;
    if (elapsed > heartbeat_timeout) {
      ESP_LOGE(TAG,
               "[시스템 모니터] 경고: BLE 태스크 "
               "하트비트 타임아웃! 마지막 "
               "하트비트: %lu ms 전",
               (unsigned long)(elapsed * portTICK_PERIOD_MS));
    }
  }

  if (s_last_heartbeat_button > 0) {
    TickType_t elapsed = current_tick - s_last_heartbeat_button;
    if (elapsed > heartbeat_timeout) {
      // 버튼 태스크 하트비트 타임아웃 경고
      // 제거 (사용자 요청) ESP_LOGE(TAG,
      // "[시스템 모니터] 경고: 버튼 태스크
      // 하트비트 타임아웃! 마지막 하트비트:
      // %lu ms 전",
      //          (unsigned long)(elapsed *
      //          portTICK_PERIOD_MS));
    }
  }
}

// Buffer and queue monitoring task
static void buffer_queue_monitor_task(void *arg) {
  (void)arg;
  const TickType_t monitor_interval = pdMS_TO_TICKS(5000); // 5초마다 체크

  ESP_LOGI(SYS_MON_TAG, "버퍼/큐 모니터링 태스크 시작");

  while (1) {
    vTaskDelay(monitor_interval);

    // 하트비트 업데이트
    update_heartbeat_ble();


    // 버퍼/큐 모니터링
    monitor_buffers_and_queues();

    // 하트비트 체크
    check_task_heartbeats();

    // 시스템 상태 체크 (5초마다 실행)
    monitor_system_health();
  }
}

// LVGL handler task (call lv_timer_handler
// periodically)
static void lvgl_handler_task(void *arg) {
  (void)arg;
  image_update_request_t img_req;

  // 하트비트 초기화
  update_heartbeat_lvgl();

  uint32_t boot_start_tick = xTaskGetTickCount();
  bool boot_timeout_triggered = false;

  while (1) {
    // [Fix] 20초 타임아웃 처리: 앱 응답이 없을 때 BOOT 모드 내에서 설치 안내 UI만 표시
    if (!boot_timeout_triggered && !s_app_communicated) {
      if ((xTaskGetTickCount() - boot_start_tick) > pdMS_TO_TICKS(10000)) {
        ESP_LOGI(TAG, "BOOT Timeout: No app communication after 10s. Showing QR/Registration UI.");
        boot_timeout_triggered = true;
        
        // [Action] Show installation UI elements (Manual trigger within current BOOT mode)
        LVGL_LOCK();
        if (s_intro_image) lv_obj_add_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN);
        if (s_boot_reg_title_label) lv_obj_clear_flag(s_boot_reg_title_label, LV_OBJ_FLAG_HIDDEN);
        if (s_boot_reg_val_label) lv_obj_clear_flag(s_boot_reg_val_label, LV_OBJ_FLAG_HIDDEN);
        if (s_boot_install_title_label) lv_obj_clear_flag(s_boot_install_title_label, LV_OBJ_FLAG_HIDDEN);
        if (s_boot_install_sub_label) lv_obj_clear_flag(s_boot_install_sub_label, LV_OBJ_FLAG_HIDDEN);
        if (s_boot_install_qr) lv_obj_clear_flag(s_boot_install_qr, LV_OBJ_FLAG_HIDDEN);
        LVGL_UNLOCK();
      }
    }

    // [Fix] Removed redundant auto-switch logic that was conflicting with NVS mode restoration.
    // The switch to s_nvs_restored_mode is now handled exclusively in process_app_command.

    // 하트비트 업데이트 (루프 시작 시)
    update_heartbeat_lvgl();

    // 1. LVGL Timer Handler (Smallest Lock Window)
    LVGL_LOCK();
    lv_timer_handler();
    LVGL_UNLOCK();

    // Give a tiny breather for other tasks (touch, ble rx)
    vTaskDelay(pdMS_TO_TICKS(1));

    // 2. Safe time display update
    if (s_time_display_update_required) {
      s_time_display_update_required = false;
      update_heartbeat_lvgl();
      LVGL_LOCK();
      update_time_display();
      LVGL_UNLOCK();
      update_heartbeat_lvgl();
    }

    // Check for image update requests from
    // other tasks (TBT) Process multiple
    // items per loop to prevent queue from
    // filling up
    if (s_image_update_queue != NULL) {
      int processed = 0;
      const int max_per_loop = 20;
      ESP_LOGD(TAG, "LVGL: Processing image update queue... (count=%d)",
               (int)uxQueueMessagesWaiting(s_image_update_queue));
      while (processed < max_per_loop &&
             xQueueReceive(s_image_update_queue, &img_req, 0) == pdTRUE) {

        LVGL_LOCK();
        const image_data_entry_t *entry = find_tbt_image_entry(
            img_req.start, img_req.id, img_req.commend, img_req.data_length,
            img_req.data1, img_req.data2);
        if (entry != NULL) {
          update_hud_image_for_data(entry, img_req.data2, img_req.force_update);
          if (img_req.has_distance) {
            update_tbt_distance_labels(img_req.data3, img_req.data4,
                                       img_req.data5);
          }
        }
        LVGL_UNLOCK();

        processed++;
        if (processed % 5 == 0) {
          update_heartbeat_lvgl();
          vTaskDelay(pdMS_TO_TICKS(1));
        }
      }
      ESP_LOGD(TAG, "LVGL: Image update queue processed");
    }

    // Check for road name update requests
    if (s_road_name_update_queue != NULL) {
      road_name_update_request_t road_req;
      if (xQueueReceive(s_road_name_update_queue, &road_req, 0) == pdTRUE) {
        update_heartbeat_lvgl();
        LVGL_LOCK();
        update_road_name_label(road_req.road_name);
        LVGL_UNLOCK();
        update_heartbeat_lvgl();
      }
    }

    // Check for Safety_DRV update requests
    // from other tasks Process only one item
    // per loop to prevent blocking during
    // image decoding 이미지 디코딩이 오래
    // 걸릴 수 있으므로 한 번에 하나씩만 처리
    if (s_safety_update_queue != NULL &&
        uxQueueMessagesWaiting(s_safety_update_queue) > 0) {
      ESP_LOGD(TAG, "LVGL: Processing safety update queue...");
      safety_update_request_t safety_req;
      if (xQueueReceive(s_safety_update_queue, &safety_req, 0) == pdTRUE) {
        const safety_data_entry_t *entry = find_safety_image_entry(
            safety_req.start, safety_req.id, safety_req.commend,
            safety_req.data_length, safety_req.data1);
        if (entry != NULL) {
          update_heartbeat_lvgl();
          LVGL_LOCK();
          update_safety_image_for_data(entry, safety_req.data2,
                                       safety_req.data3, safety_req.data4,
                                       safety_req.data5, safety_req.data6);
          LVGL_UNLOCK();
          update_heartbeat_lvgl();
        }
      }
      ESP_LOGD(TAG, "LVGL: Safety update queue processed");
    }

    // Check for speed update requests from
    // other tasks
    if (s_speed_update_queue != NULL) {
      speed_update_request_t speed_req;
      int speed_processed = 0;
      while (speed_processed < 5 &&
             xQueueReceive(s_speed_update_queue, &speed_req, 0) == pdTRUE) {
        update_heartbeat_lvgl();
        LVGL_LOCK();
        update_speed_label(speed_req.data1, speed_req.data2);
        LVGL_UNLOCK();
        update_heartbeat_lvgl();
        speed_processed++;
      }
    }

    // Check for destination update requests
    // from other tasks
    if (s_destination_update_queue != NULL &&
        uxQueueMessagesWaiting(s_destination_update_queue) > 0) {
      ESP_LOGD(TAG, "LVGL: Processing destination update queue...");
      destination_update_request_t dest_req;
      int dest_processed = 0;
      while (dest_processed < 3 && xQueueReceive(s_destination_update_queue,
                                                 &dest_req, 0) == pdTRUE) {
        update_heartbeat_lvgl();
        LVGL_LOCK();
        update_destination_info(dest_req.data1, dest_req.data2, dest_req.data3,
                                dest_req.data4, dest_req.data5);
        LVGL_UNLOCK();
        update_heartbeat_lvgl();
        dest_processed++;
      }
      ESP_LOGD(TAG, "LVGL: Destination update queue processed");
    }



    // Check for circle update requests from
    // other tasks
    if (s_circle_update_queue != NULL) {
      circle_update_request_t circle_req;
      if (xQueueReceive(s_circle_update_queue, &circle_req, 0) == pdTRUE) {
        update_heartbeat_lvgl();
        LVGL_LOCK();
        update_circle_display(circle_req.start, circle_req.id,
                              circle_req.commend, circle_req.data_length,
                              circle_req.data1);
        LVGL_UNLOCK();
        update_heartbeat_lvgl();
      }
    }

    // Check for clear display requests from
    // other tasks
    if (s_clear_display_queue != NULL) {
      clear_display_request_t clear_req;
      if (xQueueReceive(s_clear_display_queue, &clear_req, 0) == pdTRUE) {
        update_heartbeat_lvgl();
        LVGL_LOCK();
        update_clear_display(clear_req.data1);
        LVGL_UNLOCK();
        update_heartbeat_lvgl();
      }
    }

    // Update RSSI label
    if (s_rssi_label != NULL) {
      LVGL_LOCK();
      lv_obj_add_flag(s_rssi_label, LV_OBJ_FLAG_HIDDEN);
      LVGL_UNLOCK();
    }

    // 하트비트 업데이트 (루프 끝)
    update_heartbeat_lvgl();

    vTaskDelay(pdMS_TO_TICKS(5)); // LVGL recommends ~5ms delay for
                                  // responsive display updates
  }
}

// Convert RGB565 to RGB888 (AI_DRV reference:
// lv_color_make uses RGB888)
/* static lv_color_t
rgb565_to_lv_color(uint16_t rgb565) {
  // RGB565: RRRRR GGGGGG BBBBB
  // Extract and expand to 8-bit
  uint8_t r = ((rgb565 >> 11) & 0x1F) << 3; //
5 bits -> 8 bits (scale by 8) uint8_t g =
((rgb565 >> 5) & 0x3F) << 2;  // 6 bits -> 8
bits (scale by 4) uint8_t b = (rgb565 & 0x1F)
<< 3;         // 5 bits -> 8 bits (scale by 8)

  // Improve scaling for better color accuracy
  r |= (r >> 5); // Add LSB for better
approximation g |= (g >> 6); // Add LSB for
better approximation b |= (b >> 5); // Add LSB
for better approximation

  return lv_color_make(r, g, b);
} */

static void lcd_task(void *arg) {
  (void)arg;
  // LCD task - no longer needed for
  // clock/ring display
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Deleted legacy Bluedroid helper


// ---------------------------------------------------------------------------
// Utility Functions for BLE and UI Updates
// ---------------------------------------------------------------------------

// static uint32_t s_last_ble_activity_tick = 0; // Removed: duplicate of line 5056

void note_ble_activity(void) {
    s_last_ble_activity_tick = xTaskGetTickCount();
}

// void log_ble_packet(...) removed: duplicate of line 5064

void hud_send_notify_bytes(const uint8_t *data, uint16_t len) {
    if (!s_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_hud_notify_enabled) return;
    
    // [User Request] Standardized PKT_LOG for all TX packets
    if (len >= 5 && data[0] == 0x19) {
        uint8_t id = data[1];
        uint8_t cmd = data[2];
        const char *desc = "기타";
        bool silent_data = false;

        if (id == 0x4F) {
            switch(cmd) {
                case 0x01: desc = "F/W 업데이트 정보(INFO)"; break;
                case 0x02: desc = "F/W 업데이트 요청(REQ)"; break;
                case 0x03: desc = "F/W 업데이트 데이터(DATA)"; silent_data = true; break;
                case 0x04: desc = "F/W 업데이트 데이터 응답(ACK)"; break;
                case 0x05: desc = "F/W 업데이트 완료 통보(COMPLETE)"; break;
                case 0x06: desc = "F/W 업데이트 취소(CANCEL)"; break;
                default: desc = "F/W 업데이트 관련"; break;
            }
        } else if (id == 0x50) {
            switch(cmd) {
                case 0x11: desc = "이미지 전송 정보 응답(INFO_RES)"; break;
                case 0x03: desc = "이미지 데이터 응답(ACK)"; break;
                case 0x04: desc = "이미지 전송 완료 통보(COMPLETE)"; break;
                case 0x07: desc = "이미지 상태 요청(TX)"; break;
                default: desc = "이미지 전송 관련"; break;
            }
        } else if (id == 0x00 && cmd == 0x00) {
            silent_data = true; // FW Keep-alive (Null packet)
        } else {
            switch(cmd) {
                case 0x0D: desc = "시간 업데이트 요청"; break;
                case 0x11: desc = "현재 안내 모드 요청"; break;
                case 0x12: desc = "GPS 상태 요청"; break;
                case 0x0B: desc = "밝기 설정값 통보"; break;
                case 0x0C: desc = "모델명/버전 통보"; break;
                case 0x04: desc = "하트비트/링 상태"; break;
                case 0x02: desc = "배터리 레벨 통보"; break;
                case 0x50: desc = "이미지 데이터"; silent_data = true; break;
            }
        }

        if (!silent_data) {
            char hex_buf[128] = {0};
            int pos = 0;
            size_t show_len = (len > 11) ? 11 : len; 
            
            for (int i = 1; i < len && i < show_len; i++) {
                pos += snprintf(hex_buf + pos, sizeof(hex_buf) - pos, "%02X ", data[i]);
            }
            if (len > 11) strcat(hex_buf, "... ");

            ESP_LOGW(TAG, "PKT_LOG [TX] %s// %s", hex_buf, desc);
        }
    }
    
    
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om) {
        int rc = ble_gattc_notify_custom(s_conn_handle, s_hud_char_ready_handle, om);
        if (rc == 0) {
            note_ble_activity();
            if (len == 10 && data[1] == PROTOCOL_ID) s_tx_ok++;
        } else {
            if (len == 10 && data[1] == PROTOCOL_ID) s_tx_fail++;
        }
    }
}

// update_ui_progress is already defined around line 8764 now. 
// We will ensure it stays there and remove duplicate here.
// ---------------------------------------------------------------------------
// NimBLE GATT Service Definitions (HID, HUD, Battery, DIS)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// HID Report Map (Minimal Keyboard/Consumer Control)
// ---------------------------------------------------------------------------
static const uint8_t s_hid_report_map[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved)
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

static int
hid_on_access(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid16 == 0x2A4B) { // Report Map
        os_mbuf_append(ctxt->om, s_hid_report_map, sizeof(s_hid_report_map));
        return 0;
    } else if (uuid16 == 0x2A4A) { // HID Info
        uint8_t info[4] = {0x01, 0x01, 0x00, 0x03}; // bcdHID=1.1, country=0, flags=RemoteWake|NormallyConnectable
        os_mbuf_append(ctxt->om, info, sizeof(info));
        return 0;
    } else if (uuid16 == 0x2A4E) { // Protocol Mode
        uint8_t mode = 0x01; // Report Mode
        os_mbuf_append(ctxt->om, &mode, 1);
        return 0;
    } else if (uuid16 == 0x2A4D) { // Report
        os_mbuf_append(ctxt->om, s_hid_in_report, 8);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int
dis_on_access(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    if (uuid16 == 0x2A29) {
        os_mbuf_append(ctxt->om, "MOVISION", 8);
        return 0;
    } else if (uuid16 == 0x2A24) {
        os_mbuf_append(ctxt->om, "HUD1-PRO", 8);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int
hud_on_access(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid16 == 0xFFF1 || uuid16 == 0xFFF2) { // HUD Command Channel (Primary or Secondary)
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            // [Safety Removed] User requested to keep virtual drive running until power off.
            // App packets will still be processed by process_app_command in background,
            // but simulation will continue to inject data.
            struct os_mbuf *om = ctxt->om;
            while (om != NULL) {
                const uint8_t *data = om->om_data;
                uint16_t len = om->om_len;
                
                for (uint16_t i = 0; i < len; i++) {
                    uint8_t byte = data[i];
                    
                    // New packet starts with 0x19 (only if idle or very start of fragment)
                    if (byte == 0x19 && s_rx_expected_len == 0) {
                        s_rx_pos = 0;
                        s_rx_expected_len = 0;
                    }
                    
                    if (s_rx_pos < sizeof(s_rx_buffer)) {
                        s_rx_buffer[s_rx_pos++] = byte;
                        
                        // Determine expected length based on ID
                        if (s_rx_expected_len == 0) {
                            if (s_rx_pos == 4 && s_rx_buffer[0] == 0x19) {
                                uint8_t id = s_rx_buffer[1];
                                if (id == 0x4D) { // Standard format
                                    s_rx_expected_len = s_rx_buffer[3] + 5;
                                }
                            } else if (s_rx_pos == 5 && s_rx_buffer[0] == 0x19) {
                                uint8_t id = s_rx_buffer[1];
                                if (id == 0x4F || id == 0x50) { // Large data format (FW/IMG)
                                    uint16_t dlen = (s_rx_buffer[3] << 8) | s_rx_buffer[4];
                                    s_rx_expected_len = dlen + 6;
                                }
                            }
                        }
                        
                        // If we have a full packet, process it
                        if (s_rx_expected_len > 0 && s_rx_pos == s_rx_expected_len) {
                            s_app_communicated = true; // Mark as communicated on first valid packet
                            if (s_rx_buffer[s_rx_pos - 1] == 0x2F) {
                                // [Optimization] Fast-track for heavy data (ID 0x50, 0x4F)
                                // [User Request] Ensure logging even in fast-track path
                                if (s_rx_buffer[1] == 0x50 || s_rx_buffer[1] == 0x4F) {
                                    log_ble_packet(s_rx_buffer, s_rx_pos, "RX");
                                    LVGL_LOCK();
                                    process_app_command(s_rx_buffer, s_rx_pos);
                                    LVGL_UNLOCK();
                                } else {
                                    log_ble_packet(s_rx_buffer, s_rx_pos, "RX");
                                    if (s_ble_cmd_queue) {
                                        ble_cmd_t cmd;
                                        cmd.len = (s_rx_pos > 256) ? 256 : s_rx_pos;
                                        memcpy(cmd.data, s_rx_buffer, cmd.len);
                                        xQueueSend(s_ble_cmd_queue, &cmd, 0);
                                    } else {
                                        process_app_command(s_rx_buffer, s_rx_pos);
                                    }
                                }
                            } else {
                                ESP_LOGW(TAG, "Full packet received but tail is not 0x2F (0x%02X)", s_rx_buffer[s_rx_pos-1]);
                            }
                            s_rx_pos = 0;
                            s_rx_expected_len = 0;
                        }
                    } else {
                        ESP_LOGE(TAG, "RX buffer overflow!");
                        s_rx_pos = 0;
                        s_rx_expected_len = 0;
                    }
                }
                om = SLIST_NEXT(om, om_next);
            }
            return 0;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint8_t dummy[1] = {0};
            os_mbuf_append(ctxt->om, dummy, 1);
            return 0;
        }
    } else if (uuid16 == 0x2A19) { // Battery Level
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            os_mbuf_append(ctxt->om, &s_batt_level, 1);
            return 0;
        }
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// OTA Service UUIDs (128-bit)
static const ble_uuid128_t s_ota_svc_uuid = 
    BLE_UUID128_INIT(0x80, 0x32, 0x36, 0x36, 0x35, 0x37, 0x41, 0x0b, 0xab, 0x8c, 0xcc, 0x07, 0xfc, 0x96, 0x73, 0xce);
static const ble_uuid128_t s_ota_ctrl_uuid = 
    BLE_UUID128_INIT(0x80, 0x32, 0x36, 0x56, 0x35, 0x37, 0x41, 0x0b, 0xab, 0x8c, 0xcc, 0x07, 0xfc, 0x96, 0x73, 0xce);
static const ble_uuid128_t s_ota_data_uuid = 
    BLE_UUID128_INIT(0x80, 0x32, 0x36, 0x57, 0x35, 0x37, 0x41, 0x0b, 0xab, 0x8c, 0xcc, 0x07, 0xfc, 0x96, 0x73, 0xce);

extern uint16_t s_ota_ctrl_handle;
extern uint16_t s_ota_data_handle;
extern int ota_on_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def s_nimble_svc_defs[] = {
    {
        // 1. HUD Service (FFEA)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(HUD_SERVICE_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {{
            .uuid = BLE_UUID16_DECLARE(0xFFF1), // Primary Data (Write/Notify)
            .access_cb = hud_on_access,
            .val_handle = &s_hud_char_ready_handle,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
        }, {
            .uuid = BLE_UUID16_DECLARE(0xFFF2), // Secondary Command Channel (App Compatibility)
            .access_cb = hud_on_access,
            .val_handle = &s_hud_char_write_handle,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        }, {
            0,
        }},
    },
    {
        // 2. Battery Service (180F)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F),
        .characteristics = (struct ble_gatt_chr_def[]) {{
            .uuid = BLE_UUID16_DECLARE(0x2A19),
            .access_cb = hud_on_access,
            .val_handle = &s_batt_level_handle,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        }, {
            0,
        }},
    },
    {
        // 3. OTA Service (Integrated)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_ota_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {{
            .uuid = &s_ota_ctrl_uuid.u,
            .access_cb = ota_on_access,
            .val_handle = &s_ota_ctrl_handle,
            .flags = BLE_GATT_CHR_F_WRITE,
        }, {
            .uuid = &s_ota_data_uuid.u,
            .access_cb = ota_on_access,
            .val_handle = &s_ota_data_handle,
            .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        }, {
            0,
        }},
    },
    {
        // 4. DIS Service (0x180A)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),
        .characteristics = (struct ble_gatt_chr_def[]) {{
            .uuid = BLE_UUID16_DECLARE(0x2A29), // Manufacturer
            .access_cb = dis_on_access,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            .uuid = BLE_UUID16_DECLARE(0x2A24), // Model
            .access_cb = dis_on_access,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            0,
        }},
    },
    {
        // 5. HID Service (0x1812)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1812),
        .characteristics = (struct ble_gatt_chr_def[]) {{
            .uuid = BLE_UUID16_DECLARE(0x2A4B), // Report Map
            .access_cb = hid_on_access,
            .flags = BLE_GATT_CHR_F_READ_ENC,
        }, {
            .uuid = BLE_UUID16_DECLARE(0x2A4A), // HID Info
            .access_cb = hid_on_access,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            .uuid = BLE_UUID16_DECLARE(0x2A4E), // Protocol Mode
            .access_cb = hid_on_access,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
        }, {
            .uuid = BLE_UUID16_DECLARE(0x2A4D), // Report
            .access_cb = hid_on_access,
            .val_handle = &s_hid_char_rpt_in_handle,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .descriptors = (struct ble_gatt_dsc_def[]) {{
                .uuid = BLE_UUID16_DECLARE(0x2908), // Report Reference
                .access_cb = hid_on_access, // Not implemented but needed for some OS
                .att_flags = BLE_ATT_F_READ,
            }, {
                0,
            }},
        }, {
            0,
        }},
    },
    {
        0, // No more services
    },
};

// ---------------------------------------------------------------------------
// NimBLE GAP Event Handler
// ---------------------------------------------------------------------------

static int
ble_mp_gap_event(struct ble_gap_event *event, void *arg) {
    int rc;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        {
            struct ble_gap_conn_desc desc;
            ble_gap_conn_find(event->connect.conn_handle, &desc);
            char addr_str[18];
            snprintf(addr_str, sizeof(addr_str), "%02x:%02x:%02x:%02x:%02x:%02x",
                     desc.peer_id_addr.val[5], desc.peer_id_addr.val[4], desc.peer_id_addr.val[3],
                     desc.peer_id_addr.val[2], desc.peer_id_addr.val[1], desc.peer_id_addr.val[0]);
            
            ESP_LOGI(TAG, "Connected: status=%d handle=%d peer=%s", 
                     event->connect.status, event->connect.conn_handle, addr_str);
        }
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            s_need_fast_conn = true;
        } else {
            ble_mp_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected: reason=0x%02x", event->disconnect.reason);
        s_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_hud_notify_enabled = false;

        // Fallback to Guidance Standby on disconnect
        s_guide_sub_mode = GUIDE_SUB_STANDBY;
        switch_display_mode(DISPLAY_MODE_GUIDE);

        ble_mp_advertise();
        return 0;

    case BLE_GAP_EVENT_MTU:
        s_mtu = event->mtu.value;
        ESP_LOGI(TAG, "MTU updated: %d", s_mtu);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_hud_char_ready_handle) {
            s_hud_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "HUD CCCD updated: %d", s_hud_notify_enabled);
        } else if (event->subscribe.attr_handle == s_hid_char_rpt_in_handle) {
            s_hid_cccd = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "HID CCCD updated: %d", s_hid_cccd);
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* Android 16 Re-pairing Fix: If the phone asks to pair again, its 
         * bond info is likely stale or invalid. Delete our side so we can 
         * genuinely re-sync and stay bonded permanently. */
        struct ble_gap_conn_desc repeat_desc;
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &repeat_desc);
        if (rc == 0) {
            ble_store_util_delete_peer(&repeat_desc.peer_id_addr);
            ESP_LOGW(TAG, "Stale bond detected. Deleted old peer info for re-sync.");
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "Encryption changed: status=%d (%s)", 
                 event->enc_change.status,
                 (event->enc_change.status == 13) ? "HCI_ERR_PIN_OR_KEY_MISSING - HUD forgot the phone!" : "OK");
        
        // Check bond count after encryption success
        if (event->enc_change.status == 0) {
            int count_p = 0;
            ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &count_p);
            ESP_LOGI(TAG, "Security established. Total bonded peers now: %d", count_p);
        }
        return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGI(TAG, "Passkey Action: type=%d", event->passkey.params.action);
        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            struct ble_sm_io pk;
            pk.action = event->passkey.params.action;
            pk.passkey = 123456; // Default or random
            ESP_LOGI(TAG, "Display Passkey: %06ld", pk.passkey);
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pk);
            if (rc != 0) ESP_LOGE(TAG, "Error injecting passkey; rc=%d", rc);
        }
        return 0;
    }
    return 0;
}

static void ble_mp_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    // 1. Primary Advertising Data - HUD focused
    memset(&fields, 0, sizeof fields);
    
    // Flags and Name
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    
    // CRITICAL: HUD Service UUID must be in the main packet for many Apps
    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(HUD_SERVICE_UUID16)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    // Appearance (Generic HID icon - avoids "Install Keyboard App" nag)
    fields.appearance = 0x03C0; 
    fields.appearance_is_present = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields; rc=%d", rc);
        return;
    }

    // 2. Scan Response Data - HID Service ID for discovery
    memset(&fields, 0, sizeof fields);
    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(0x1812) 
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan rsp fields; rc=%d", rc);
        return;
    }

    // 3. Start Advertising with aggressive intervals (for faster App pickup)
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 32; // 20ms
    adv_params.itvl_max = 48; // 30ms

    rc = ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, ble_mp_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Error starting advertisement; rc=%d", rc);
    } else if (rc == 0) {
        ESP_LOGI(TAG, "Advertising started (HUD prioritized)");
    }
}

void hud_request_fast_conn(uint16_t conn_handle) {
    struct ble_gap_upd_params params = {
        .itvl_min = 0x0018, // 30ms
        .itvl_max = 0x0028, // 50ms
        .latency = 2,
        .supervision_timeout = 600, // 6s
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    ble_gap_update_params(conn_handle, &params);
}

static void ble_tx_task(void *pvParameters) {
    while (1) {
        if (s_connected && s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            // Connection management
            if (s_need_fast_conn) {
                s_need_fast_conn = false;
                hud_request_fast_conn(s_conn_handle);
            }

        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------------------------------------
// NimBLE Host Setup and Initialization
// ---------------------------------------------------------------------------

static void ble_mp_on_reset(int reason) {
    ESP_LOGI(TAG, "NimBLE Reset: reason=%d", reason);
}

static void ble_mp_on_sync(void) {
    int rc;

    // Suggest 512 MTU to peers
    ble_att_set_preferred_mtu(512);

    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) return;

    ble_mp_advertise();
    
    // [Fix] Check existing bond count to verify NVS persistence
    int count_o = 0, count_p = 0;
    ble_store_util_count(BLE_STORE_OBJ_TYPE_OUR_SEC, &count_o);
    ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &count_p);
    ESP_LOGI(TAG, "NimBLE Sync: Bonds found in NVS: Our=%d, Peer=%d", count_o, count_p);
    
    ESP_LOGI(TAG, "NimBLE Sync Complete. Advertising started.");
}

static void ble_mp_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t init_ble(void) {
    esp_err_t ret;

    // 1. Configure Host Global Settings BEFORE Init
    ble_hs_cfg.sync_cb = ble_mp_on_sync;
    ble_hs_cfg.reset_cb = ble_mp_on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    
    // [Fix] Stable Identity + Modern Security (Android 16 Stability)
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;      
    ble_hs_cfg.sm_mitm = 0;         // Just Works is more stable for HID without display
    ble_hs_cfg.sm_sc = 1;           // Maintain Secure Connections for bond persistence
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    // [New] Enforce Identity Stability: Using Public Address by default
    // We ensure identity persists via SM_PAIR_KEY_DIST_ID below.

    // [Staleness Check] Ensure NimBLE is actually configured for persistence
    // [Fix] Cleanup legacy Bluedroid namespace if it exists to avoid conflicts
    nvs_handle_t h_clean;
    if (nvs_open("bt_config.conf", NVS_READWRITE, &h_clean) == ESP_OK) {
        nvs_erase_all(h_clean);
        nvs_commit(h_clean);
        nvs_close(h_clean);
        ESP_LOGW(TAG, "Legacy Bluedroid NVS namespace cleaned.");
    }

    // 3. Storage Init (Bonding)
    ble_store_config_init();

    // 2. Core NimBLE Port Init
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble port: %d", ret);
        return ret;
    }

    // [Fix] Log Static Identity based on MAC
    uint8_t bt_addr[6];
    esp_read_mac(bt_addr, ESP_MAC_BT);
    ESP_LOGI(TAG, "Bluetooth Identity (MAC): %02X:%02X:%02X:%02X:%02X:%02X",
             bt_addr[0], bt_addr[1], bt_addr[2], bt_addr[3], bt_addr[4], bt_addr[5]);

    // 4. Initialize Built-in Services

    // 4. Initialize Built-in Services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // 5. Unified User Service Registration
    int rc;
    rc = ble_gatts_count_cfg(s_nimble_svc_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT; rc=%d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_nimble_svc_defs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add services; rc=%d", rc);
        return ESP_FAIL;
    }

    // 6. Start Host Task
    nimble_port_freertos_init(ble_mp_host_task);
    return ESP_OK;
}

// ble_mp_host_task duplicate removed (original at line 7487)

// LittleFS mount - based on ESP-IDF v5.5.1
// example
static esp_err_t init_littlefs(void) {
  ESP_LOGI(TAG, "Initializing LittleFS");

  esp_vfs_littlefs_conf_t conf = {
      .base_path = "/littlefs",
      .partition_label = "storage",
      .format_if_mount_failed = true,
      .dont_mount = false,
  };

  // Use settings defined above to initialize
  // and mount LittleFS filesystem.
  esp_err_t ret = esp_vfs_littlefs_register(&conf);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Failed to mount or "
                    "format filesystem");
    } else if (ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGE(TAG, "Failed to find LittleFS "
                    "partition");
    } else {
      ESP_LOGE(TAG,
               "Failed to initialize "
               "LittleFS (%s)",
               esp_err_to_name(ret));
    }
    s_littlefs_mounted = false;
    return ret;
  }

  size_t total = 0, used = 0;
  ret = esp_littlefs_info(conf.partition_label, &total, &used);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to get LittleFS "
             "partition information (%s)",
             esp_err_to_name(ret));
    s_littlefs_mounted = false;
    return ret;
  }

  ESP_LOGI(TAG, "LittleFS mounted at %s", conf.base_path);
  ESP_LOGI(TAG,
           "Partition size: total: %d bytes, "
           "used: %d bytes",
           total, used);
  s_littlefs_mounted = true;

  return ESP_OK;
}



// ---------------------------------------------------------------------------
// New Mode UIs and Touch Driver
// ---------------------------------------------------------------------------

// Helper declarations
static void draw_analog_clock(int hour, int minute, int second);

// Clock 1 Objects (Pill Style Redesign)
static lv_obj_t *s_clock1_hour_bg;
static lv_obj_t *s_clock1_hour_fg;
static lv_obj_t *s_clock1_hour_shadow;
static lv_obj_t *s_clock1_minute_bg;
static lv_obj_t *s_clock1_minute_fg;
static lv_obj_t *s_clock1_minute_shadow;
static lv_point_t s_clock1_hour_points[2];
static lv_point_t s_clock1_hour_fg_points[2];
static lv_point_t s_clock1_hour_shadow_points[2];
static lv_point_t s_clock1_minute_points[2];
static lv_point_t s_clock1_minute_fg_points[2];
static lv_point_t s_clock1_minute_shadow_points[2];
static lv_obj_t *s_clock_bg_img = NULL;
static lv_obj_t *s_clock_center_dot = NULL;

// Clock 2 Objects (Reverted to Gold Stick Style)
static lv_obj_t *s_clock2_hour_line;
static lv_obj_t *s_clock2_minute_line;
static lv_obj_t *s_clock2_second_line;
static lv_point_t s_clock2_hour_points[2];
static lv_point_t s_clock2_minute_points[2];
static lv_point_t s_clock2_second_points[2];

static lv_timer_t *s_clock_timer = NULL;
// static lv_obj_t *s_clock_wday_label = NULL; // Day of Week (SUN, MON...)
// static lv_obj_t *s_clock_day_label = NULL;  // Day of Month (01, 11...)

// Boot Mode 전용 디지털 시계 - Moved to global section at top

static void clock_timer_cb(lv_timer_t *timer) {
  if (s_current_mode != DISPLAY_MODE_CLOCK &&
      s_current_mode != DISPLAY_MODE_BOOT &&
      s_current_mode != DISPLAY_MODE_GUIDE)
    return;

  // Handle Boot Registration display logic
  // [Fix] iOS 자동 재연결 대응: 앱 명령 수신 전까지 설치 안내 타이머 유지
  // [Fix] Increase timeout to 60s and suppress if connected
  if (s_current_mode == DISPLAY_MODE_BOOT && !s_hud_seen_first_cmd && !s_connected) {
    if (!s_boot_reg_shown) {
      s_boot_reg_timer_sec++;
      if (s_boot_reg_timer_sec >= 60) {
        ESP_LOGI(TAG, "[DISPLAY] Step 5: 60s Timeout - Displaying Registration/QR UI");
        if (s_intro_image) lv_obj_add_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN);
        
        if (s_boot_reg_title_label) {
          lv_obj_clear_flag(s_boot_reg_title_label, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(s_boot_reg_title_label);
        }
        if (s_boot_reg_val_label) {
          lv_obj_clear_flag(s_boot_reg_val_label, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(s_boot_reg_val_label);
        }
        if (s_boot_install_title_label) {
          lv_obj_clear_flag(s_boot_install_title_label, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(s_boot_install_title_label);
        }
        if (s_boot_install_sub_label) {
          lv_obj_clear_flag(s_boot_install_sub_label, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(s_boot_install_sub_label);
        }
        if (s_boot_install_qr) {
          lv_obj_clear_flag(s_boot_install_qr, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(s_boot_install_qr);
        }
        // [제거] 부팅 모드에서는 애초에 라벨이 뜨지 않으므로 수동으로 숨길 필요 없음
        s_boot_reg_shown = true;
      }
    }
  } else {
    s_boot_reg_timer_sec = 0;
    if (s_boot_reg_shown) {
       if (s_boot_reg_title_label) lv_obj_add_flag(s_boot_reg_title_label, LV_OBJ_FLAG_HIDDEN);
       if (s_boot_reg_val_label) lv_obj_add_flag(s_boot_reg_val_label, LV_OBJ_FLAG_HIDDEN);
       if (s_boot_install_title_label) lv_obj_add_flag(s_boot_install_title_label, LV_OBJ_FLAG_HIDDEN);
       if (s_boot_install_sub_label) lv_obj_add_flag(s_boot_install_sub_label, LV_OBJ_FLAG_HIDDEN);
       if (s_boot_install_qr) lv_obj_add_flag(s_boot_install_qr, LV_OBJ_FLAG_HIDDEN);
       s_boot_reg_shown = false;
    }
  }

  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);

  int day = timeinfo.tm_mday;
  int hour = timeinfo.tm_hour;
  int minute = timeinfo.tm_min;
  int second = timeinfo.tm_sec;
  int wday = timeinfo.tm_wday;
  const char *week_days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

  if (s_current_mode == DISPLAY_MODE_GUIDE && s_guide_sub_mode == GUIDE_SUB_STANDBY) {
    if (s_boot_time_label && s_font_orb_100) {
      lv_label_set_text_fmt(s_boot_time_label, "%02d:%02d", hour, minute);
    }
    if (s_boot_date_label && s_font_addr_30) {
      const char *months[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                              "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
      lv_label_set_text_fmt(s_boot_date_label, "%s %02d %s",
                            months[timeinfo.tm_mon % 12], day,
                            week_days[wday % 7]);
    }
  } else if (s_current_mode == DISPLAY_MODE_CLOCK) {
    bool use_clock2 = (s_clock_option == 1);
    if (s_clock_option == 2) {
      if (s_auto_clock_offset == -1) {
        s_auto_clock_offset = (int)(esp_random() % 2);
      }
      use_clock2 = ((hour + s_auto_clock_offset) % 2 == 0);
    }
    
    if (!use_clock2) draw_analog_clock(hour, minute, second);
    else draw_analog_clock2(hour, minute, second);
  }
}


// 부팅 모드 전용 디지털 시계 UI 생성
static void create_boot_ui(void) {
  ESP_LOGI(TAG, "[UI] Step 3-1: Creating BOOT screen components (Hidden by default)...");
  if (s_boot_screen == NULL)
    return;

  // 1. HH:MM (Large, Center)
  s_boot_time_label = lv_label_create(s_boot_screen);
  lv_obj_set_style_text_font(s_boot_time_label, &font_ORB_100, 0);
  lv_obj_set_style_text_color(s_boot_time_label, lv_color_white(), 0);
  lv_obj_align(s_boot_time_label, LV_ALIGN_CENTER, 0, 0); // Perfectly centered
  lv_label_set_text(s_boot_time_label, "12:00");
  lv_obj_add_flag(s_boot_time_label, LV_OBJ_FLAG_HIDDEN);

  // 3. Date Header (Top)
  s_boot_date_label = lv_label_create(s_boot_screen);
  lv_obj_set_style_text_font(s_boot_date_label, &font_addr_30, 0);
  lv_obj_set_style_text_color(s_boot_date_label, lv_color_hex(0xAAAAAA), 0);
  lv_obj_align(s_boot_date_label, LV_ALIGN_CENTER, 0, -100);
  lv_label_set_text(s_boot_date_label, "JAN 01 MON");
  lv_obj_add_flag(s_boot_date_label, LV_OBJ_FLAG_HIDDEN);

  // App Reg Info (Hidden by default)
  s_boot_reg_title_label = lv_label_create(s_boot_screen);
  lv_obj_set_style_text_font(s_boot_reg_title_label, &font_addr_30, 0); 
  lv_obj_set_style_text_color(s_boot_reg_title_label, lv_color_hex(0xAAAAAA), 0);
  lv_obj_align(s_boot_reg_title_label, LV_ALIGN_CENTER, 0, 135);
  lv_label_set_text(s_boot_reg_title_label, "MCon K 등록번호");
  lv_obj_add_flag(s_boot_reg_title_label, LV_OBJ_FLAG_HIDDEN);

  s_boot_reg_val_label = lv_label_create(s_boot_screen);
  if (strcmp(s_app_reg_num, "판매사에 문의하세요..") == 0) {
    lv_obj_set_style_text_font(s_boot_reg_val_label, &font_addr_30, 0); 
    lv_obj_set_style_text_color(s_boot_reg_val_label, lv_color_make(255, 100, 0), 0); // Orange
  } else {
    lv_obj_set_style_text_font(s_boot_reg_val_label, &font_kopub_40, 0); 
    lv_obj_set_style_text_color(s_boot_reg_val_label, lv_color_hex(0xFFFF00), 0);
  }
  lv_obj_align(s_boot_reg_val_label, LV_ALIGN_CENTER, 0, 180);
  lv_label_set_text_fmt(s_boot_reg_val_label, "%s", s_app_reg_num);
  lv_obj_add_flag(s_boot_reg_val_label, LV_OBJ_FLAG_HIDDEN);

  // App Installation Info (Top Side, Hidden by default)
  s_boot_install_title_label = lv_label_create(s_boot_screen);
  lv_obj_set_style_text_font(s_boot_install_title_label, &font_addr_30, 0); 
  lv_obj_set_style_text_color(s_boot_install_title_label, lv_color_white(), 0);
  lv_obj_align(s_boot_install_title_label, LV_ALIGN_CENTER, 0, -170);
  lv_label_set_text(s_boot_install_title_label, "MCon K 설치");
  lv_obj_add_flag(s_boot_install_title_label, LV_OBJ_FLAG_HIDDEN);

  s_boot_install_sub_label = lv_label_create(s_boot_screen);
  lv_obj_set_style_text_font(s_boot_install_sub_label, &font_addr_30, 0); 
  lv_obj_set_style_text_color(s_boot_install_sub_label, lv_color_hex(0xAAAAAA), 0);
  lv_obj_align(s_boot_install_sub_label, LV_ALIGN_CENTER, 0, -135);
  lv_label_set_text(s_boot_install_sub_label, "설치방법..");
  lv_obj_add_flag(s_boot_install_sub_label, LV_OBJ_FLAG_HIDDEN);

  // QR Code (Perfect Center)
  s_boot_install_qr = lv_qrcode_create(s_boot_screen, 160, lv_color_make(180, 180, 180), lv_color_black());
  if (s_boot_install_qr) {
    const char *qr_data = "https://play.google.com/store/search?q=mcon+k&c=apps&hl=ko";
    lv_qrcode_update(s_boot_install_qr, qr_data, strlen(qr_data));
    lv_obj_align(s_boot_install_qr, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_border_color(s_boot_install_qr, lv_color_make(180, 180, 180), 0);
    lv_obj_set_style_border_width(s_boot_install_qr, 3, 0);
    lv_obj_set_style_border_side(s_boot_install_qr, LV_BORDER_SIDE_FULL, 0);
    lv_obj_add_flag(s_boot_install_qr, LV_OBJ_FLAG_HIDDEN);
  }
}

/* static void rotate_point(int px, int py, double angle_rad, int *ox, int *oy) {
  *ox = (int)(px * cos(angle_rad) - py * sin(angle_rad));
  *oy = (int)(px * sin(angle_rad) + py * cos(angle_rad));
} */

static void draw_analog_clock(int hour, int minute, int second) {
  if (!s_clock1_hour_bg) return;
  const int cx = LCD_H_RES / 2;
  const int cy = LCD_V_RES / 2;
  const int r = (LCD_H_RES < LCD_V_RES ? LCD_H_RES : LCD_V_RES) / 2 - 20;

  // Hour Hand (Proportional increase and match thickness)
  double h_rad = ((hour % 12) * 30 + minute * 0.5 - 90) * M_PI / 180.0;
  int hl = r * 0.65; // Slightly lengthened for larger M-logo
  int h_off = 43;    // Increased to 43px

  // BG (Full length)
  s_clock1_hour_points[0].x = cx; s_clock1_hour_points[0].y = cy;
  s_clock1_hour_points[1].x = cx + (int)(hl * cos(h_rad));
  s_clock1_hour_points[1].y = cy + (int)(hl * sin(h_rad));
  lv_line_set_points(s_clock1_hour_bg, s_clock1_hour_points, 2);

  // Hour Hand Shadow
  s_clock1_hour_shadow_points[0].x = cx + 10; s_clock1_hour_shadow_points[0].y = cy + 10;
  s_clock1_hour_shadow_points[1].x = cx + (int)(hl * cos(h_rad)) + 10;
  s_clock1_hour_shadow_points[1].y = cy + (int)(hl * sin(h_rad)) + 10;
  lv_line_set_points(s_clock1_hour_shadow, s_clock1_hour_shadow_points, 2);

  // FG (Offset Start)
  s_clock1_hour_fg_points[0].x = cx + (int)(h_off * cos(h_rad));
  s_clock1_hour_fg_points[0].y = cy + (int)(h_off * sin(h_rad));
  s_clock1_hour_fg_points[1].x = s_clock1_hour_points[1].x;
  s_clock1_hour_fg_points[1].y = s_clock1_hour_points[1].y;
  lv_line_set_points(s_clock1_hour_fg, s_clock1_hour_fg_points, 2);

  // Minute Hand (Maximize length)
  double m_rad = (minute * 6 + second * 0.1 - 90) * M_PI / 180.0;
  int ml = r * 0.95; // Maximum length to match larger aesthetic
  int m_off = 43;    // Increased to 43px

  // BG (Full length)
  s_clock1_minute_points[0].x = cx; s_clock1_minute_points[0].y = cy;
  s_clock1_minute_points[1].x = cx + (int)(ml * cos(m_rad));
  s_clock1_minute_points[1].y = cy + (int)(ml * sin(m_rad));
  lv_line_set_points(s_clock1_minute_bg, s_clock1_minute_points, 2);

  // Minute Hand Shadow
  s_clock1_minute_shadow_points[0].x = cx + 10; s_clock1_minute_shadow_points[0].y = cy + 10;
  s_clock1_minute_shadow_points[1].x = cx + (int)(ml * cos(m_rad)) + 10;
  s_clock1_minute_shadow_points[1].y = cy + (int)(ml * sin(m_rad)) + 10;
  lv_line_set_points(s_clock1_minute_shadow, s_clock1_minute_shadow_points, 2);

  // FG (Offset Start)
  s_clock1_minute_fg_points[0].x = cx + (int)(m_off * cos(m_rad));
  s_clock1_minute_fg_points[0].y = cy + (int)(m_off * sin(m_rad));
  s_clock1_minute_fg_points[1].x = s_clock1_minute_points[1].x;
  s_clock1_minute_fg_points[1].y = s_clock1_minute_points[1].y;
  lv_line_set_points(s_clock1_minute_fg, s_clock1_minute_fg_points, 2);
}

static void draw_analog_clock2(int hour, int minute, int second) {
  if (!s_clock2_hour_line) return;
  const int cx = LCD_H_RES / 2;
  const int cy = LCD_V_RES / 2;
  const int r = (LCD_H_RES < LCD_V_RES ? LCD_H_RES : LCD_V_RES) / 2 - 20;

  double h_rad = ((hour % 12) * 30 + minute * 0.5 - 90) * M_PI / 180.0;
  int hl = r * 0.55;
  s_clock2_hour_points[0].x = cx; s_clock2_hour_points[0].y = cy;
  s_clock2_hour_points[1].x = cx + (int)(hl * cos(h_rad));
  s_clock2_hour_points[1].y = cy + (int)(hl * sin(h_rad));
  lv_line_set_points(s_clock2_hour_line, s_clock2_hour_points, 2);

  double m_rad = (minute * 6 + second * 0.1 - 90) * M_PI / 180.0;
  int ml = r * 0.8;
  s_clock2_minute_points[0].x = cx; s_clock2_minute_points[0].y = cy;
  s_clock2_minute_points[1].x = cx + (int)(ml * cos(m_rad));
  s_clock2_minute_points[1].y = cy + (int)(ml * sin(m_rad));
  lv_line_set_points(s_clock2_minute_line, s_clock2_minute_points, 2);

  double s_rad = (second * 6 - 90) * M_PI / 180.0;
  int sl = r * 0.85;
  int s_tail = -50;
  s_clock2_second_points[0].x = cx + (int)(s_tail * cos(s_rad));
  s_clock2_second_points[0].y = cy + (int)(s_tail * sin(s_rad));
  s_clock2_second_points[1].x = cx + (int)(sl * cos(s_rad));
  s_clock2_second_points[1].y = cy + (int)(sl * sin(s_rad));
  lv_line_set_points(s_clock2_second_line, s_clock2_second_points, 2);
}

static void create_clock_ui(void) {
  if (s_clock_screen == NULL) return;
  ESP_LOGI(TAG, "[CLOCK UI] Loading background: S:/littlefs/clock_1/screen.jpg");
  
  // Clean up image cache to ensure fresh loading from PSRAM
  lv_img_cache_invalidate_src(NULL);

  s_clock_bg_img = lv_img_create(s_clock_screen);
  lv_img_set_src(s_clock_bg_img, "S:/littlefs/clock_1/screen.jpg");
  lv_img_set_zoom(s_clock_bg_img, 256); // 100% scale (Original)
  lv_obj_center(s_clock_bg_img);
  // Disable scrolling to remove white horizontal scrollbar at bottom
  lv_obj_clear_flag(s_clock_screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(s_clock_screen, LV_SCROLLBAR_MODE_OFF);

  lv_color_t orange_fill = lv_color_make(255, 100, 0); 
  lv_color_t dark_fill = lv_color_make(64, 64, 64);     

  // Shadow lines (Draw first to be at bottom)
  s_clock1_hour_shadow = lv_line_create(s_clock_screen);
  lv_obj_set_style_line_width(s_clock1_hour_shadow, 32, 0);
  lv_obj_set_style_line_color(s_clock1_hour_shadow, lv_color_black(), 0);
  lv_obj_set_style_line_opa(s_clock1_hour_shadow, LV_OPA_20, 0);
  lv_obj_set_style_line_rounded(s_clock1_hour_shadow, true, 0);

  s_clock1_minute_shadow = lv_line_create(s_clock_screen);
  lv_obj_set_style_line_width(s_clock1_minute_shadow, 32, 0);
  lv_obj_set_style_line_color(s_clock1_minute_shadow, lv_color_black(), 0);
  lv_obj_set_style_line_opa(s_clock1_minute_shadow, LV_OPA_20, 0);
  lv_obj_set_style_line_rounded(s_clock1_minute_shadow, true, 0);

  s_clock1_hour_bg = lv_line_create(s_clock_screen);
  lv_obj_set_style_line_width(s_clock1_hour_bg, 32, 0); // Thicker (was 26px)
  lv_obj_set_style_line_color(s_clock1_hour_bg, lv_color_white(), 0);
  lv_obj_set_style_line_rounded(s_clock1_hour_bg, true, 0);

  s_clock1_hour_fg = lv_line_create(s_clock_screen);
  lv_obj_set_style_line_width(s_clock1_hour_fg, 22, 0); // Thicker (was 26px)
  lv_obj_set_style_line_color(s_clock1_hour_fg, dark_fill, 0);
  lv_obj_set_style_line_rounded(s_clock1_hour_fg, true, 0);

  s_clock1_minute_bg = lv_line_create(s_clock_screen);
  lv_obj_set_style_line_width(s_clock1_minute_bg, 32, 0); // Thicker (was 26px)
  lv_obj_set_style_line_color(s_clock1_minute_bg, lv_color_white(), 0);
  lv_obj_set_style_line_rounded(s_clock1_minute_bg, true, 0);

  s_clock1_minute_fg = lv_line_create(s_clock_screen);
  lv_obj_set_style_line_width(s_clock1_minute_fg, 22, 0); // Thicker (was 26px)
  lv_obj_set_style_line_color(s_clock1_minute_fg, orange_fill, 0);
  lv_obj_set_style_line_rounded(s_clock1_minute_fg, true, 0);

  s_clock_center_dot = lv_obj_create(s_clock_screen);
  lv_obj_set_size(s_clock_center_dot, 46, 46); // Adjusted to 46px
  lv_obj_set_style_radius(s_clock_center_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(s_clock_center_dot, lv_color_hex(0x404040), 0); // Lighter Grey
  lv_obj_set_style_border_width(s_clock_center_dot, 0, 0); // Removed white border
  // Center dot shadow for 3D depth
  lv_obj_set_style_shadow_width(s_clock_center_dot, 15, 0);
  lv_obj_set_style_shadow_color(s_clock_center_dot, lv_color_black(), 0);
  lv_obj_set_style_shadow_opa(s_clock_center_dot, LV_OPA_20, 0);
  lv_obj_set_style_shadow_ofs_x(s_clock_center_dot, 10, 0);
  lv_obj_set_style_shadow_ofs_y(s_clock_center_dot, 10, 0);
  lv_obj_center(s_clock_center_dot);

  if (s_clock_timer == NULL) s_clock_timer = lv_timer_create(clock_timer_cb, 1000, NULL);
}

static void create_clock2_ui(void) {
  if (s_clock2_screen != NULL) return;
  s_clock2_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s_clock2_screen, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(s_clock2_screen, LV_OPA_COVER, 0); // Ensure background is opaque black
  ESP_LOGI(TAG, "[CLOCK UI] Loading background: S:/littlefs/clock_2/screen.png");
  lv_obj_t *bg_img = lv_img_create(s_clock2_screen);
  lv_img_set_src(bg_img, "S:/littlefs/clock_2/screen.png");
  lv_obj_center(bg_img);

  lv_color_t gold_color = lv_color_make(220, 190, 120);
  s_clock2_hour_line = lv_line_create(s_clock2_screen);
  lv_obj_set_style_line_width(s_clock2_hour_line, 8, 0);
  lv_obj_set_style_line_color(s_clock2_hour_line, gold_color, 0);
  lv_obj_set_style_line_rounded(s_clock2_hour_line, true, 0);

  s_clock2_minute_line = lv_line_create(s_clock2_screen);
  lv_obj_set_style_line_width(s_clock2_minute_line, 5, 0);
  lv_obj_set_style_line_color(s_clock2_minute_line, gold_color, 0);
  lv_obj_set_style_line_rounded(s_clock2_minute_line, true, 0);

  s_clock2_second_line = lv_line_create(s_clock2_screen);
  lv_obj_set_style_line_width(s_clock2_second_line, 2, 0);
  lv_obj_set_style_line_color(s_clock2_second_line, gold_color, 0);
  lv_obj_set_style_line_rounded(s_clock2_second_line, true, 0);

  lv_obj_t *center_img = lv_img_create(s_clock2_screen);
  lv_img_set_src(center_img, "S:/littlefs/clock_2/center.png");
  lv_obj_center(center_img);
}

static void scan_intro_images(void);

static void create_album_ui(void) {
  s_album_img = lv_img_create(s_album_screen);
  lv_obj_center(s_album_img);

  // 안내 문구 라벨 (사진 없을 때 표시)
  s_album_guide_label = lv_label_create(s_album_screen);
  lv_obj_set_style_text_font(s_album_guide_label, &font_addr_30, 0);
  lv_obj_set_style_text_color(s_album_guide_label, lv_color_make(180, 160, 0), 0); // 어두운 노랑색 (Dark Yellow)
  lv_label_set_text(s_album_guide_label, "앱에서 사진을 보내주세요!");
  lv_obj_align(s_album_guide_label, LV_ALIGN_CENTER, 0, 150); // 중앙에서 아래로 150px
  lv_obj_add_flag(s_album_guide_label, LV_OBJ_FLAG_HIDDEN);

  // Scan and load default image (toy_car.jpg)
  reset_album_to_default_image();
}

static void create_speedometer_ui(void) {
  if (s_speedometer_screen == NULL)
    return;

  // 1. Background Image
  struct stat st;
  if (stat("/littlefs/speed/screen.png", &st) == 0) {
    ESP_LOGI(TAG, "[UI] Speedometer background found (%ld bytes)", st.st_size);
    s_speedometer_bg_img = lv_img_create(s_speedometer_screen);
    lv_img_set_src(s_speedometer_bg_img, "S:/littlefs/speed/screen.png");
    lv_obj_center(s_speedometer_bg_img);
    lv_obj_clear_flag(s_speedometer_bg_img, LV_OBJ_FLAG_CLICKABLE);
  } else {
    ESP_LOGE(TAG, "[UI] Error: Speedometer background NOT FOUND at /littlefs/speed/screen.png");
    lv_obj_set_style_bg_color(s_speedometer_screen, lv_color_make(0, 0, 50), 0); // Dark Blue as fallback
  }

  // 2. Needle Line (Tapered shape)
  s_speedometer_needle_line = lv_line_create(s_speedometer_screen);
  lv_obj_set_style_line_width(s_speedometer_needle_line, 5,
                              0); // 5px width * 2 offset = 10px center
  lv_obj_set_style_line_color(s_speedometer_needle_line, lv_color_hex(0xFF0033),
                              0); // Vibrant Red
  lv_obj_set_style_line_rounded(s_speedometer_needle_line, true, 0);
  lv_obj_clear_flag(s_speedometer_needle_line, LV_OBJ_FLAG_CLICKABLE);

  // Set initial position (0 km/h)
  double initial_angle_rad = 156.0 * M_PI / 180.0;
  int cx = LCD_H_RES / 2;
  int cy = LCD_V_RES / 2;
  int r = (LCD_H_RES < LCD_V_RES ? LCD_H_RES : LCD_V_RES) / 2 - 20;
  int nl = r * 0.85;
  int tl = -40;

  double cos_i = cos(initial_angle_rad);
  double sin_i = sin(initial_angle_rad);
  double cos_ip = cos(initial_angle_rad + M_PI / 2.0);
  double sin_ip = sin(initial_angle_rad + M_PI / 2.0);

  s_speedometer_needle_points[0].x = cx + (int)(nl * cos_i);
  s_speedometer_needle_points[0].y = cy + (int)(nl * sin_i);
  s_speedometer_needle_points[1].x = cx + (int)(2.5 * cos_ip);
  s_speedometer_needle_points[1].y = cy + (int)(2.5 * sin_ip);
  s_speedometer_needle_points[2].x = cx + (int)(tl * cos_i);
  s_speedometer_needle_points[2].y = cy + (int)(tl * sin_i);
  s_speedometer_needle_points[3].x = cx + (int)(-2.5 * cos_ip);
  s_speedometer_needle_points[3].y = cy + (int)(-2.5 * sin_ip);
  s_speedometer_needle_points[4] = s_speedometer_needle_points[0];

  lv_line_set_points(s_speedometer_needle_line, s_speedometer_needle_points, 5);

  // 3. Center Cap Image
  s_speedometer_center_img = lv_img_create(s_speedometer_screen);
  lv_img_set_src(s_speedometer_center_img, "S:/littlefs/speed/center.png");
  lv_obj_center(s_speedometer_center_img);
  lv_obj_clear_flag(s_speedometer_center_img, LV_OBJ_FLAG_CLICKABLE);

  // 2. Speed Labels
  s_speedometer_speed_label = lv_label_create(s_speedometer_screen);
  lv_obj_set_style_text_font(s_speedometer_speed_label, &font_ORB_100, 0);
  lv_obj_set_style_text_color(s_speedometer_speed_label, lv_color_white(), 0);
  lv_label_set_text(s_speedometer_speed_label, "0");
  lv_obj_align(s_speedometer_speed_label, LV_ALIGN_CENTER, 0, 116);

  s_speedometer_unit_label = lv_label_create(s_speedometer_screen);
  lv_obj_set_style_text_font(s_speedometer_unit_label, &font_kopub_35, 0);
  lv_obj_set_style_text_color(s_speedometer_unit_label, lv_color_white(), 0);
  lv_label_set_text(s_speedometer_unit_label, "km/h");
  lv_obj_align(s_speedometer_unit_label, LV_ALIGN_CENTER, 0, 190);

  // 3. Safety UI Elements (Initially Hidden)
  s_speedometer_safety_arc = lv_arc_create(s_speedometer_screen);
  lv_obj_set_size(s_speedometer_safety_arc, 450, 450); // Diameter 450px
  lv_obj_center(s_speedometer_safety_arc);

  // Set the arc to match the speedometer scale: 156 start, 228 degrees span
  lv_arc_set_rotation(s_speedometer_safety_arc, 156);
  lv_arc_set_bg_angles(s_speedometer_safety_arc, 0, 228);
  lv_arc_set_mode(s_speedometer_safety_arc, LV_ARC_MODE_NORMAL);

  lv_obj_remove_style(s_speedometer_safety_arc, NULL, LV_PART_KNOB);
  lv_obj_set_style_arc_width(s_speedometer_safety_arc, 0,
                             LV_PART_MAIN); // Hide background completely
  lv_obj_set_style_arc_rounded(s_speedometer_safety_arc, false, LV_PART_MAIN);

  lv_obj_set_style_arc_width(s_speedometer_safety_arc, 10,
                             LV_PART_INDICATOR); // Width 10px
  lv_obj_set_style_arc_color(s_speedometer_safety_arc, lv_color_hex(0xFF8800),
                             LV_PART_INDICATOR); // Orange
  lv_obj_set_style_arc_opa(
      s_speedometer_safety_arc, LV_OPA_COVER,
      LV_PART_INDICATOR); // 100% opaque to skip alpha blending overhead
  lv_obj_set_style_arc_rounded(
      s_speedometer_safety_arc, false,
      LV_PART_INDICATOR); // Flat ends exactly stop at intended angle

  // 180~220km/h 구간 표시 (기본값)
  int start_angle = (int)((180 / 220.0) * 228.0);
  lv_arc_set_angles(s_speedometer_safety_arc, start_angle, 228);

  lv_obj_clear_flag(s_speedometer_safety_arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(s_speedometer_safety_arc,
                    LV_OBJ_FLAG_HIDDEN); // 기본적으로 표시됨

  s_speedometer_safety_image = lv_img_create(s_speedometer_screen);
  lv_obj_add_flag(s_speedometer_safety_image, LV_OBJ_FLAG_HIDDEN);
  lv_obj_align(s_speedometer_safety_image, LV_ALIGN_CENTER, 0, 116);

  s_speedometer_safety_value_label = lv_label_create(s_speedometer_screen);
  lv_obj_add_flag(s_speedometer_safety_value_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_text_font(s_speedometer_safety_value_label, &font_kopub_35,
                             0);
  lv_obj_set_style_text_color(s_speedometer_safety_value_label,
                              lv_color_hex(0x00FF00), 0); // Green
  lv_obj_align(s_speedometer_safety_value_label, LV_ALIGN_CENTER, 110, 146);

  s_speedometer_safety_unit_label = lv_label_create(s_speedometer_screen);
  lv_obj_add_flag(s_speedometer_safety_unit_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_text_font(s_speedometer_safety_unit_label, &font_kopub_25,
                             0);
  lv_obj_set_style_text_color(s_speedometer_safety_unit_label, lv_color_white(),
                              0);
  lv_obj_align(s_speedometer_safety_unit_label, LV_ALIGN_CENTER, 170, 146);

  // Road Name Label for Speedometer Mode
  s_speedometer_road_name_label = lv_label_create(s_speedometer_screen);
  lv_obj_add_flag(s_speedometer_road_name_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_text_font(s_speedometer_road_name_label, &font_addr_30, 0); // Use address font
  lv_obj_set_style_text_color(s_speedometer_road_name_label, lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_text_align(s_speedometer_road_name_label, LV_TEXT_ALIGN_CENTER, 0);
  // 위치를 HUD 모드와 동일하게 설정 (1행 중심 403pt 위치)
  lv_obj_align(s_speedometer_road_name_label, LV_ALIGN_TOP_MID, 0, 388);
  lv_label_set_text(s_speedometer_road_name_label, "");

  s_speedometer_road_name_sub_label = lv_label_create(s_speedometer_screen);
  lv_obj_add_flag(s_speedometer_road_name_sub_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_text_font(s_speedometer_road_name_sub_label, &font_addr_30, 0);
  lv_obj_set_style_text_color(s_speedometer_road_name_sub_label, lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_text_align(s_speedometer_road_name_sub_label, LV_TEXT_ALIGN_CENTER, 0);
  // 2행 위치: 1행(388) + 폰트 높이 고려 (약 35pt 간격)
  lv_obj_align(s_speedometer_road_name_sub_label, LV_ALIGN_TOP_MID, 0, 423);
  lv_label_set_text(s_speedometer_road_name_sub_label, "");

  // 4. Average Speed Labels (Initially Hidden)
  // [구간속도 문구] 10pt [구간속도 숫자] 5pt [구간속도 단위]
  // 위치: 문구("구간속도")가 화면 중앙에서 아래로 50pt 이동 (중앙 정렬 기준)

  // 4. Average Speed Labels (Initially Hidden)
  // [구간속도 문구] 10pt [구간속도 숫자] 5pt [구간속도 단위]
  // 위치: [구간속도 숫자]의 앞자리가 화면 중심에서 위로 130pt 이동 (숫자
  // 좌측이 센터 정렬)

  // Value Label (Anchor for this layout logic)
  s_speedometer_avr_speed_value_label = lv_label_create(s_speedometer_screen);
  lv_obj_set_style_text_font(s_speedometer_avr_speed_value_label,
                             &font_kopub_25, 0); // 25pt font
  lv_obj_set_style_text_color(s_speedometer_avr_speed_value_label,
                              lv_color_white(), 0); // White
  lv_label_set_text(s_speedometer_avr_speed_value_label, "0");
  // Position will be set dynamically in update_speed_label (Left Edge at
  // Center)
  lv_obj_add_flag(s_speedometer_avr_speed_value_label, LV_OBJ_FLAG_HIDDEN);

  // Title Label
  s_speedometer_avr_speed_title_label = lv_label_create(s_speedometer_screen);
  lv_obj_set_style_text_font(s_speedometer_avr_speed_title_label,
                             &font_kopub_20, 0); // 20pt font
  lv_obj_set_style_text_color(s_speedometer_avr_speed_title_label,
                              lv_color_white(), 0); // White
  lv_label_set_text(s_speedometer_avr_speed_title_label, "구간속도");
  // Position will be set dynamically in update_speed_label (Left of Value)
  lv_obj_add_flag(s_speedometer_avr_speed_title_label, LV_OBJ_FLAG_HIDDEN);

  // Unit Label
  s_speedometer_avr_speed_unit_label = lv_label_create(s_speedometer_screen);
  lv_obj_set_style_text_font(s_speedometer_avr_speed_unit_label, &font_kopub_20,
                             0); // 20pt font
  lv_obj_set_style_text_color(s_speedometer_avr_speed_unit_label,
                              lv_color_white(), 0); // White
  lv_label_set_text(s_speedometer_avr_speed_unit_label, "km/h");
  // Position will be set dynamically in update_speed_label (Right of Value)
  lv_obj_add_flag(s_speedometer_avr_speed_unit_label, LV_OBJ_FLAG_HIDDEN);

  // 5. Ensure Center Cap is on top level
  lv_obj_move_foreground(s_speedometer_center_img);
}

static void create_setting_ui(void) {
  if (s_setting_screen == NULL)
    return;

  // 1. SETUP Title (Moved Up for Safe Zone)
  s_setting_title_label = lv_label_create(s_setting_screen);
  lv_obj_set_style_text_font(s_setting_title_label, &font_kopub_40, 0);
  lv_obj_set_style_text_color(s_setting_title_label, lv_color_white(), 0);
  lv_label_set_text(s_setting_title_label, "SETUP 1");
  lv_obj_align(s_setting_title_label, LV_ALIGN_TOP_MID, 0, 44);

  // --- PAGE 1 CONTAINER (Narrow & Safe) ---
  s_setting_page1_obj = lv_obj_create(s_setting_screen);
  lv_obj_set_size(s_setting_page1_obj, 460, 340);
  lv_obj_align(s_setting_page1_obj, LV_ALIGN_TOP_MID, 0, 99);
  lv_obj_set_style_bg_opa(s_setting_page1_obj, 0, 0);
  lv_obj_set_style_border_width(s_setting_page1_obj, 0, 0);
  lv_obj_set_scrollbar_mode(s_setting_page1_obj, LV_SCROLLBAR_MODE_OFF);

  // Row 1: 밝기 옵션 (Moved Left)
  lv_obj_t *label1 = lv_label_create(s_setting_page1_obj);
  lv_obj_set_style_text_font(label1, &font_addr_30, 0);
  lv_obj_set_style_text_color(label1, lv_color_make(210, 210, 0), 0);
  lv_label_set_text(label1, "밝기 옵션");
  lv_obj_align(label1, LV_ALIGN_TOP_MID, -115, 10);

  s_setting_btn_down = lv_btn_create(s_setting_page1_obj);
  lv_obj_set_size(s_setting_btn_down, 60, 60);
  lv_obj_align(s_setting_btn_down, LV_ALIGN_TOP_MID, 15, 0);
  lv_obj_set_style_radius(s_setting_btn_down, 15, 0);
  lv_obj_set_style_bg_color(s_setting_btn_down, lv_color_hex(0x222222), 0);
  lv_obj_set_style_bg_color(s_setting_btn_down, lv_color_hex(0x555555),
                            LV_STATE_PRESSED);
  lv_obj_set_style_bg_color(s_setting_btn_down, lv_color_hex(0x222222),
                            LV_STATE_DISABLED);
  lv_obj_set_style_bg_opa(s_setting_btn_down, 255, 0);
  lv_obj_set_style_bg_opa(s_setting_btn_down, 255, LV_STATE_DISABLED);
  lv_obj_set_style_opa(s_setting_btn_down, 255, 0);
  lv_obj_set_style_opa(s_setting_btn_down, 255, LV_STATE_DISABLED);
  lv_obj_set_style_shadow_width(s_setting_btn_down, 0, 0);
  lv_obj_set_style_shadow_width(s_setting_btn_down, 0, LV_STATE_DISABLED);
  lv_obj_set_style_border_width(s_setting_btn_down, 0, 0);
  lv_obj_set_style_border_width(s_setting_btn_down, 0, LV_STATE_DISABLED);
  lv_obj_add_event_cb(s_setting_btn_down, brightness_down_event_cb,
                      LV_EVENT_ALL, NULL);

  // Vector Arrow Down (Centered & Corrected)
  static lv_point_t points_dn[3] = {{13, 20}, {28, 36}, {43, 20}};
  s_setting_line_bright_dn = lv_line_create(s_setting_btn_down);
  lv_obj_set_size(s_setting_line_bright_dn, 60, 60);
  lv_line_set_points(s_setting_line_bright_dn, points_dn, 3);
  lv_obj_set_style_line_width(s_setting_line_bright_dn, 6, 0);
  lv_obj_set_style_line_color(s_setting_line_bright_dn,
                              lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_line_rounded(s_setting_line_bright_dn, true, 0);
  lv_obj_align(s_setting_line_bright_dn, LV_ALIGN_CENTER, 0, 0);

  s_setting_circ_bright_dn = lv_obj_create(s_setting_btn_down);
  lv_obj_set_size(s_setting_circ_bright_dn, 26, 26);
  lv_obj_set_style_radius(s_setting_circ_bright_dn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(s_setting_circ_bright_dn, 0, 0);
  lv_obj_set_style_border_width(s_setting_circ_bright_dn, 4, 0);
  lv_obj_set_style_border_color(s_setting_circ_bright_dn,
                                lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_pad_all(s_setting_circ_bright_dn, 0, 0);
  lv_obj_center(s_setting_circ_bright_dn);
  lv_obj_add_flag(s_setting_circ_bright_dn, LV_OBJ_FLAG_HIDDEN);

  s_setting_bright_val_label = lv_label_create(s_setting_page1_obj);
  lv_obj_set_style_text_font(s_setting_bright_val_label, &font_kopub_35, 0);
  lv_obj_set_style_text_color(s_setting_bright_val_label,
                              lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_align(s_setting_bright_val_label, LV_ALIGN_TOP_MID, 75, 10);

  s_setting_btn_up = lv_btn_create(s_setting_page1_obj);
  lv_obj_set_size(s_setting_btn_up, 60, 60);
  lv_obj_align(s_setting_btn_up, LV_ALIGN_TOP_MID, 135, 0);
  lv_obj_set_style_radius(s_setting_btn_up, 15, 0);
  lv_obj_set_style_bg_color(s_setting_btn_up, lv_color_hex(0x222222), 0);
  lv_obj_set_style_bg_color(s_setting_btn_up, lv_color_hex(0x555555),
                            LV_STATE_PRESSED);
  lv_obj_set_style_bg_color(s_setting_btn_up, lv_color_hex(0x222222),
                            LV_STATE_DISABLED);
  lv_obj_set_style_bg_opa(s_setting_btn_up, 255, 0);
  lv_obj_set_style_bg_opa(s_setting_btn_up, 255, LV_STATE_DISABLED);
  lv_obj_set_style_opa(s_setting_btn_up, 255, 0);
  lv_obj_set_style_opa(s_setting_btn_up, 255, LV_STATE_DISABLED);
  lv_obj_set_style_shadow_width(s_setting_btn_up, 0, 0);
  lv_obj_set_style_shadow_width(s_setting_btn_up, 0, LV_STATE_DISABLED);
  lv_obj_set_style_border_width(s_setting_btn_up, 0, 0);
  lv_obj_set_style_border_width(s_setting_btn_up, 0, LV_STATE_DISABLED);
  lv_obj_add_event_cb(s_setting_btn_up, brightness_up_event_cb,
                      LV_EVENT_ALL, NULL);

  // Vector Arrow Up (Centered & Corrected)
  static lv_point_t points_up[3] = {{13, 36}, {28, 20}, {43, 36}};
  s_setting_line_bright_up = lv_line_create(s_setting_btn_up);
  lv_obj_set_size(s_setting_line_bright_up, 60, 60);
  lv_line_set_points(s_setting_line_bright_up, points_up, 3);
  lv_obj_set_style_line_width(s_setting_line_bright_up, 6, 0);
  lv_obj_set_style_line_color(s_setting_line_bright_up,
                              lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_line_rounded(s_setting_line_bright_up, true, 0);
  lv_obj_align(s_setting_line_bright_up, LV_ALIGN_CENTER, 0, 0);

  s_setting_circ_bright_up = lv_obj_create(s_setting_btn_up);
  lv_obj_set_size(s_setting_circ_bright_up, 26, 26);
  lv_obj_set_style_radius(s_setting_circ_bright_up, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(s_setting_circ_bright_up, 0, 0);
  lv_obj_set_style_border_width(s_setting_circ_bright_up, 4, 0);
  lv_obj_set_style_border_color(s_setting_circ_bright_up,
                                lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_pad_all(s_setting_circ_bright_up, 0, 0);
  lv_obj_center(s_setting_circ_bright_up);
  lv_obj_add_flag(s_setting_circ_bright_up, LV_OBJ_FLAG_HIDDEN);

  // Row 2: 앨범 옵션 (Moved Left)
  lv_obj_t *label2 = lv_label_create(s_setting_page1_obj);
  lv_obj_set_style_text_font(label2, &font_addr_30, 0);
  lv_obj_set_style_text_color(label2, lv_color_make(210, 210, 0), 0);
  lv_label_set_text(label2, "앨범 옵션");
  lv_obj_align(label2, LV_ALIGN_TOP_MID, -115, 90);

  s_setting_album_btn_down = lv_btn_create(s_setting_page1_obj);
  lv_obj_set_size(s_setting_album_btn_down, 60, 60);
  lv_obj_align(s_setting_album_btn_down, LV_ALIGN_TOP_MID, 15, 80);
  lv_obj_set_style_radius(s_setting_album_btn_down, 15, 0);
  lv_obj_set_style_bg_color(s_setting_album_btn_down, lv_color_hex(0x222222),
                            0);
  lv_obj_set_style_bg_color(s_setting_album_btn_down, lv_color_hex(0x555555),
                            LV_STATE_PRESSED);
  lv_obj_set_style_bg_color(s_setting_album_btn_down, lv_color_hex(0x222222),
                            LV_STATE_DISABLED);
  lv_obj_set_style_bg_opa(s_setting_album_btn_down, 255, 0);
  lv_obj_set_style_bg_opa(s_setting_album_btn_down, 255, LV_STATE_DISABLED);
  lv_obj_set_style_opa(s_setting_album_btn_down, 255, 0);
  lv_obj_set_style_opa(s_setting_album_btn_down, 255, LV_STATE_DISABLED);
  lv_obj_set_style_shadow_width(s_setting_album_btn_down, 0, 0);
  lv_obj_set_style_shadow_width(s_setting_album_btn_down, 0, LV_STATE_DISABLED);
  lv_obj_set_style_border_width(s_setting_album_btn_down, 0, 0);
  lv_obj_set_style_border_width(s_setting_album_btn_down, 0, LV_STATE_DISABLED);
  lv_obj_add_event_cb(s_setting_album_btn_down, album_down_event_cb,
                      LV_EVENT_ALL, NULL);

  s_setting_line_album_dn = lv_line_create(s_setting_album_btn_down);
  lv_obj_set_size(s_setting_line_album_dn, 60, 60);
  lv_line_set_points(s_setting_line_album_dn, points_dn, 3);
  lv_obj_set_style_line_width(s_setting_line_album_dn, 6, 0);
  lv_obj_set_style_line_color(s_setting_line_album_dn,
                              lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_line_rounded(s_setting_line_album_dn, true, 0);
  lv_obj_align(s_setting_line_album_dn, LV_ALIGN_CENTER, 0, 0);

  s_setting_circ_album_dn = lv_obj_create(s_setting_album_btn_down);
  lv_obj_set_size(s_setting_circ_album_dn, 26, 26);
  lv_obj_set_style_radius(s_setting_circ_album_dn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(s_setting_circ_album_dn, 0, 0);
  lv_obj_set_style_border_width(s_setting_circ_album_dn, 4, 0);
  lv_obj_set_style_border_color(s_setting_circ_album_dn,
                                lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_pad_all(s_setting_circ_album_dn, 0, 0);
  lv_obj_center(s_setting_circ_album_dn);
  lv_obj_add_flag(s_setting_circ_album_dn, LV_OBJ_FLAG_HIDDEN);

  s_setting_album_val_label = lv_label_create(s_setting_page1_obj);
  lv_obj_set_style_text_font(s_setting_album_val_label, &font_kopub_35, 0);
  lv_obj_set_style_text_color(s_setting_album_val_label,
                              lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_align(s_setting_album_val_label, LV_ALIGN_TOP_MID, 75, 90);

  s_setting_album_btn_up = lv_btn_create(s_setting_page1_obj);
  lv_obj_set_size(s_setting_album_btn_up, 60, 60);
  lv_obj_align(s_setting_album_btn_up, LV_ALIGN_TOP_MID, 135, 80);
  lv_obj_set_style_radius(s_setting_album_btn_up, 15, 0);
  lv_obj_set_style_bg_color(s_setting_album_btn_up, lv_color_hex(0x222222), 0);
  lv_obj_set_style_bg_color(s_setting_album_btn_up, lv_color_hex(0x555555),
                            LV_STATE_PRESSED);
  lv_obj_set_style_bg_color(s_setting_album_btn_up, lv_color_hex(0x222222),
                            LV_STATE_DISABLED);
  lv_obj_set_style_bg_opa(s_setting_album_btn_up, 255, 0);
  lv_obj_set_style_bg_opa(s_setting_album_btn_up, 255, LV_STATE_DISABLED);
  lv_obj_set_style_opa(s_setting_album_btn_up, 255, 0);
  lv_obj_set_style_opa(s_setting_album_btn_up, 255, LV_STATE_DISABLED);
  lv_obj_set_style_shadow_width(s_setting_album_btn_up, 0, 0);
  lv_obj_set_style_shadow_width(s_setting_album_btn_up, 0, LV_STATE_DISABLED);
  lv_obj_set_style_border_width(s_setting_album_btn_up, 0, 0);
  lv_obj_set_style_border_width(s_setting_album_btn_up, 0, LV_STATE_DISABLED);
  lv_obj_add_event_cb(s_setting_album_btn_up, album_up_event_cb,
                      LV_EVENT_ALL, NULL);

  s_setting_line_album_up = lv_line_create(s_setting_album_btn_up);
  lv_obj_set_size(s_setting_line_album_up, 60, 60);
  lv_line_set_points(s_setting_line_album_up, points_up, 3);
  lv_obj_set_style_line_width(s_setting_line_album_up, 6, 0);
  lv_obj_set_style_line_color(s_setting_line_album_up,
                              lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_line_rounded(s_setting_line_album_up, true, 0);
  lv_obj_align(s_setting_line_album_up, LV_ALIGN_CENTER, 0, 0);

  s_setting_circ_album_up = lv_obj_create(s_setting_album_btn_up);
  lv_obj_set_size(s_setting_circ_album_up, 26, 26);
  lv_obj_set_style_radius(s_setting_circ_album_up, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(s_setting_circ_album_up, 0, 0);
  lv_obj_set_style_border_width(s_setting_circ_album_up, 4, 0);
  lv_obj_set_style_border_color(s_setting_circ_album_up,
                                lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_pad_all(s_setting_circ_album_up, 0, 0);
  lv_obj_center(s_setting_circ_album_up);
  lv_obj_add_flag(s_setting_circ_album_up, LV_OBJ_FLAG_HIDDEN);

  // Row 3: 시계 옵션 (Moved Left)
  lv_obj_t *label3 = lv_label_create(s_setting_page1_obj);
  lv_obj_set_style_text_font(label3, &font_addr_30, 0);
  lv_obj_set_style_text_color(label3, lv_color_make(210, 210, 0), 0);
  lv_label_set_text(label3, "시계 옵션");
  lv_obj_align(label3, LV_ALIGN_TOP_MID, -115, 170);

  s_setting_clock_btn_down = lv_btn_create(s_setting_page1_obj);
  lv_obj_set_size(s_setting_clock_btn_down, 60, 60);
  lv_obj_align(s_setting_clock_btn_down, LV_ALIGN_TOP_MID, 15, 160);
  lv_obj_set_style_radius(s_setting_clock_btn_down, 15, 0);
  lv_obj_set_style_bg_color(s_setting_clock_btn_down, lv_color_hex(0x222222),
                            0);
  lv_obj_set_style_bg_color(s_setting_clock_btn_down, lv_color_hex(0x555555),
                            LV_STATE_PRESSED);
  lv_obj_set_style_bg_color(s_setting_clock_btn_down, lv_color_hex(0x222222),
                            LV_STATE_DISABLED);
  lv_obj_set_style_bg_opa(s_setting_clock_btn_down, 255, 0);
  lv_obj_set_style_bg_opa(s_setting_clock_btn_down, 255, LV_STATE_DISABLED);
  lv_obj_set_style_opa(s_setting_clock_btn_down, 255, 0);
  lv_obj_set_style_opa(s_setting_clock_btn_down, 255, LV_STATE_DISABLED);
  lv_obj_set_style_shadow_width(s_setting_clock_btn_down, 0, 0);
  lv_obj_set_style_shadow_width(s_setting_clock_btn_down, 0, LV_STATE_DISABLED);
  lv_obj_set_style_border_width(s_setting_clock_btn_down, 0, 0);
  lv_obj_set_style_border_width(s_setting_clock_btn_down, 0, LV_STATE_DISABLED);
  lv_obj_add_event_cb(s_setting_clock_btn_down, clock_down_event_cb,
                      LV_EVENT_ALL, NULL);

  s_setting_line_clock_dn = lv_line_create(s_setting_clock_btn_down);
  lv_obj_set_size(s_setting_line_clock_dn, 60, 60);
  lv_line_set_points(s_setting_line_clock_dn, points_dn, 3);
  lv_obj_set_style_line_width(s_setting_line_clock_dn, 6, 0);
  lv_obj_set_style_line_color(s_setting_line_clock_dn,
                              lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_line_rounded(s_setting_line_clock_dn, true, 0);
  lv_obj_align(s_setting_line_clock_dn, LV_ALIGN_CENTER, 0, 0);

  s_setting_circ_clock_dn = lv_obj_create(s_setting_clock_btn_down);
  lv_obj_set_size(s_setting_circ_clock_dn, 26, 26);
  lv_obj_set_style_radius(s_setting_circ_clock_dn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(s_setting_circ_clock_dn, 0, 0);
  lv_obj_set_style_border_width(s_setting_circ_clock_dn, 4, 0);
  lv_obj_set_style_border_color(s_setting_circ_clock_dn,
                                lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_pad_all(s_setting_circ_clock_dn, 0, 0);
  lv_obj_center(s_setting_circ_clock_dn);
  lv_obj_add_flag(s_setting_circ_clock_dn, LV_OBJ_FLAG_HIDDEN);

  s_setting_clock_val_label = lv_label_create(s_setting_page1_obj);
  lv_obj_set_style_text_font(s_setting_clock_val_label, &font_kopub_35, 0);
  lv_obj_set_style_text_color(s_setting_clock_val_label,
                              lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_align(s_setting_clock_val_label, LV_ALIGN_TOP_MID, 75, 170);

  s_setting_clock_btn_up = lv_btn_create(s_setting_page1_obj);
  lv_obj_set_size(s_setting_clock_btn_up, 60, 60);
  lv_obj_align(s_setting_clock_btn_up, LV_ALIGN_TOP_MID, 135, 160);
  lv_obj_set_style_radius(s_setting_clock_btn_up, 15, 0);
  lv_obj_set_style_bg_color(s_setting_clock_btn_up, lv_color_hex(0x222222), 0);
  lv_obj_set_style_bg_color(s_setting_clock_btn_up, lv_color_hex(0x555555),
                            LV_STATE_PRESSED);
  lv_obj_set_style_bg_color(s_setting_clock_btn_up, lv_color_hex(0x222222),
                            LV_STATE_DISABLED);
  lv_obj_set_style_bg_opa(s_setting_clock_btn_up, 255, 0);
  lv_obj_set_style_bg_opa(s_setting_clock_btn_up, 255, LV_STATE_DISABLED);
  lv_obj_set_style_opa(s_setting_clock_btn_up, 255, 0);
  lv_obj_set_style_opa(s_setting_clock_btn_up, 255, LV_STATE_DISABLED);
  lv_obj_set_style_shadow_width(s_setting_clock_btn_up, 0, 0);
  lv_obj_set_style_shadow_width(s_setting_clock_btn_up, 0, LV_STATE_DISABLED);
  lv_obj_set_style_border_width(s_setting_clock_btn_up, 0, 0);
  lv_obj_set_style_border_width(s_setting_clock_btn_up, 0, LV_STATE_DISABLED);
  lv_obj_add_event_cb(s_setting_clock_btn_up, clock_up_event_cb,
                      LV_EVENT_ALL, NULL);

  s_setting_line_clock_up = lv_line_create(s_setting_clock_btn_up);
  lv_obj_set_size(s_setting_line_clock_up, 60, 60);
  lv_line_set_points(s_setting_line_clock_up, points_up, 3);
  lv_obj_set_style_line_width(s_setting_line_clock_up, 6, 0);
  lv_obj_set_style_line_color(s_setting_line_clock_up,
                              lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_line_rounded(s_setting_line_clock_up, true, 0);
  lv_obj_align(s_setting_line_clock_up, LV_ALIGN_CENTER, 0, 0);

  s_setting_circ_clock_up = lv_obj_create(s_setting_clock_btn_up);
  lv_obj_set_size(s_setting_circ_clock_up, 26, 26);
  lv_obj_set_style_radius(s_setting_circ_clock_up, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_opa(s_setting_circ_clock_up, 0, 0);
  lv_obj_set_style_border_width(s_setting_circ_clock_up, 4, 0);
  lv_obj_set_style_border_color(s_setting_circ_clock_up,
                                lv_color_make(210, 210, 0), 0);
  lv_obj_set_style_pad_all(s_setting_circ_clock_up, 0, 0);
  lv_obj_center(s_setting_circ_clock_up);
  lv_obj_add_flag(s_setting_circ_clock_up, LV_OBJ_FLAG_HIDDEN);

  // 4. Save Button
  s_setting_save_btn = lv_btn_create(s_setting_page1_obj);
  lv_obj_set_size(s_setting_save_btn, 160, 60);
  lv_obj_align(s_setting_save_btn, LV_ALIGN_TOP_MID, 0, 245);
  lv_obj_set_style_radius(s_setting_save_btn, 15, 0);
  lv_obj_set_style_bg_color(s_setting_save_btn, lv_color_hex(0x222222), 0);
  lv_obj_set_style_bg_color(s_setting_save_btn, lv_color_hex(0x555555),
                            LV_STATE_PRESSED);
  lv_obj_set_style_shadow_width(s_setting_save_btn, 0, 0);
  lv_obj_set_style_border_width(s_setting_save_btn, 0, 0);
  lv_obj_add_event_cb(s_setting_save_btn, save_btn_event_cb, LV_EVENT_ALL,
                      NULL);

  s_setting_save_label = lv_label_create(s_setting_save_btn);
  lv_obj_set_style_text_font(s_setting_save_label, &font_addr_30, 0);
  lv_obj_set_style_text_color(s_setting_save_label,
                              lv_color_make(210, 210, 0), 0);
  lv_label_set_text(s_setting_save_label, "저장");
  lv_obj_center(s_setting_save_label);

  // --- PAGE 2 CONTAINER ---
  s_setting_page2_obj = lv_obj_create(s_setting_screen);
  lv_obj_set_size(s_setting_page2_obj, 460, 360); // Expanded height for lower QR
  lv_obj_align(s_setting_page2_obj, LV_ALIGN_TOP_MID, 0, 99);
  lv_obj_set_style_bg_opa(s_setting_page2_obj, 0, 0);
  lv_obj_set_style_border_width(s_setting_page2_obj, 0, 0);
  lv_obj_set_scrollbar_mode(s_setting_page2_obj, LV_SCROLLBAR_MODE_OFF);
  lv_obj_add_flag(s_setting_page2_obj, LV_OBJ_FLAG_HIDDEN);

  // --- 1. Info Texts Control (5 Rows, Centered Colon Alignment) ---
  const char *keys[] = {"모델명", "S/W 버전", "시리얼 번호", "앱등록 번호", "제품 홈페이지"};
  const char *values[] = {"MOVISION HUD1", FIRMWARE_VERSION, s_device_serial, s_app_reg_num, "https://m.smartstore.naver.com/b2broom"};
  int start_y = 5;
  int row_h = 34; // Spacing adjusted for 5 rows (+4pt total)

  for (int i = 0; i < 5; i++) {
    if (i < 4) {
        // --- Rows 1-4: Label : Value ---
        // Colon (Center Reference - Move Left by 60px)
        lv_obj_t *colon_label = lv_label_create(s_setting_page2_obj);
        lv_obj_set_style_text_font(colon_label, &font_addr_30, 0);
        lv_obj_set_style_text_color(colon_label, lv_color_make(210, 210, 0), 0);
        lv_label_set_text(colon_label, ":");
        lv_obj_align(colon_label, LV_ALIGN_TOP_MID, -60, start_y + (i * row_h));

        // Key (To the left of colon)
        lv_obj_t *key_label = lv_label_create(s_setting_page2_obj);
        lv_obj_set_style_text_font(key_label, &font_addr_30, 0);
        lv_obj_set_style_text_color(key_label, lv_color_make(210, 210, 0), 0);
        lv_label_set_text(key_label, keys[i]);
        lv_obj_align_to(key_label, colon_label, LV_ALIGN_OUT_LEFT_MID, -15, 0);

        // Value (To the right of colon)
        lv_obj_t *val_label = lv_label_create(s_setting_page2_obj);
        lv_obj_set_style_text_font(val_label, &font_addr_30, 0);
        lv_obj_set_style_text_color(val_label, lv_color_make(210, 210, 0), 0);
        lv_label_set_text(val_label, values[i]);
        lv_obj_align_to(val_label, colon_label, LV_ALIGN_OUT_RIGHT_MID, 15, 0);
    } else {
        // --- Row 5: Website URL only (Centered) ---
        lv_obj_t *url_label = lv_label_create(s_setting_page2_obj);
        lv_obj_set_style_text_font(url_label, &lv_font_montserrat_20, 0); // Smaller font for long URL
        lv_obj_set_style_text_color(url_label, lv_color_make(210, 210, 0), 0);
        lv_label_set_text(url_label, values[i]);
        lv_obj_align(url_label, LV_ALIGN_TOP_MID, 0, start_y + (i * row_h) + 5);
    }
  }

  // --- 2. QR Code (Grey on Black theme) ---
  lv_obj_t *qr = lv_qrcode_create(s_setting_page2_obj, 120, lv_color_make(180, 180, 180), lv_color_black());
  if (qr) {
    const char *qr_data = "https://m.smartstore.naver.com/b2broom";
    lv_qrcode_update(qr, qr_data, strlen(qr_data));
    // Adjusted position down to 180 to accommodate wider row spacing
    lv_obj_align(qr, LV_ALIGN_TOP_MID, 0, 180); 
    
    // Clean Grey border for the QR
    lv_obj_set_style_border_color(qr, lv_color_make(180, 180, 180), 0);
    lv_obj_set_style_border_width(qr, 2, 0); 
    lv_obj_set_style_border_side(qr, LV_BORDER_SIDE_FULL, 0);
  }

  // Initial state update
  set_lcd_brightness(s_brightness_level, true);
  update_setting_ui_labels();


  // Title is no longer clickable
  lv_obj_clear_flag(s_setting_title_label, LV_OBJ_FLAG_CLICKABLE);
  // Removed click event to prevent accidental toggles during mode-switch swipes
  // lv_obj_add_event_cb(s_setting_title_label, setting_page_cb, LV_EVENT_CLICKED, NULL);
}

// create_virtual_drive_ui removed

static void create_ota_ui(void) {
  if (s_ota_screen == NULL)
    return;

  // Title
  lv_obj_t *title = lv_label_create(s_ota_screen);
  lv_obj_set_style_text_font(title, &font_kopub_40, 0);
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_label_set_text(title, "OTA Update");
  // Title
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 50);

  // QR Code
  // Size: 150x150, Color: Dark=Black, Light=White
  lv_obj_t *qr =
      lv_qrcode_create(s_ota_screen, 150, lv_color_black(), lv_color_white());
  if (qr) {
    // Set data to Wi-Fi connection string:
    // WIFI:S:<SSID>;T:<WPA|WEP>;P:<password>;;
    char qr_data[128];
    snprintf(qr_data, sizeof(qr_data), "WIFI:S:%s;T:WPA;P:%s;;", WIFI_OTA_SSID,
             WIFI_OTA_PASS);
    lv_qrcode_update(qr, qr_data, strlen(qr_data));
    lv_obj_align(qr, LV_ALIGN_CENTER, 0, -20);

    // Add a white border
    lv_obj_set_style_border_color(qr, lv_color_white(), 0);
    lv_obj_set_style_border_width(qr, 8, 0);
  }

  // IP Info
  lv_obj_t *ip_label = lv_label_create(s_ota_screen);
  lv_obj_set_style_text_font(ip_label, &font_kopub_25, 0);
  lv_obj_set_style_text_color(ip_label, lv_palette_main(LV_PALETTE_CYAN), 0);
  lv_label_set_text(ip_label, "http://192.168.4.1");
  lv_obj_align(ip_label, LV_ALIGN_CENTER, 0, 90);

  // SSID Info
  lv_obj_t *ssid_label = lv_label_create(s_ota_screen);
  lv_obj_set_style_text_font(ssid_label, &font_kopub_25, 0);
  lv_obj_set_style_text_color(ssid_label, lv_color_hex(0xCCCCCC), 0);
  lv_label_set_text_fmt(ssid_label, "SSID: %s", WIFI_OTA_SSID);
  lv_obj_set_style_text_align(ssid_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(ssid_label, LV_ALIGN_BOTTOM_MID, 0, -80);

  // PW Info (Positioned lower)
  lv_obj_t *pw_label = lv_label_create(s_ota_screen);
  lv_obj_set_style_text_font(pw_label, &font_kopub_25, 0);
  lv_obj_set_style_text_color(pw_label, lv_color_hex(0xCCCCCC), 0);
  lv_label_set_text_fmt(pw_label, "PW: %s", WIFI_OTA_PASS);
  lv_obj_set_style_text_align(pw_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(pw_label, LV_ALIGN_BOTTOM_MID, 0, -45);
}

static void scan_intro_images(void) {
  s_image_count = 0;
  ESP_LOGI(TAG, "Album: Scanning for album1~5.jpg in /littlefs/Photo...");

  for (int i = 1; i <= 5; i++) {
    const char *base_paths[] = {"/littlefs/Photo", "/littlefs/photo",
                                "/littlefs/flash_data/Photo", "/littlefs/flash_data/photo"};
    
    for (int p = 0; p < 4; p++) {
      char path1[64], path2[64];
      snprintf(path1, sizeof(path1), "%s/album%d.jpg", base_paths[p], i);
      snprintf(path2, sizeof(path2), "%s/album_%d.jpg", base_paths[p], i);
      
      const char *to_check[] = {path1, path2};
      for (int c = 0; c < 2; c++) {
        struct stat st;
        if (stat(to_check[c], &st) == 0 && S_ISREG(st.st_mode)) {
          if (s_image_count < MAX_IMAGE_FILES) {
            bool duplicate = false;
            for (int k = 0; k < s_image_count; k++) {
              if (strstr(s_image_files[k], to_check[c])) { duplicate = true; break; }
            }
            if (!duplicate) {
              snprintf(s_image_files[s_image_count], sizeof(s_image_files[0]), 
                       "S:%s", to_check[c]);
              s_image_count++;
              ESP_LOGI(TAG, "Album: Added (%d) %s", s_image_count, to_check[c]);
            }
          }
        }
      }
    }
  }

  if (s_image_count == 0) {
    ESP_LOGW(TAG, "Album: No album images found. Fallback to guide.jpg.");
  } else {
    ESP_LOGI(TAG, "Album: Found %d album images.", s_image_count);
  }
}

// UI Objects for OTA and Image Transfer
static lv_obj_t *s_update_bg = NULL;
static lv_obj_t *s_update_label = NULL;
static lv_obj_t *s_update_bar = NULL;

static lv_obj_t *s_img_update_bg = NULL;
static lv_obj_t *s_img_update_label = NULL;
static lv_obj_t *s_img_progress_bar = NULL;

void update_ui_progress(int percent, const char *status) {
  static int last_percent = -1;
  if (percent == last_percent && status == NULL)
    return;
  last_percent = percent;

  LVGL_LOCK();
  // 업데이트 시작 시(0%) 기존 앨범 이미지의 파일 핸들 및 캐시 해제를 통해 안정성 확보
  if (percent == 0) {
      if (s_album_img) lv_img_set_src(s_album_img, NULL);
      if (s_album_gif) lv_gif_set_src(s_album_gif, NULL);
      lv_img_cache_invalidate_src(NULL);
  }
  if (!s_update_bg) {
    s_update_bg = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_update_bg, LCD_H_RES, LCD_V_RES);
    lv_obj_set_style_bg_color(s_update_bg, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_update_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_update_bg, 0, 0);

    s_update_label = lv_label_create(s_update_bg);
    lv_obj_align(s_update_label, LV_ALIGN_CENTER, 0, -30);
    lv_obj_set_style_text_color(s_update_label, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_style_text_font(s_update_label, &font_addr_30, 0);
    lv_label_set_text(s_update_label, status ? status : "OTA 업데이트 진행중..");

    s_update_bar = lv_bar_create(s_update_bg);
    lv_obj_set_size(s_update_bar, 300, 18);
    lv_obj_align(s_update_bar, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(s_update_bar, lv_color_make(64, 64, 64), 0);
    lv_obj_set_style_bg_color(s_update_bar, lv_color_hex(0xFFFF00), LV_PART_INDICATOR);
    lv_bar_set_range(s_update_bar, 0, 100);
  }

  if (s_update_bar) {
    lv_bar_set_value(s_update_bar, percent, LV_ANIM_OFF);
  }
  if (status && s_update_label) {
    lv_label_set_text(s_update_label, status);
  }
  LVGL_UNLOCK();
}

void update_img_transfer_ui(int percent, bool finished) {
  LVGL_LOCK();
  if (finished) {
    if (s_img_update_bg) {
        if (s_img_update_label) lv_obj_add_flag(s_img_update_label, LV_OBJ_FLAG_HIDDEN);
        if (s_img_progress_bar) lv_obj_add_flag(s_img_progress_bar, LV_OBJ_FLAG_HIDDEN);
    }
    LVGL_UNLOCK();
    return;
  }

  // [Fix] 이미지 전송 시작 시 기존 앨범 이미지의 파일 핸들을 명시적으로 해제하여
  // 전송 완료 후 파일 교체(rename/unlink) 시 발생할 수 있는 EBUSY(Has open FD) 에러 방지
  if (percent == 0) {
      if (s_album_img) lv_img_set_src(s_album_img, NULL);
      if (s_album_gif) lv_gif_set_src(s_album_gif, NULL);
      lv_img_cache_invalidate_src(NULL);
  }

  if (!s_img_update_bg) {
    s_img_update_bg = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_img_update_bg, LCD_H_RES, LCD_V_RES);
    lv_obj_set_style_bg_color(s_img_update_bg, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_img_update_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_img_update_bg, 0, 0);

    s_img_update_label = lv_label_create(s_img_update_bg);
    lv_obj_align(s_img_update_label, LV_ALIGN_CENTER, 0, -30);
    lv_obj_set_style_text_color(s_img_update_label, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_style_text_font(s_img_update_label, &font_addr_30, 0);
    lv_label_set_text(s_img_update_label, "사진 전송 중...\n잠시만 기다려 주세요.");

    s_img_progress_bar = lv_bar_create(s_img_update_bg);
    lv_obj_set_size(s_img_progress_bar, 300, 20);
    lv_obj_align(s_img_progress_bar, LV_ALIGN_CENTER, 0, 60);
    lv_obj_set_style_bg_color(s_img_progress_bar, lv_color_make(64, 64, 64), 0);
    lv_obj_set_style_bg_color(s_img_progress_bar, lv_color_hex(0xFFFF00), LV_PART_INDICATOR);
    lv_bar_set_range(s_img_progress_bar, 0, 100);
  } else {
    // 연속 전송 시: 이전에 숨겨두었던 텍스트와 바를 다시 보이게 함
    if (s_img_update_label) lv_obj_clear_flag(s_img_update_label, LV_OBJ_FLAG_HIDDEN);
    if (s_img_progress_bar) lv_obj_clear_flag(s_img_progress_bar, LV_OBJ_FLAG_HIDDEN);
  }


  if (s_img_progress_bar) {
    lv_bar_set_value(s_img_progress_bar, percent, LV_ANIM_OFF);
  }
  LVGL_UNLOCK();
}

static void reset_album_to_default_image(void) {
  if (s_image_count == 0) {
    scan_intro_images();
    if (s_image_count == 0)
      return;
  }

  s_current_image_index = 0;
  bool found = false;
  for (int i = 0; i < s_image_count; i++) {
    // Extract filename from path
    const char *fname = strrchr(s_image_files[i], '/');
    if (fname)
      fname++;
    else
      fname = s_image_files[i];

    ESP_LOGI(TAG, "Checking image [%d]: %s (extracted: %s)", i,
             s_image_files[i], fname);

    if (strcasecmp(fname, "album_1.png") == 0) {
      s_current_image_index = i;
      found = true;
      ESP_LOGI(TAG, "Default image found: album_1.png at index %d", i);
      break;
    }
  }

  // Fallback to toy_car.jpg if album_1.png not found
  if (!found) {
    for (int i = 0; i < s_image_count; i++) {
        const char *fname = strrchr(s_image_files[i], '/');
        if (fname) fname++; else fname = s_image_files[i];
        if (strcasecmp(fname, "toy_car.jpg") == 0) {
            s_current_image_index = i;
            found = true;
            ESP_LOGI(TAG, "Default image found: toy_car.jpg at index %d", i);
            break;
        }
    }
  }

  if (!found) {
    ESP_LOGI(TAG, "Selected first available image as default: %s",
             s_image_files[0]);
  }

  load_image_from_sd(0);
}

void load_image_from_sd(int direction) {
  LVGL_LOCK();
  // Reset auto timer whenever an image is loaded (either auto or manual)
  s_album_auto_timer = 0;

  // If no images scannned yet (or count 0), try scanning
  if (s_image_count == 0) {
    scan_intro_images();
    if (s_image_count == 0) {
      LVGL_UNLOCK();
      return; // Still empty
    }
    s_current_image_index = 0;
  }

  // Initialize index if invalid
  if (s_current_image_index < 0)
    s_current_image_index = 0;

  if (direction != 0) {
    s_current_image_index += direction;
  }

  // Wrap around logic
  if (s_current_image_index >= s_image_count) {
    s_current_image_index = 0;
  } else if (s_current_image_index < 0) {
    s_current_image_index = s_image_count - 1;
  }

  const char *filename = s_image_files[s_current_image_index];
  ESP_LOGI(TAG, "Album: Loading (%d/%d) %s", s_current_image_index + 1,
           s_image_count, filename);

  // Invalidate image cache for this specific source to avoid stale/corrupted rendering
  // (Using NULL invalidates everything, which is safer when files might have been updated)
  lv_img_cache_invalidate_src(NULL);

  // Check file extension
  const char *ext = strrchr(filename, '.');
  bool is_gif = (ext && strcasecmp(ext, ".gif") == 0);

  // Clean up previous GIF if exists
  if (s_album_gif) {
    lv_obj_del(s_album_gif);
    s_album_gif = NULL;
  }

  if (is_gif) {
#if LV_USE_GIF
    // Hide static image object when playing GIF
    if (s_album_img) {
      lv_obj_add_flag(s_album_img, LV_OBJ_FLAG_HIDDEN);
    }

    s_album_gif = lv_gif_create(s_album_screen);
    if (s_album_gif) {
      lv_gif_set_src(s_album_gif, filename);
      lv_obj_center(s_album_gif);
    } else {
      ESP_LOGE(TAG, "Album: Failed to create GIF object for %s", filename);
    }
#endif
  } else {
    // Normal Image
    if (s_album_img) {
      lv_img_set_src(s_album_img, filename);
      lv_obj_clear_flag(s_album_img, LV_OBJ_FLAG_HIDDEN);
      
      // Optimization: if it's a big image, we might want to suggest a refresh
      // but lv_timer_handler will pick it up on next cycle.
    } else {
      ESP_LOGE(TAG, "Album: s_album_img is NULL!");
    }
  }
  LVGL_UNLOCK();
}

// ---------------------------------------------------------------------------
// Touch Driver Implementation (CST92xx I2C)
// ---------------------------------------------------------------------------
static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_touch_dev_handle = NULL;

static esp_err_t init_touch(void) {
  i2c_master_bus_config_t i2c_bus_conf = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .scl_io_num = PIN_TOUCH_SCL,
      .sda_io_num = PIN_TOUCH_SDA,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  esp_err_t ret = i2c_new_master_bus(&i2c_bus_conf, &s_i2c_bus_handle);
  if (ret != ESP_OK)
    return ret;

  i2c_device_config_t dev_conf = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = TOUCH_I2C_ADDR,
      .scl_speed_hz = TOUCH_I2C_FREQ_HZ,
  };
  ret = i2c_master_bus_add_device(s_i2c_bus_handle, &dev_conf,
                                  &s_touch_dev_handle);
  if (ret != ESP_OK)
    return ret;

  gpio_config_t rst_gpio_conf = {.mode = GPIO_MODE_OUTPUT,
                                 .pin_bit_mask = 1ULL << PIN_TOUCH_RST};
  gpio_config(&rst_gpio_conf);
  gpio_set_level(PIN_TOUCH_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(PIN_TOUCH_RST, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Configure Touch INT pin as input
  gpio_config_t int_gpio_conf = {.mode = GPIO_MODE_INPUT,
                                 .pin_bit_mask = 1ULL << PIN_TOUCH_INT,
                                 .pull_up_en = GPIO_PULLUP_ENABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&int_gpio_conf);

  ESP_LOGI(TAG, "Touch initialization successful");
  return ESP_OK;
}

static void touch_read_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  static int start_x = -1;
  static int start_y = -1;
  static bool swiped = false;

  // Read buffer size: points * 5 + 5 overhead (safe size 20)
  uint8_t read_buf[20] = {0};
  uint8_t write_buf[3] = {0};

  // Reverted INT check to polling for better sensitivity (in case of missed
  // pulses)
  // 1. Read Command 0xD000
  write_buf[0] = (CST92XX_READ_COMMAND >> 8) & 0xFF;
  write_buf[1] = CST92XX_READ_COMMAND & 0xFF;

  // Reduced timeout or silent fail to avoid log spam on occasional glich
  if (i2c_master_transmit_receive(s_touch_dev_handle, write_buf, 2, read_buf,
                                  sizeof(read_buf),
                                  pdMS_TO_TICKS(50)) != ESP_OK) {
    // ESP_LOGW("TOUCH", "I2C Read Failed"); // Optional: uncomment if needed
    data->state = LV_INDEV_STATE_REL;
    start_x = -1;
    start_y = -1;
    swiped = false;
    return;
  }

  // 2. Send Handshake ACK: 0xD0 0x00 0xAB
  write_buf[2] = CST92XX_ACK;
  i2c_master_transmit(s_touch_dev_handle, write_buf, 3, pdMS_TO_TICKS(50));

  // 3. Verify Device ACK (Index 6)
  if (read_buf[6] != CST92XX_ACK) {
    // ESP_LOGW("TOUCH", "Invalid ACK: 0x%02X", read_buf[6]);
    data->state = LV_INDEV_STATE_REL;
    start_x = -1;
    start_y = -1;
    swiped = false;
    return;
  }

  // 4. Parse Data
  uint8_t point_count = read_buf[5] & 0x0F;

  if (point_count > 0 && point_count <= CST92XX_MAX_FINGER_NUM) {
    // Parse Point 0 (Index 0..4)
    // Structure: [id:4][pressed:4] [x_high] [y_high] [x_low:4][y_low:4]
    uint8_t pressed = read_buf[0] & 0x0F;
    if (pressed == 0x06 || pressed == 0x01 || pressed == 0x03) {
      uint16_t x = ((read_buf[1] << 4) | (read_buf[3] >> 4));
      uint16_t y = ((read_buf[2] << 4) | (read_buf[3] & 0x0F));

      data->state = LV_INDEV_STATE_PR;
      // Flip X and Y coordinates to resolve inverted touch input
      data->point.x = (LCD_H_RES - 1 - x);
      data->point.y = (LCD_V_RES - 1 - y);

      ESP_LOGI("TOUCH", "Touch: raw_x=%d, raw_y=%d -> mapped_x=%d, mapped_y=%d",
               x, y, (int)data->point.x, (int)data->point.y);

      // Swipe detection
      if (start_x == -1) {
        start_x = x;
        start_y = y;
        swiped = false;
      } else if (!swiped) {
        int dx = x - start_x;
        int dy = y - start_y;

        // Horizontal Swipe (Mode Change) detection
        if (abs(dx) > abs(dy) && abs(dx) > 40) {
          // [User Request] Ignore ALL touch inputs in Virtual Drive mode
          if (s_virt_drive_active) {
              ESP_LOGI("TOUCH", "Horizontal touch ignored in Virtual Drive mode");
              swiped = true;
          } else {
            ESP_LOGI("TOUCH", "HORIZONTAL SWIPE: dx=%d dy=%d", dx, dy);
            // OTA 모드에서는 가로 스와이프 무시 (실수 방지)
            if (s_current_mode == DISPLAY_MODE_OTA) {
            swiped = true; // 다산, 메세지만 더이상 발생 안 함
            // nothing
          } else {
            // OTA는 실수 방지를 위해 스와이프 순환에서 제외
            // 우→좌(Next):
            // Horizontal Swipe Loop: GUIDE <-> CLOCK <-> ALBUM <-> SETTING
            int next_mode;
            if (dx > 0) { // Right to Left (Next)
              switch (s_current_mode) {
              case DISPLAY_MODE_GUIDE:   next_mode = DISPLAY_MODE_CLOCK; break;
              case DISPLAY_MODE_CLOCK:   next_mode = DISPLAY_MODE_ALBUM; break;
              case DISPLAY_MODE_ALBUM:   next_mode = DISPLAY_MODE_SETTING; break;
              case DISPLAY_MODE_SETTING: next_mode = DISPLAY_MODE_GUIDE; break;
              default:                   next_mode = DISPLAY_MODE_GUIDE; break;
              }
            } else { // Left to Right (Prev)
              switch (s_current_mode) {
              case DISPLAY_MODE_GUIDE:   next_mode = DISPLAY_MODE_SETTING; break;
              case DISPLAY_MODE_SETTING: next_mode = DISPLAY_MODE_ALBUM; break;
              case DISPLAY_MODE_ALBUM:   next_mode = DISPLAY_MODE_CLOCK; break;
              case DISPLAY_MODE_CLOCK:   next_mode = DISPLAY_MODE_GUIDE; break;
              default:                   next_mode = DISPLAY_MODE_GUIDE; break;
              }
            }

            if (next_mode == DISPLAY_MODE_ALBUM && s_current_mode != DISPLAY_MODE_ALBUM) {
              reset_album_to_default_image();
            }
            s_is_manual_mode_switch = true;
            switch_display_mode(next_mode);
            s_is_manual_mode_switch = false;
            swiped = true;
          } // end else (not OTA)
          }
        }
        // Vertical Swipe
        else if (abs(dy) > abs(dx) && abs(dy) > 60) {
          // [User Request] Ignore touch inputs in Virtual Drive mode
          if (s_virt_drive_active) {
              ESP_LOGI("TOUCH", "Vertical touch ignored in Virtual Drive mode");
              swiped = true;
          } else if (s_current_mode == DISPLAY_MODE_GUIDE) {
            // [Integrated] Disable vertical swipe in Guide Mode to prevent accidental changes while driving
            ESP_LOGI("TOUCH", "Vertical swipe disabled in GUIDE mode");
            swiped = true; 
          } else if (s_current_mode == DISPLAY_MODE_ALBUM) {
            // Album Image Change
            if (dy < 0) load_image_from_sd(1);
            else {
              load_image_from_sd(-1);
            }
            swiped = true;
          } else if (s_current_mode == DISPLAY_MODE_CLOCK) {
            // Toggle between Clock 1 and Clock 2
            s_clock_option = (s_clock_option == 0) ? 1 : 0;
            ESP_LOGI("TOUCH", "Clock option toggled via vertical swipe: %d", s_clock_option);
            s_is_manual_mode_switch = true;
            switch_display_mode(DISPLAY_MODE_CLOCK);
            s_is_manual_mode_switch = false;
            swiped = true;
          } else if (s_current_mode == DISPLAY_MODE_BOOT) {
            // [Secret Trigger] 5 vertical swipes in 10s enters Virtual Drive
            uint32_t now = xTaskGetTickCount();
            if (s_secret_swipe_count == 0 || (now - s_secret_swipe_start_tick) > pdMS_TO_TICKS(10000)) {
                s_secret_swipe_count = 1;
                s_secret_swipe_start_tick = now;
                ESP_LOGI("TOUCH", "Secret Trigger: Swipe 1/5 detected (10s timer started)");
            } else {
                s_secret_swipe_count++;
                ESP_LOGI("TOUCH", "Secret Trigger: Swipe %d/5 detected", s_secret_swipe_count);
                if (s_secret_swipe_count >= 5) {
                    ESP_LOGW("TOUCH", "SECRET TRIGGER ACTIVATED! Entering Virtual Drive...");
                    toggle_virtual_drive(true);
                    s_secret_swipe_count = 0;
                }
            }
            swiped = true;
          } else if (s_current_mode == DISPLAY_MODE_SETTING) {
            setting_page_cb(NULL);
            swiped = true;
          } else if (s_current_mode == DISPLAY_MODE_OTA) {
            s_is_manual_mode_switch = true;
            switch_display_mode(DISPLAY_MODE_SETTING);
            s_is_manual_mode_switch = false;
            swiped = true;
          }
        }
      }
      return;
    }
  }

  // No valid touch
  data->state = LV_INDEV_STATE_REL;
  start_x = -1;
  start_y = -1;
  swiped = false;
}


static void show_restart_msg(void) {
  LVGL_LOCK();
  lv_obj_t *scr = lv_obj_create(lv_layer_top());
  lv_obj_set_size(scr, LCD_H_RES, LCD_V_RES);
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(scr, 0, 0);

  lv_obj_t *label = lv_label_create(scr);
  lv_obj_set_style_text_font(label, &font_addr_30, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(0xFFFF00), 0); // Yellow
  lv_label_set_text(label, "시스템을 재부팅합니다!");
  lv_obj_center(label);
  LVGL_UNLOCK();
}

static void delayed_reboot_task(void *pvParameter) {
  ESP_LOGI(TAG, "Reboot task started: Waiting for 2.0s of BLE inactivity...");
  
  while (1) {
      uint32_t current_tick = xTaskGetTickCount();
      // note_ble_activity()가 업데이트하는 마지막 수신 시간을 기준으로 2초(2000ms) 경과 확인
      uint32_t elapsed_ms = (current_tick - s_last_ble_activity_tick) * portTICK_PERIOD_MS;
      
      // 이미지 전송 중에는 2초 비활성 체크를 skip하지만, 타이머 값은 여기서 보지 않고 
      // note_ble_activity()가 호출되는 한 안전함.
      if (!s_img_transfer_active && elapsed_ms >= 2000) {
          ESP_LOGW(TAG, "No activity for 2.0s. Showing restart message...");
          show_restart_msg();
          
          // 시스템을 시원하게 재부팅하기 전에 문자열이 화면에 그려지는 것을 보장(1초 여유)
          vTaskDelay(pdMS_TO_TICKS(1000));
          
          ESP_LOGW(TAG, "Executing scheduled restart NOW.");
          esp_restart();
      }
      
      // 0.1초마다 대기하며 갱신 여부 체크
      vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void start_reboot_task(void) {
  static bool reboot_started = false;
  if (!reboot_started) {
    reboot_started = true;
    xTaskCreate(delayed_reboot_task, "reboot_task", 4096, NULL, 1, NULL);
  }
}

void app_main(void) {
  uint32_t intro_start_time = 0;
  bool intro_playing = false;

  // 0. Rescue mode check (BOOT 버튼을 누른 채 부팅하면 Factory 모드로 강제
  // 진입)
  gpio_reset_pin(GPIO_NUM_0);
  gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);
  if (gpio_get_level(GPIO_NUM_0) == 0) {
    // 로그 초기화가 아직 안 되었을 수 있으므로 기본 로그 사용
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGW(TAG, "Rescue button detected! Booting to Factory partition...");
    const esp_partition_t *factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (factory) {
      esp_ota_set_boot_partition(factory);
      esp_restart();
    } else {
      ESP_LOGE(TAG, "Factory partition NOT FOUND!");
    }
  }

  // 로그 레벨 설정: 에러만 표시하고 시스템
  // 모니터링만 정보 표시
  esp_log_level_set("*", ESP_LOG_ERROR);
  esp_log_level_set(SYS_MON_TAG, ESP_LOG_NONE);
  esp_log_level_set(TAG, ESP_LOG_INFO);
  esp_log_level_set("OTA_BLE", ESP_LOG_INFO);
  esp_log_level_set("WIFI_OTA", ESP_LOG_INFO);
  esp_log_level_set("OTA_SD", ESP_LOG_INFO); // Enable OTA logs
  esp_log_level_set("TOUCH", ESP_LOG_INFO);
  esp_log_level_set("IMG_TRANSFER", ESP_LOG_INFO); // 이미지 전송 디버그 로그 활성화
  esp_log_level_set(
      "i2c.master",
      ESP_LOG_NONE); // Suppress I2C driver error logs (touch polling nack)

  // Print reset reason to diagnose double
  // boot issue
  esp_reset_reason_t reset_reason = esp_reset_reason();
  const char *reset_reason_str;
  switch (reset_reason) {
  case ESP_RST_POWERON:
    reset_reason_str = "POWERON";
    break;
  case ESP_RST_EXT:
    reset_reason_str = "EXT";
    break;
  case ESP_RST_SW:
    reset_reason_str = "SW";
    break;
  case ESP_RST_PANIC:
    reset_reason_str = "PANIC";
    break;
  case ESP_RST_INT_WDT:
    reset_reason_str = "INT_WDT";
    break;
  case ESP_RST_TASK_WDT:
    reset_reason_str = "TASK_WDT";
    break;
  case ESP_RST_WDT:
    reset_reason_str = "WDT";
    break;
  case ESP_RST_DEEPSLEEP:
    reset_reason_str = "DEEPSLEEP";
    break;
  case ESP_RST_BROWNOUT:
    reset_reason_str = "BROWNOUT";
    break;
  case ESP_RST_SDIO:
    reset_reason_str = "SDIO";
    break;
  default:
    reset_reason_str = "UNKNOWN";
    break;
  }
  ESP_LOGI(TAG, "Reset reason: %s (%d)", reset_reason_str, reset_reason);
  
  s_boot_tick = xTaskGetTickCount(); // Record boot time for 20s monitor
  
  // [Fix] 전력 관리 비활성화 (클럭 지터 및 절전으로 인한 BLE 타임아웃 방지)
#if CONFIG_PM_ENABLE
  esp_pm_config_esp32s3_t pm_config = {
      .max_freq_mhz = 240,
      .min_freq_mhz = 240,
      .light_sleep_enable = false,
  };
  esp_pm_configure(&pm_config);
  ESP_LOGI(TAG, "Power Management set to High-Performance (Fixed 240MHz)");
#endif

  // [Fix] NVS 초기화 및 설정을 UI 생성보다 먼저 수행하여 시리얼 번호(s_device_serial) 등이 올바르게 표시되도록 합니다.
  esp_err_t ret_nvs = nvs_flash_init();
  if (ret_nvs == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret_nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  } else {
    ESP_ERROR_CHECK(ret_nvs);
  }
  load_nvs_settings(); 

  const esp_app_desc_t *app_desc = esp_app_get_description();
  ESP_LOGI(TAG, "Firmware Version (Base): %s", app_desc->version);
  ESP_LOGI(TAG, "Build Version (Dynamic): %s", FIRMWARE_VERSION);
  ESP_LOGI(TAG, "Project Name: %s", app_desc->project_name);
  ESP_LOGI(TAG, "Compile Time: %s %s", app_desc->date, app_desc->time);

  ESP_LOGI(TAG, "Booting BLE-only...");

  // Set default virtual time: 2026-01-10 12:15:30
  struct tm default_time = {0};
  default_time.tm_year = 2026 - 1900;
  default_time.tm_mon = 0; // January
  default_time.tm_mday = 10;
  default_time.tm_hour = 12;
  default_time.tm_min = 15;
  default_time.tm_sec = 30;
  default_time.tm_isdst = -1;
  struct timeval tv_default = {0};
  tv_default.tv_sec = mktime(&default_time);
  settimeofday(&tv_default, NULL);
  ESP_LOGI(TAG, "Virtual time set to default: 2026-01-10 12:15:30");

  // 1. Initialize LittleFS first (so labels and images can be loaded during
  // UI init)
  esp_err_t lfs_ret = init_littlefs();
  if (lfs_ret != ESP_OK) {
    ESP_LOGW(TAG, "LittleFS is not available: %s", esp_err_to_name(lfs_ret));
  } else {
    ESP_LOGI(TAG, "LittleFS mounted successfully.");
    // esp_littlefs_info is skipped to reduce boot delay.
  }

  // Start LittleFS UART console (ls / cat / stat / df commands in monitor)

  // [Fix] NVS에서 불러온 모드를 잠시 백업하고, 부팅 과정(인트로/설치안내)을 위해 강제로 BOOT 모드로 시작합니다.
  // 이 설정을 lcd_init_panel 이전에 함으로써 초기 UI 타이머가 대기화면 라벨을 띄우는 것을 방지합니다.
  s_current_mode = DISPLAY_MODE_BOOT;

  // 2. Initialize LCD & LVGL (Moved before SD for Update UI)
  ESP_LOGI(TAG, "[LCD] Step 1: Initializing hardware panel...");
  esp_err_t ret = lcd_init_panel();
  set_lcd_brightness(s_brightness_level, true); // [User Request] Use restored NVS level

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "[LCD] Error: Failed to initialize panel");
  } else {
    ESP_LOGI(TAG, "[LCD] Step 2: Initializing LVGL graphic engine...");
    ret = lvgl_init();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "[LVGL] Error: Failed to initialize engine");
    } else {
      ESP_LOGI(TAG, "[LVGL] Step 3: Creating screen objects and buffers...");
      // Create Queues
      s_image_update_queue = xQueueCreate(100, sizeof(image_update_request_t));
      if (!s_image_update_queue)
        ESP_LOGE(TAG, "Failed to create image update queue");

      s_safety_update_queue =
          xQueueCreate(100, sizeof(safety_update_request_t));
      if (!s_safety_update_queue)
        ESP_LOGE(TAG, "Failed to create safety update queue");

      s_speed_update_queue = xQueueCreate(50, sizeof(speed_update_request_t));
      if (!s_speed_update_queue)
        ESP_LOGE(TAG, "Failed to create speed update queue");

      s_destination_update_queue =
          xQueueCreate(100, sizeof(destination_update_request_t));
      if (!s_destination_update_queue)
        ESP_LOGE(TAG, "Failed to create destination update queue");

      s_circle_update_queue = xQueueCreate(50, sizeof(circle_update_request_t));
      if (!s_circle_update_queue)
        ESP_LOGE(TAG, "Failed to create circle update queue");

      s_clear_display_queue = xQueueCreate(50, sizeof(clear_display_request_t));
      if (!s_clear_display_queue)
        ESP_LOGE(TAG, "Failed to create clear display queue");

      s_road_name_update_queue =
          xQueueCreate(10, sizeof(road_name_update_request_t));
      if (!s_road_name_update_queue)
        ESP_LOGE(TAG, "Failed to create road name update queue");

      // Start LVGL Handler Task for Update UI
      s_lvgl_task_handle = NULL;
      // Increased stack size for stability during heavy UI/SD operations
      xTaskCreate(lvgl_handler_task, "lvgl_handler", 9216, NULL, 4,
                  &s_lvgl_task_handle);

      if (s_intro_image) {
        const char *intro_gif_paths[] = {"/littlefs/intro.gif", "/littlefs/flash_data/intro.gif"};
        const char *intro_lv_paths[] = {"S:/littlefs/intro.gif", "S:/littlefs/flash_data/intro.gif"};
        for (int i = 0; i < 2; i++) {
          struct stat st;
          if (stat(intro_gif_paths[i], &st) == 0) {
            ESP_LOGI(TAG, "System Boot: Intro GIF found at %s. Playing early...", intro_gif_paths[i]);
            LVGL_LOCK();
            if (s_intro_image) lv_obj_del(s_intro_image);
#ifndef LV_USE_GIF
#define LV_USE_GIF 1
#endif
#if LV_USE_GIF
            s_intro_image = lv_gif_create(s_boot_screen);
            if (s_intro_image) {
              ESP_LOGI(TAG, "[DISPLAY] Step 4: Loading Intro GIF and switching to BOOT screen");
              lv_gif_set_src(s_intro_image, intro_lv_paths[i]);
              lv_obj_clear_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN);
              lv_obj_center(s_intro_image);
              lv_obj_move_foreground(s_intro_image);
              lv_gif_set_loop_count(s_intro_image, 1);
              lv_scr_load(s_boot_screen); // Ensure boot screen is active for intro
              LVGL_UNLOCK();
              // [User Request] 설정 메뉴의 밝기 값을 보존하기 위해 하드웨어 밝기만 직접 올립니다.
              apply_hw_brightness(5); // Turn on backlight for intro only
              ESP_LOGI(TAG, "[DISPLAY] Backlight ON (Level 5)");
              intro_start_time = (uint32_t)(esp_timer_get_time() / 1000);
              intro_playing = true;
            } else {
              LVGL_UNLOCK();
            }
#endif
            break;
          }
        }
      }
    }
  }

  // 3. Firmware Update (no SD check required for internal logic if needed)
  {
    // Pass UI callback if LVGL initialized
    esp_err_t update_ret = check_and_perform_update(update_ui_progress);
    if (update_ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Firmware update scan finished (no update found or error)");
    }
    // Re-scan images from LittleFS
    scan_intro_images();
  }

  // Initialize boot button for mode switching
  esp_err_t btn_ret = init_boot_button();
  if (btn_ret != ESP_OK) {
    ESP_LOGW(TAG,
             "Boot button initialization "
             "failed: %s",
             esp_err_to_name(btn_ret));
  }

  // 앱이 정상 부팅되었음을 마킹 (Rollback 방지)
  esp_ota_mark_app_valid_cancel_rollback();

  ota_ble_init();
  img_transfer_init();
  fw_update_init();

  // Initialize BLE Command Queue and Handler Task
  s_ble_cmd_queue = xQueueCreate(20, sizeof(ble_cmd_t));
  xTaskCreate(ble_cmd_handler_task, "ble_cmd_h", 8192, NULL, 4, &s_ble_cmd_task_handle);

  ESP_ERROR_CHECK(init_ble());

  // Derived from assumption: simple TX task. 6KB is safer.
  xTaskCreate(ble_tx_task, "ble_tx", 6144, NULL, 4, &s_ble_tx_task_handle);
  // xTaskCreate(virtual_drive_task, "virt_drive", 8192, NULL, 5,
  //             &s_virt_drive_task_handle); // Create Virtual Drive Task

  // LittleFS is already mounted at the beginning of app_main.
  // Check if it was successful before loading CSV files.
  if (lfs_ret == ESP_OK) {
    // esp_littlefs_info block removed to avoid locking the filesystem 
    // and causing the LVGL intro GIF to freeze.

    // Load image data CSV file
    ESP_LOGI(TAG, "Attempting to load CSV "
                  "file from littlefs...");
    esp_err_t csv_ret = load_image_data_csv();
    if (csv_ret == ESP_OK) {
      ESP_LOGI(TAG,
               "CSV file loaded "
               "successfully, entries=%zu",
               s_image_data_count);
      if (s_image_data_count == 0) {
        ESP_LOGW(TAG, "WARNING: CSV file loaded "
                      "but contains 0 entries!");
      }
    } else if (csv_ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGE(TAG, "CSV file not found at "
                    "/littlefs/TBT.CSV or "
                    "/littlefs/flash_data/TBT.CSV");
      ESP_LOGE(TAG, "Please ensure TBT.CSV is included "
                    "in the littlefs partition");
    } else if (csv_ret == ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "LittleFS not mounted, "
                    "cannot load CSV file");
    } else {
      ESP_LOGE(TAG,
               "Failed to load image data "
               "CSV: %s (error code: %d)",
               esp_err_to_name(csv_ret), csv_ret);
    }

    // Load Safety_DRV CSV file
    ESP_LOGI(TAG, "Attempting to load Safety_DRV "
                  "CSV file from littlefs...");
    esp_err_t safety_csv_ret = load_safety_data_csv();
    if (safety_csv_ret == ESP_OK) {
      ESP_LOGI(TAG,
               "Safety_DRV CSV file loaded "
               "successfully, entries=%zu",
               s_safety_data_count);
      if (s_safety_data_count == 0) {
        ESP_LOGW(TAG, "WARNING: Safety_DRV CSV file "
                      "loaded but contains 0 entries!");
      }
    } else if (safety_csv_ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGW(TAG, "Safety_DRV CSV file not "
                    "found (optional)");
    } else if (safety_csv_ret == ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG, "LittleFS not mounted, cannot "
                    "load Safety_DRV CSV file");
    } else {
      ESP_LOGE(TAG,
               "Failed to load Safety_DRV CSV: %s "
               "(error code: %d)",
               esp_err_to_name(safety_csv_ret), safety_csv_ret);
    }
    // LittleFS 마운트 후 이미지 경로 초기화
    s_current_image_path[0] = '\0';
  }

  // 1. System Boot: Intro GIF Wait (2~3 seconds total)
  // ---------------------------------------------------------
  if (intro_playing) {
    ESP_LOGI(TAG, "System Boot: Waiting for intro.gif to finish...");
    uint32_t check_ms = 50;
    uint32_t max_ms = 3000;
    while (1) {
      uint32_t current_time = (uint32_t)(esp_timer_get_time() / 1000);
      uint32_t elapsed_ms = 0;
      if (current_time > intro_start_time) {
        elapsed_ms = current_time - intro_start_time;
      }
      if (elapsed_ms >= max_ms) break;

      if (s_hud_seen_first_cmd) {
        ESP_LOGI(TAG, "System Boot: App command received, skipping remaining intro (%u ms elapsed)", (unsigned)elapsed_ms);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(check_ms));
    }
  } else {
    ESP_LOGW(TAG, "System Boot: Intro GIF not found or not played.");
  }

  // [User Request] 인트로 재생 완료 후 NVS 설정값으로 밝기 복구
  set_lcd_brightness(s_brightness_level, true);


  // [Fix] 앱 연결 대기 전 부팅 화면(s_boot_screen)을 활성화하여 10초 후 설치 안내가 보이도록 함
  LVGL_LOCK();
  update_display_mode_ui(s_current_mode);
  LVGL_UNLOCK();

  // 1.5. 앱 연결 대기 루프 (최대 10초 타임아웃은 lvgl_handler_task에서 처리됨)
  // [Fix] iOS 자동 재연결 대응: BLE 연결이 아닌 실제 앱 명령(0x4D) 수신까지 대기
  if (!s_hud_seen_first_cmd) {
    ESP_LOGI(TAG, "Intro playback finished. Waiting for App command before moving from BOOT mode...");
    while (!s_hud_seen_first_cmd) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  // [Fix] 앱 연결 직후 상태 업데이트 (패킷에 의해 이미 모드가 바뀌었을 수 있음)
  if (s_current_mode == DISPLAY_MODE_BOOT) {
    ESP_LOGI(TAG, "App connection detected. Staying in BOOT mode until command received.");
  } else {
    ESP_LOGI(TAG, "App connection detected. Maintaining current active mode (%d).", s_current_mode);
  }
  update_display_mode_ui(s_current_mode);
  LVGL_UNLOCK();

  // 2. HUD Mode Entry: Logo (Removed as per user request)
  // ---------------------------------------------------------
  /*
  LVGL_LOCK();
  s_intro_image = lv_img_create(lv_scr_act());

  if (s_intro_image) {
    const char *logo_paths[] = {
        "/littlefs/flash_data/logo.png", "/littlefs/logo.png",
        "/littlefs/flash_data/intro.png", "/littlefs/intro.png"}; // Fallback
    const char *logo_lv_paths[] = {
        "S:/littlefs/flash_data/logo.png", "S:/littlefs/logo.png",
        "S:/littlefs/flash_data/intro.png", "S:/littlefs/intro.png"};

    bool logo_found = false;
    for (int i = 0; i < 4; i++) {
      struct stat st;
      if (stat(logo_paths[i], &st) == 0) {
        ESP_LOGI(TAG, "HUD Entry: Logo found at %s", logo_paths[i]);
        lv_img_set_src(s_intro_image, logo_lv_paths[i]);

        lv_obj_clear_flag(s_intro_image, LV_OBJ_FLAG_HIDDEN);
        lv_obj_center(s_intro_image);
        lv_obj_move_foreground(s_intro_image);
        logo_found = true;
        break;
      }
    }

    LVGL_UNLOCK();

    if (logo_found) {
      set_lcd_brightness(0);
      ESP_LOGI(TAG, "HUD Entry: Displaying logo.png for 1 second...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      ESP_LOGI(TAG, "HUD Entry: Logo displayed. Waiting for App
  Connection..."); } else { set_lcd_brightness(0, true); LVGL_LOCK();
      lv_obj_del(s_intro_image); // Cleanup if unused
      s_intro_image = NULL;
      LVGL_UNLOCK();
      ESP_LOGW(TAG, "HUD Entry: No logo image found.");
    }
  } else {
    LVGL_UNLOCK();
  }
  */

  // 인트로 종료 후 초기 모드 설정 (HUD 모드) 및 UI 표시
  // switch_display_mode() uses locks internally usually? Check
  // implementation. Assuming switch_display_mode() is task-safe or needs
  // lock. Most likely it needs lock if it touches LVGL objects.
  LVGL_LOCK();
  // 부팅 상황 및 파티션 상태 확인
  const esp_partition_t *running = esp_ota_get_running_partition();
  bool is_factory =
      (running && running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY);

  // 현재 파티션의 OTA 상태 확인 (Rollback 상황인지 판단)
  esp_ota_img_states_t img_state;
  bool state_ret = (esp_ota_get_state_partition(running, &img_state) == ESP_OK);

  // OTA 모드 진입 조건:
  // 1. Factory 파티션이면서 BOOT 버튼이 눌려 있는 경우 (수동 복구)
  // 2. Factory 파티션이면서 Rollback 등에 의해 강제로 돌아온 경우 (자동 복구)
  // 그 외 일반적인 부팅(USB 플래시 포함)은 HUD 모드로 시작
  if (is_factory && (gpio_get_level(GPIO_NUM_0) == 0 ||
                     (state_ret && img_state == ESP_OTA_IMG_PENDING_VERIFY))) {
    ESP_LOGW(TAG, "Recovery conditions met. Starting OTA mode...");
    switch_display_mode(DISPLAY_MODE_OTA);
  } else {
    // [Fix] 상단에서 이미 대기 및 모드 적용이 완료되었으므로 여기서는 중복 처리를 하지 않습니다.
    ESP_LOGI(TAG, "System Boot: Normal boot path verification.");
  }
  LVGL_UNLOCK();

  // LVGL Task already created early
  // s_lvgl_task_handle = NULL;
  // xTaskCreate(lvgl_handler_task, "lvgl_handler", 8192, NULL, 5,
  // &s_lvgl_task_handle);
  // Derived from SYS_MON: Used ~2200. 4KB is safe.
  xTaskCreate(buffer_queue_monitor_task, "buf_queue_monitor", 4096, NULL, 3,
              &s_monitor_task_handle);
  // LCD task is simple queue waiter. 4KB safe.
  xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 4, &s_lcd_task_handle);

  ESP_LOGI(TAG, "Initialization complete");

  // keep app_main alive (so monitor doesn't
  // show "Returned from app_main()")
    while (1) {
        static uint32_t s_clock_auto_timer = 0;

        // 부팅 시 시계 화면 전환 트리거 처리 (이전에는 대기모드로 자동 전환했으나 
        // 사용자 요청으로 자동 전환 로직 제거, 플래그만 초기화함)
        if (s_boot_clock_trigger) {
            s_boot_clock_trigger = false;
            ESP_LOGI(TAG, "Main Task: First clock update received, maintaining current mode %d", s_current_mode);
        }

    // 이미지 전송 완료 비동기 처리 (BLE 태스크 블로킹 방지)
    if (s_img_transfer_finished_flag) {
      s_img_transfer_finished_flag = false;
      ESP_LOGI(TAG, "Main Task: Cleaning up image transfer UI...");
      update_img_transfer_ui(100, true);
    }





    // 시스템 진단 로그 (5초마다)
    static uint32_t s_diag_timer = 0;
    if (++s_diag_timer >= 50) { // 100ms * 50 = 5s
      s_diag_timer = 0;
      ESP_LOGI("DIAG", "Heap: %lu, PSRAM: %lu, Connected: %d", 
               (unsigned long)esp_get_free_heap_size(),
               (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
               s_connected);
    }

    // 1. 앨범 자동 갱신 (10초) - 이미지 전송이나 펌웨어 업데이트 중에는 동작하지 않도록 엄격히 제한
    extern bool s_fw_update_active;
    if (s_album_option == 0 && s_current_mode == DISPLAY_MODE_ALBUM && !s_img_transfer_active && !s_fw_update_active) {
      if (++s_album_auto_timer >= 100) { // 100ms * 100 = 10s
        load_image_from_sd(1); // Next
      }
    } else {
      s_album_auto_timer = 0; // 전송 중이거나 모드가 다르면 항상 0으로 초기화
    }

    // 2. 시계 자동 갱신 (1시간 = 3600초)
    if (s_clock_option == 2 && s_current_mode == DISPLAY_MODE_CLOCK) {
      if (++s_clock_auto_timer >= 3600) {
        s_clock_auto_timer = 0;
        ESP_LOGI(TAG, "Auto Clock Toggle (1 hour)");
        LVGL_LOCK();
        update_display_mode_ui(DISPLAY_MODE_CLOCK);
        LVGL_UNLOCK();
      }
    } else {
      s_clock_auto_timer = 0;
    }

    update_auto_brightness(false);
    img_transfer_check_timeout(); // 이미지 전송 데드락 체크
    vTaskDelay(pdMS_TO_TICKS(100)); // 0.1초 반응성
  }
}

