#ifndef OTA_SD_H
#define OTA_SD_H

#include "esp_err.h"

// Callback type for progress updates (0-100 percent)
// status: Current operation (e.g., "Reading...", "Flashing...")
typedef void (*ota_progress_cb_t)(int percent, const char *status);

// Check for update file on SD card and perform update if found.
// Call this after SD card is mounted.
// progress_cb: Optional callback for UI updates.
// Header structure (Shared between SD and WiFi OTA)
#define HEADER_MAGIC "UPGD"

typedef struct {
  char magic[4];
  uint32_t app_size;
  uint32_t storage_size;
  uint32_t reserved;
} update_header_t;

// Check for update file on SD card and perform update if found.
// Call this after SD card is mounted.
// progress_cb: Optional callback for UI updates.
esp_err_t check_and_perform_update(ota_progress_cb_t progress_cb);

#endif // OTA_SD_H
