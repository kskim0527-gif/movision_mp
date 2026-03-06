#include "ota_sd.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "OTA_SD";

#define UPDATE_FILENAME "/sdcard/update.bin"
#define UPDATE_FILENAME_DONE "/sdcard/update.bak"

esp_err_t check_and_perform_update(ota_progress_cb_t progress_cb) {
  ESP_LOGI(TAG, "Checking for update file: %s", UPDATE_FILENAME);

  FILE *f = fopen(UPDATE_FILENAME, "rb");
  if (f == NULL) {
    ESP_LOGI(TAG, "No update file found.");
    return ESP_OK; // No update, proceed normally
  }

  update_header_t header;
  if (fread(&header, 1, sizeof(header), f) != sizeof(header)) {
    ESP_LOGE(TAG, "Failed to read header");
    fclose(f);
    return ESP_FAIL;
  }

  if (strncmp(header.magic, HEADER_MAGIC, 4) != 0) {
    ESP_LOGE(TAG, "Invalid header magic: %02x %02x %02x %02x", header.magic[0],
             header.magic[1], header.magic[2], header.magic[3]);
    fclose(f);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Update found! App size: %lu, Storage size: %lu",
           header.app_size, header.storage_size);

  if (progress_cb)
    progress_cb(1, "Update Found...");

  // Unmount LittleFS if it's mounted
  esp_vfs_littlefs_unregister("storage");

  // 1. Update App
  const esp_partition_t *update_partition =
      esp_ota_get_next_update_partition(NULL);
  if (update_partition == NULL) {
    ESP_LOGE(TAG, "No OTA partition found");
    fclose(f);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Writing app to partition subtype %d at offset 0x%lx",
           update_partition->subtype, update_partition->address);

  esp_ota_handle_t update_handle = 0;
  esp_err_t err =
      esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
    fclose(f);
    return err;
  }

  size_t buf_size = 4096;
  char *buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
  if (buf == NULL) {
    ESP_LOGE(TAG, "Failed to allocate buffer");
    fclose(f);
    return ESP_ERR_NO_MEM;
  }

  uint32_t remaining = header.app_size;
  uint32_t total_app = header.app_size;
  while (remaining > 0) {
    size_t to_read = (remaining > buf_size) ? buf_size : remaining;
    if (fread(buf, 1, to_read, f) != to_read) {
      ESP_LOGE(TAG, "Failed to read app data");
      free(buf);
      fclose(f);
      return ESP_FAIL;
    }
    err = esp_ota_write(update_handle, buf, to_read);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
      free(buf);
      fclose(f);
      return err;
    }
    remaining -= to_read;
    if (progress_cb) {
      // Map 0-50% for App Update
      int p = (int)((uint64_t)(total_app - remaining) * 50 / total_app);
      if (p < 1)
        p = 1;
      static int last_p = -1;
      if (p != last_p) {
        progress_cb(p, "Updating App...");
        last_p = p;
      }
    }
    // Feed WDT and allow other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  err = esp_ota_end(update_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_end failed (%s)", esp_err_to_name(err));
    free(buf);
    fclose(f);
    return err;
  }

  err = esp_ota_set_boot_partition(update_partition);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)",
             esp_err_to_name(err));
    free(buf);
    fclose(f);
    return err;
  }

  ESP_LOGI(TAG, "App update successful");

  // 2. Update Storage
  ESP_LOGI(TAG, "Starting storage update...");
  const esp_partition_t *storage_part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
  if (storage_part == NULL) {
    ESP_LOGE(TAG, "Storage partition not found");
    free(buf);
    fclose(f);
    return ESP_FAIL;
  }

  uint32_t erase_size = storage_part->size; // Erase full partition

  ESP_LOGI(TAG, "Erasing storage partition (size: %lu)...", erase_size);

  if (progress_cb)
    progress_cb(50, "Erasing Storage...");

  // Chunky erase to prevent display freeze/glitch
  uint32_t erased = 0;
  // Use 64KB chunks (Block size) typically
  uint32_t chunk_size = 65536;

  while (erased < erase_size) {
    uint32_t len = erase_size - erased;
    if (len > chunk_size)
      len = chunk_size;

    err = esp_partition_erase_range(storage_part, erased, len);
    if (err != ESP_OK) {
      ESP_LOGE(TAG,
               "Failed to erase storage partition range 0x%lx size 0x%lx (%s)",
               erased, len, esp_err_to_name(err));
      free(buf);
      fclose(f);
      return err;
    }
    erased += len;

    // Update progress for erase phase (0-50% relative to erase, mapped to
    // global logic if needed) Here we just keep the message alive and yeld
    if (progress_cb) {
      // Keep it at "Erasing..." phase
      // Optional: You could update percentage like 50 + (erased * 5 /
      // erase_size)
      progress_cb(50 + (erased * 5 / erase_size), "Erasing Storage...");
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Yield to let LVGL update screen safely
  }

  if (progress_cb)
    progress_cb(55, "Writing Storage...");

  remaining = header.storage_size;
  uint32_t total_storage = header.storage_size;
  uint32_t offset = 0;
  while (remaining > 0) {
    size_t to_read = (remaining > buf_size) ? buf_size : remaining;
    if (fread(buf, 1, to_read, f) != to_read) {
      ESP_LOGE(TAG, "Failed to read storage data");
      free(buf);
      fclose(f);
      return ESP_FAIL;
    }
    err = esp_partition_write(storage_part, offset, buf, to_read);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to write storage partition (%s)",
               esp_err_to_name(err));
      free(buf);
      fclose(f);
      return err;
    }
    offset += to_read;
    remaining -= to_read;
    if (progress_cb) {
      // Map 55-95% for Storage Update
      int p = 55 +
              (int)((uint64_t)(total_storage - remaining) * 40 / total_storage);
      static int last_p_storage = -1;
      if (p != last_p_storage) {
        progress_cb(p, "Updating Storage...");
        last_p_storage = p;
      }
    }
    // Feed WDT and allow other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_LOGI(TAG, "Storage update successful. Renaming update file...");

  free(buf);
  fclose(f);

  // Rename file to prevent loop
  struct stat st;
  if (stat(UPDATE_FILENAME_DONE, &st) == 0) {
    ESP_LOGI(TAG, "Removing existing backup: %s", UPDATE_FILENAME_DONE);
    unlink(UPDATE_FILENAME_DONE);
  }

  if (rename(UPDATE_FILENAME, UPDATE_FILENAME_DONE) != 0) {
    ESP_LOGE(
        TAG,
        "Rename failed! Attempting to delete update file to prevent loop.");
    unlink(UPDATE_FILENAME); // Just delete it if rename fails
  } else {
    ESP_LOGI(TAG, "Update file renamed to .bak");
  }

  if (progress_cb)
    progress_cb(100, "Rebooting...");

  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Rebooting system...");
  esp_restart();
  return ESP_OK; // Should not reach here
}
