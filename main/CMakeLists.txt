idf_component_register(
    SRCS "main.c" "ota_sd.c" "ota_wifi.c" "ota_ble.c"

     INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_lcd esp_lcd_sh8601 driver lvgl sdmmc fatfs littlefs app_update esp_partition esp_http_server esp_wifi esp_netif

)

# Define LVGL custom memory allocator functions
target_compile_definitions(${COMPONENT_LIB} PRIVATE 
    -DCONFIG_LV_MEM_CUSTOM_ALLOC=lvgl_psram_malloc
    -DCONFIG_LV_MEM_CUSTOM_FREE=lvgl_psram_free
    -DCONFIG_LV_MEM_CUSTOM_REALLOC=lvgl_psram_realloc
)

