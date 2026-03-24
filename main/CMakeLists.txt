idf_component_register(
    SRCS "img_transfer.c" "main.c" "ota_sd.c" "ota_wifi.c" "ota_ble.c"

     INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_lcd esp_lcd_sh8601 driver lvgl sdmmc fatfs littlefs app_update esp_partition esp_http_server esp_wifi esp_netif

)
