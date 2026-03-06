#pragma once

#include "esp_err.h"

// Wi-Fi AP 설정
#define WIFI_OTA_SSID "Movision-Update"
#define WIFI_OTA_PASS "12345678"
#define WIFI_OTA_CHANNEL 1
#define WIFI_OTA_MAX_CONN 4

/**
 * @brief OTA 업데이트 시작 전 불필요한 태스크 정리
 *
 * main.c에서 구현됨. 업데이트가 실제 시작될 때 호출하여
 * BLE, 모니터링 등 불필요한 태스크를 제거하고 메모리를 확보합니다.
 */
void prepare_for_ota(void);

/**
 * @brief Wi-Fi OTA 모드 시작 (AP 모드 + 웹 서버)
 *
 * 1. Wi-Fi를 AP 모드로 초기화합니다.
 * 2. HTTP 웹 서버를 시작합니다.
 * 3. 192.168.4.1로 접속하여 OTA를 진행할 수 있습니다.
 */
void start_wifi_ota_mode(void);

/**
 * @brief Wi-Fi OTA 모드 중지
 *
 * 웹 서버와 Wi-Fi를 중지합니다.
 */
void stop_wifi_ota_mode(void);
