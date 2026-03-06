#include "ota_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "ota_sd.h"
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

static const char *TAG = "WIFI_OTA";
static httpd_handle_t s_http_server = NULL;
static bool s_wifi_initialized = false;
static TaskHandle_t s_dns_task_handle = NULL;

extern void update_ui_progress(int percent, const char *status);

static const char *ota_index_html =
    "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' "
    "content='width=device-width, initial-scale=1'><title>Mando HUD "
    "Smart Update</title>"
    "<style>body{font-family:sans-serif;text-align:center;padding:15px;"
    "background-color:#f4f7f6;color:#333}"
    ".container{max-width:420px;margin:40px "
    "auto;padding:35px;background:white;border-radius:24px;box-shadow:0 15px "
    "35px rgba(0,0,0,0.1)}"
    "h1{margin-top:0;font-size:26px;color:#2c3e50;margin-bottom:10px}p{color:#"
    "666;font-size:15px;margin-bottom:30px}"
    "input[type='file']{display:block;margin:25px "
    "auto;width:100%;font-size:14px;padding:10px;border:2px dashed "
    "#ddd;border-radius:12px;cursor:pointer}"
    "button{background:#007bff;color:white;border:none;padding:16px;font-size:"
    "18px;font-weight:700;border-radius:12px;cursor:pointer;width:100%;"
    "transition:all 0.3s;box-shadow:0 4px 15px rgba(0,123,255,0.3)}"
    "button:hover{background:#0056b3;transform:translateY(-2px)}button:"
    "disabled{background:#ccc;cursor:not-allowed;box-shadow:none}"
    "#progress{margin-top:30px;padding:15px;border-radius:12px;display:none;"
    "background:#f8f9fa;border:1px solid #ebeef5}"
    ".pro-bg{background:#e9ecef;height:12px;border-radius:6px;margin-top:12px;"
    "overflow:hidden}"
    "#pro-bar{background:linear-gradient(90deg, #007bff, "
    "#00c6ff);width:0%;height:100%;transition:width 0.3s}"
    ".status-msg{margin-top:10px;font-size:15px;font-weight:600}.error{color:#"
    "dc3545}.success{color:#28a745}"
    ".hint{font-size:12px;color:#999;margin-top:20px}</style></head>"
    "<body><div class='container'><h1>Device Update</h1><p>관리자로부터 받은 "
    "업데이트 파일(.bin)을<br>선택 후 아래 버튼을 눌러주세요.</p>"
    "<input type='file' id='file_input' accept='.bin'>"
    "<button id='up_btn' onclick='uploadFile()'>업데이트 시작 (Start "
    "Update)</button>"
    "<div id='progress'>"
    "<div id='status' class='status-msg'>준비 중...</div>"
    "<div class='pro-bg'><div id='pro-bar'></div></div>"
    "</div>"
    "<div class='hint'>※ 업데이트 도중 전원을 끄지 마세요.</div></div>"
    "<script>function uploadFile(){"
    "var f=document.getElementById('file_input').files[0];if(!f){alert('파일을 "
    "선택해주세요!');return;}"
    "var btn=document.getElementById('up_btn');var "
    "pDiv=document.getElementById('progress');"
    "var bar=document.getElementById('pro-bar');var "
    "st=document.getElementById('status');"
    "btn.disabled=true;pDiv.style.display='block';st.innerText='장치 연결 "
    "중...';st.className='status-msg';"
    "var x=new "
    "XMLHttpRequest();x.timeout=1200000;x.open('POST','/update',true);"
    "x.upload.onprogress=function(e){if(e.lengthComputable){var "
    "p=Math.floor((e.loaded/e.total)*100);"
    "bar.style.width=p+'%';st.innerText='업데이트 진행 중: '+p+'%';}};"
    "x.onload=function(){if(x.status==200){st.innerText='업데이트 성공! 잠시 "
    "후 재부팅됩니다.';st.className='status-msg success';"
    "bar.style.background='#28a745';setTimeout(function(){location.reload();},"
    "6000);}"
    "else{st.innerText='오류 발생: '+x.responseText;st.className='status-msg "
    "error';btn.disabled=false;}};"
    "x.onerror=function(){st.innerText='네트워크 오류가 "
    "발생했습니다.';st.className='status-msg error';btn.disabled=false;};"
    "x.send(f);}</script></body></html>";

static esp_err_t index_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, ota_index_html, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t reboot_post_handler(httpd_req_t *req) {
  httpd_resp_sendstr(req, "Rebooting...");
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_restart();
  return ESP_OK;
}

static esp_err_t http_404_error_handler(httpd_req_t *req,
                                        httpd_err_code_t err) {
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

typedef enum {
  OTA_STEP_HEADER,
  OTA_STEP_APP,
  OTA_STEP_STORAGE,
  OTA_STEP_FINISH
} ota_step_t;

static esp_err_t update_post_handler(httpd_req_t *req) {
  esp_wifi_set_ps(WIFI_PS_NONE);
  vTaskDelay(pdMS_TO_TICKS(100));
  prepare_for_ota();

  const size_t buf_size = 4096;
  char *buf = heap_caps_malloc(buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!buf) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  esp_ota_handle_t update_handle = 0;
  const esp_partition_t *update_partition = NULL;
  const esp_partition_t *storage_partition = NULL;
  update_header_t header = {0};

  ota_step_t step = OTA_STEP_HEADER;
  uint32_t header_got = 0;
  uint32_t app_written = 0, storage_written = 0;
  uint32_t app_erased = 0, storage_erased = 0;
  uint32_t app_limit = 0, storage_limit = 0;

  int remaining = req->content_len;
  int total_size = remaining;
  int last_progress = -1;
  esp_err_t err = ESP_OK;

  ESP_LOGI(TAG, "Smart OTA Start: %d bytes", total_size);

  while (remaining > 0) {
    int ret = httpd_req_recv(req, buf, MIN(remaining, (int)buf_size));
    if (ret <= 0) {
      if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        vTaskDelay(1);
        continue;
      }
      err = ESP_FAIL;
      break;
    }
    remaining -= ret;
    char *ptr = buf;
    int chunk_len = ret;

    while (chunk_len > 0) {
      if (step == OTA_STEP_HEADER) {
        int to_copy = MIN((int)sizeof(header) - (int)header_got, chunk_len);
        memcpy(((char *)&header) + header_got, ptr, to_copy);
        header_got += to_copy;
        ptr += to_copy;
        chunk_len -= to_copy;

        if (header_got == sizeof(header)) {
          if (strncmp(header.magic, HEADER_MAGIC, 4) == 0) {
            app_limit = header.app_size;
            storage_limit = header.storage_size;
            ESP_LOGI(TAG, "Mode: Combined Package (App:%u, Storage:%u)",
                     app_limit, storage_limit);
          } else {
            if (total_size <= 2100000) { // App max is ~2MB
              ESP_LOGI(TAG, "Mode: Smart Detection - Firmware Only (App)");
              app_limit = total_size;
              storage_limit = 0;
            } else {
              ESP_LOGI(TAG, "Mode: Smart Detection - Data Only (Storage)");
              app_limit = 0;
              storage_limit = total_size;
            }

            if (app_limit > 0) {
              update_partition = esp_ota_get_next_update_partition(NULL);
              if (!update_partition) {
                err = ESP_FAIL;
                break;
              }
              err = esp_ota_begin(update_partition, 4096, &update_handle);
              if (err != ESP_OK)
                break;
              err = esp_ota_write(update_handle, &header, sizeof(header));
              if (err != ESP_OK)
                break;
              app_written = sizeof(header);
              app_erased = 4096;
            }
          }
          step = OTA_STEP_APP;
        }
        continue;
      }

      if (step == OTA_STEP_APP) {
        uint32_t to_process = MIN((uint32_t)chunk_len, app_limit - app_written);
        if (to_process > 0) {
          if (!update_handle) {
            update_partition = esp_ota_get_next_update_partition(NULL);
            if (!update_partition) {
              err = ESP_FAIL;
              break;
            }
            err = esp_ota_begin(update_partition, 4096, &update_handle);
            if (err != ESP_OK)
              break;
            app_erased = 4096;
          }
          while (app_erased < app_written + to_process) {
            err = esp_partition_erase_range(update_partition, app_erased, 4096);
            if (err != ESP_OK)
              break;
            app_erased += 4096;
            vTaskDelay(1);
          }
          if (err != ESP_OK)
            break;
          err = esp_ota_write(update_handle, ptr, to_process);
          if (err != ESP_OK)
            break;
          app_written += to_process;
          ptr += to_process;
          chunk_len -= to_process;
        }
        if (app_written >= app_limit)
          step = OTA_STEP_STORAGE;
        continue;
      }

      if (step == OTA_STEP_STORAGE) {
        uint32_t to_process =
            MIN((uint32_t)chunk_len, storage_limit - storage_written);
        if (to_process > 0) {
          if (!storage_partition) {
            storage_partition = esp_partition_find_first(
                ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
            if (!storage_partition) {
              err = ESP_FAIL;
              break;
            }
          }
          while (storage_erased < storage_written + to_process) {
            err = esp_partition_erase_range(storage_partition, storage_erased,
                                            4096);
            if (err != ESP_OK)
              break;
            storage_erased += 4096;
            vTaskDelay(1);
          }
          if (err != ESP_OK)
            break;
          err = esp_partition_write(storage_partition, storage_written, ptr,
                                    to_process);
          if (err != ESP_OK)
            break;
          storage_written += to_process;
          ptr += to_process;
          chunk_len -= to_process;
        }
        if (storage_written >= storage_limit)
          step = OTA_STEP_FINISH; // Not in enum but effectively ends
        continue;
      }
      chunk_len = 0;
    }
    if (err != ESP_OK)
      break;

    int progress =
        ((total_size - remaining) * 100) / (total_size > 0 ? total_size : 1);
    if (progress != last_progress) {
      last_progress = progress;
      ESP_LOGI(TAG, "Progress: %d%%", progress);
      update_ui_progress(progress, "Wi-Fi Updating...");
    }
  }

  free(buf);
  if (remaining > 0 || err != ESP_OK) {
    if (update_handle)
      esp_ota_abort(update_handle);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Update Failed");
    return ESP_FAIL;
  }

  if (update_handle) {
    esp_ota_end(update_handle);
    esp_ota_set_boot_partition(update_partition);
  }

  ESP_LOGI(TAG, "OTA Success!");
  httpd_resp_sendstr(req, "Success");
  vTaskDelay(pdMS_TO_TICKS(1000));
  esp_restart();
  return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id,
                               void *data) {
  if (id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
    ESP_LOGI(TAG, "station join, AID=%d", e->aid);
  } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
    ESP_LOGI(TAG, "station leave");
  }
}

static void dns_server_task(void *pv) {
  uint8_t d[128];
  struct sockaddr_in s, c;
  socklen_t sl = sizeof(c);
  int sk = socket(AF_INET, SOCK_DGRAM, 0);
  memset(&s, 0, sizeof(s));
  s.sin_family = AF_INET;
  s.sin_addr.s_addr = htonl(INADDR_ANY);
  s.sin_port = htons(53);
  bind(sk, (struct sockaddr *)&s, sizeof(s));
  ESP_LOGI(TAG, "DNS Server started");
  while (1) {
    int n = recvfrom(sk, d, sizeof(d), 0, (struct sockaddr *)&c, &sl);
    if (n > 0) {
      d[2] = 0x81;
      d[3] = 0x80;
      d[7] = 1;
      int idx = 12;
      while (idx < n && d[idx] != 0)
        idx += d[idx] + 1;
      idx += 5;
      d[idx++] = 0xC0;
      d[idx++] = 0x0C;
      d[idx++] = 0;
      d[idx++] = 1;
      d[idx++] = 0;
      d[idx++] = 1;
      d[idx++] = 0;
      d[idx++] = 0;
      d[idx++] = 0;
      d[idx++] = 60;
      d[idx++] = 0;
      d[idx++] = 4;
      d[idx++] = 192;
      d[idx++] = 168;
      d[idx++] = 4;
      d[idx++] = 1;
      sendto(sk, d, idx, 0, (struct sockaddr *)&c, sl);
    }
    vTaskDelay(10);
  }
}

void start_wifi_ota_mode(void) {
  ESP_LOGI(TAG, "Starting Wi-Fi OTA...");
  if (!s_wifi_initialized) {
    esp_netif_init();
    esp_event_loop_create_default();
    if (esp_netif_get_handle_from_ifkey("WIFI_AP_DEF") == NULL)
      esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, NULL);
    wifi_config_t wc = {0};
    strncpy((char *)wc.ap.ssid, WIFI_OTA_SSID, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = strlen(WIFI_OTA_SSID);
    strncpy((char *)wc.ap.password, WIFI_OTA_PASS, sizeof(wc.ap.password));
    wc.ap.channel = 6;
    wc.ap.max_connection = WIFI_OTA_MAX_CONN;
    wc.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wc);
    esp_wifi_start();
    s_wifi_initialized = true;
  }
  if (!s_dns_task_handle)
    xTaskCreate(dns_server_task, "dns", 3072, NULL, 5, &s_dns_task_handle);
  if (!s_http_server) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 5;
    config.lru_purge_enable = true;
    config.stack_size = 8192;
    if (httpd_start(&s_http_server, &config) == ESP_OK) {
      httpd_uri_t u1 = {"/", HTTP_GET, index_get_handler, NULL};
      httpd_uri_t u2 = {"/update", HTTP_POST, update_post_handler, NULL};
      httpd_uri_t u3 = {"/reboot", HTTP_POST, reboot_post_handler, NULL};
      httpd_register_uri_handler(s_http_server, &u1);
      httpd_register_uri_handler(s_http_server, &u2);
      httpd_register_uri_handler(s_http_server, &u3);
      httpd_register_err_handler(s_http_server, HTTPD_404_NOT_FOUND,
                                 http_404_error_handler);
    }
  }
  ESP_LOGI(TAG, "OTA Ready! SSID: %s", WIFI_OTA_SSID);
}

void stop_wifi_ota_mode(void) {
  if (s_http_server) {
    httpd_stop(s_http_server);
    s_http_server = NULL;
  }
  if (s_dns_task_handle) {
    vTaskDelete(s_dns_task_handle);
    s_dns_task_handle = NULL;
  }
  if (s_wifi_initialized)
    esp_wifi_stop();
}
