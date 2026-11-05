/* C6 main/main.c - ESP-Hosted Coprocessor Network Split Trace Added */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Coprocessor 전용 헤더
#include "eh_cp.h"

static const char *TAG = "MAIN_C6";

// 🌟 Wi-Fi AP 및 비밀번호 설정 (기존 유지)
#define EXAMPLE_ESP_WIFI_SSID      "WAVEON_2.4G"
#define EXAMPLE_ESP_WIFI_PASS      "1qaz2wsx^^"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10

static int s_retry_num = 0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi STA 시작됨. AP 접속 시도 중... (SSID: %s)", EXAMPLE_ESP_WIFI_SSID);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* evt = (wifi_event_sta_disconnected_t*) event_data;
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "AP 접속 재시도 (%d/%d), Reason Code: %u", s_retry_num, EXAMPLE_ESP_MAXIMUM_RETRY, evt->reason);
        } else {
            ESP_LOGE(TAG, "AP 접속 최종 실패. AP 이름 및 비밀번호를 확인하세요.");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "========================================================");
        ESP_LOGI(TAG, " [C6 SUCCESS] AP 연결 완료! 할당된 IP 주소: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "========================================================");
        s_retry_num = 0;
    }
}

static void wifi_init_sta(void)
{
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta 가동 완료.");
}

void app_main(void)
{
    uint32_t t_c6_start = esp_log_timestamp();
    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, "[C6 T+%lu ms] C6 Coprocessor Firmware Start!", (unsigned long)t_c6_start);
    ESP_LOGI(TAG, "========================================================");

    // 🌟 [기존 로그 설정 100% 유지]
    esp_log_level_set("ehcp_rpc", ESP_LOG_VERBOSE);
    esp_log_level_set("SDIO_SLAVE", ESP_LOG_VERBOSE);
    esp_log_level_set("ehcp_data", ESP_LOG_VERBOSE);
    esp_log_level_set("ehcp_pcom", ESP_LOG_VERBOSE);

    // 🌟 [신규 추가] Network Split 패킷 포워딩 및 라우팅 판별 디버그 로그
    esp_log_level_set("nw_split_router", ESP_LOG_VERBOSE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());

    // Wi-Fi 접속 가동
    wifi_init_sta();

    ESP_LOGI(TAG, "[C6 T+%lu ms] SDIO Slave Ready. Waiting for Host RPC Packets...", (unsigned long)esp_log_timestamp());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}