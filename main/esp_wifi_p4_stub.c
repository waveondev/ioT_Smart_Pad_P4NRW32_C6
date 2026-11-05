
/* SPDX-License-Identifier: Apache-2.0 */
/* ESP32-P4 Dummy Stubs for IDF esp_wifi physical driver functions */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"

#define KEEP_SYMBOL __attribute__((used))

#ifdef __cplusplus
extern "C" {
#endif

// 1. Missing Event Base Definition (보존 속성 부여)
ESP_EVENT_DEFINE_BASE(WIFI_EVENT);

// 2. IDF esp_wifi internal physical driver dummy implementations
KEEP_SYMBOL esp_err_t esp_wifi_internal_set_sta_ip(void) { return ESP_OK; }
KEEP_SYMBOL esp_err_t esp_wifi_internal_reg_netstack_buf_cb(void *cb) { (void)cb; return ESP_OK; }
KEEP_SYMBOL esp_err_t esp_wifi_stop(void) { return ESP_OK; }
KEEP_SYMBOL esp_err_t esp_wifi_internal_free_rx_buffer(void *eb) { (void)eb; return ESP_OK; }
KEEP_SYMBOL esp_err_t esp_wifi_internal_tx(uint8_t ifx, void *buffer, uint16_t len) { (void)ifx; (void)buffer; (void)len; return ESP_OK; }
KEEP_SYMBOL esp_err_t esp_wifi_internal_reg_rxcb(uint8_t ifx, void *fn) { (void)ifx; (void)fn; return ESP_OK; }

// esp_wifi_get_mac() 실제 호출 여부 확인용 진단 로그
KEEP_SYMBOL esp_err_t esp_wifi_get_mac(uint8_t ifx, uint8_t mac[6])
{
    ESP_LOGE("P4_WIFI_STUB",
             "esp_wifi_get_mac CALLED: ifx=%u",
             (unsigned int)ifx);

    (void)mac;
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif
