/* main/dummy_camera.c - Verified H.264 Full Stream Streamer */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdbool.h>

/* RTSP camera takeover hook; default preserves the existing fallback stream. */
static volatile bool s_dummy_video_enabled = true;

extern void rtsp_server_set_dummy_enabled(bool enabled);

void dummy_video_set_enabled(bool enabled)
{
    rtsp_server_set_dummy_enabled(enabled);
    s_dummy_video_enabled = enabled;
}

static const char *TAG = "HW_ISP_720P";

extern void rtsp_server_push_dummy_nal(uint8_t *data, size_t len);
extern bool rtsp_server_is_connected(void); // extern 선언 유지

// PC에서 FFmpeg로 생성 후 C 배열로 변환한 test_720p30.h264 원본 바이너리
static const uint8_t raw_h264_stream[] = {
    #include "h264_array.txt"
};

void dummy_video_task(void *pvParameters)
{
    ESP_LOGI(TAG, "H.264 더미 스트림 송출 루프 가동 시작");

    while (1) {
        if (!s_dummy_video_enabled) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (rtsp_server_is_connected()) {
            ESP_LOGI(TAG, "H.264 스트림 전송 시작 (%u Bytes)",
                    (unsigned int)sizeof(raw_h264_stream));

            int64_t push_start_us = esp_timer_get_time();

            rtsp_server_push_dummy_nal(
                (uint8_t *)raw_h264_stream,
                sizeof(raw_h264_stream)
            );

            int64_t push_elapsed_us = esp_timer_get_time() - push_start_us;

            ESP_LOGI(TAG,
                    "[STREAM TIME] bytes=%u elapsed=%" PRId64 " ms",
                    (unsigned int)sizeof(raw_h264_stream),
                    push_elapsed_us / 1000);

            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            ESP_LOGD(TAG, "RTSP 클라이언트 대기 중...");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}