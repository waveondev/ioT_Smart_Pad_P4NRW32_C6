/* main/rtsp_server.c
 * RTSP server with H.264 Annex-B NAL parsing and RTP-over-TCP FU-A packetization.
 *
 * Notes:
 * - Keeps the existing RTSP endpoint on TCP port 8554.
 * - Supports RTP interleaved channel 0 for the negotiated RTP/AVP/TCP transport.
 * - Splits Annex-B H.264 input into NAL units and fragments large NAL units with FU-A.
 * - This is a baseline implementation; production RTSP servers should also handle
 *   complete RTSP message framing, per-frame timestamps, and concurrent socket teardown.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <limits.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_timer.h"

/* 疫꿸퀣??s_client_sock ?醫롫섧?? ?醫? */
static SemaphoreHandle_t s_rtp_mutex;
static SemaphoreHandle_t s_push_mutex;
static bool s_dummy_push_enabled = true;
static volatile bool s_stream_stop = true;

static const char *TAG = "RTSP_SERVER";

static volatile int s_client_sock = -1;
static uint16_t s_rtp_seq = 0;
static uint32_t s_rtp_timestamp = 0;
static const uint32_t s_rtp_ssrc = 0x12345678;
static bool s_first_rtp_logged = false;
bool rtsp_server_sync_init(void)
{
    bool created_rtp_mutex = false;

    if (s_rtp_mutex == NULL) {
        s_rtp_mutex = xSemaphoreCreateMutex();
        if (s_rtp_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create RTP mutex");
            return false;
        }
        created_rtp_mutex = true;
    }

    if (s_push_mutex == NULL) {
        s_push_mutex = xSemaphoreCreateMutex();
        if (s_push_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create stream push mutex");

            if (created_rtp_mutex) {
                vSemaphoreDelete(s_rtp_mutex);
                s_rtp_mutex = NULL;
            }

            return false;
        }
    }

    return true;
}
/* Max RTP payload bytes. FU-A consumes 2 bytes of this for its headers. */
#define RTP_PAYLOAD_MAX 1400

bool rtsp_server_is_connected(void)
{
    return (s_client_sock >= 0) && !s_stream_stop;
}

#define RTSP_RX_BUFFER_SIZE 2048

typedef struct {
    uint8_t data[RTSP_RX_BUFFER_SIZE];
    size_t used;
    size_t interleaved_remaining;
} rtsp_rx_state_t;

/* Find the end of an RTSP header without assuming recv() boundaries. */
static size_t find_rtsp_header_end(const uint8_t *data, size_t len)
{
    if (len < 4) return 0;

    for (size_t i = 0; i <= len - 4; i++) {
        if (data[i] == '\r' && data[i + 1] == '\n' &&
            data[i + 2] == '\r' && data[i + 3] == '\n') {
            return i + 4;
        }
    }
    return 0;
}

/* Parse Content-Length from the header; absent means an empty body. */
static int parse_content_length(const uint8_t *data, size_t header_len,
                                size_t *content_len)
{
    *content_len = 0;
    size_t pos = 0;

    while (pos + 1 < header_len) {
        size_t end = pos;
        while (end + 1 < header_len &&
               !(data[end] == '\r' && data[end + 1] == '\n')) {
            end++;
        }

        if (end == pos) break;

        static const char key[] = "Content-Length:";
        if (end - pos >= sizeof(key) - 1) {
            bool match = true;
            for (size_t i = 0; i < sizeof(key) - 1; i++) {
                char c = (char)data[pos + i];
                if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
                char k = key[i];
                if (k >= 'a' && k <= 'z') k = (char)(k - 'a' + 'A');
                if (c != k) { match = false; break; }
            }

            if (match) {
                size_t p = pos + sizeof(key) - 1;
                while (p < end && (data[p] == ' ' || data[p] == '\t')) p++;

                size_t value = 0;
                if (p == end) return -1;

                for (; p < end; p++) {
                    if (data[p] < '0' || data[p] > '9') return -1;
                    size_t digit = (size_t)(data[p] - '0');
                    if (value > (SIZE_MAX - digit) / 10) return -1;
                    value = value * 10 + digit;
                }

                *content_len = value;
                return 0;
            }
        }

        pos = end + 2;
    }

    return 0;
}

/*
 * Returns: positive request length, 0 on peer close, -1 on receive/parse error.
 * Interleaved '$' frames are consumed and never passed to the RTSP command parser.
 */
static int recv_rtsp_message(int sock, rtsp_rx_state_t *state,
                             char *out, size_t out_cap)
{
    if (state == NULL || out == NULL || out_cap < 2) {
        errno = EINVAL;
        return -1;
    }

    for (;;) {
        if (state->interleaved_remaining > 0) {
            size_t take = state->used;
            if (take > state->interleaved_remaining) {
                take = state->interleaved_remaining;
            }

            if (take > 0) {
                memmove(state->data, state->data + take, state->used - take);
                state->used -= take;
                state->interleaved_remaining -= take;
                continue;
            }
        }

        if (state->used > 0 && state->data[0] == '$') {
            if (state->used >= 4) {
                size_t frame_len =
                    ((size_t)state->data[2] << 8) | (size_t)state->data[3];

                memmove(state->data, state->data + 4, state->used - 4);
                state->used -= 4;
                state->interleaved_remaining = frame_len;
                continue;
            }
        } else if (state->used > 0) {
            size_t header_len = find_rtsp_header_end(state->data, state->used);

            if (header_len > 0) {
                size_t content_len = 0;
                if (parse_content_length(state->data, header_len,
                                         &content_len) != 0 ||
                    content_len > SIZE_MAX - header_len) {
                    errno = EPROTO;
                    return -1;
                }

                size_t total = header_len + content_len;
                if (total >= out_cap) {
                    errno = EMSGSIZE;
                    return -1;
                }

                if (state->used >= total) {
                    memcpy(out, state->data, total);
                    out[total] = '\0';

                    memmove(state->data, state->data + total,
                            state->used - total);
                    state->used -= total;
                    return (int)total;
                }
            }
        }

        if (state->used == sizeof(state->data)) {
            errno = EMSGSIZE;
            return -1;
        }

        int n = recv(sock, state->data + state->used,
                     sizeof(state->data) - state->used, 0);
        if (n <= 0) return n;

        state->used += (size_t)n;
    }
}
static int parse_cseq(const char *req)
{
    const char *p = strstr(req, "CSeq:");
    if (p) {
        p += 5;
        while (*p == ' ' || *p == '\t') p++;
        return atoi(p);
    }
    return 1;
}

/* Send all bytes on a TCP socket, handling partial writes. */
static int send_all(int sock, const uint8_t *buf, size_t len)
{
    size_t sent = 0;
    unsigned int calls = 0;
    unsigned int partials = 0;
    unsigned int would_block_retries = 0;
    int64_t max_send_us = 0;
    int64_t total_start_us = esp_timer_get_time();

    while (sent < len) {
        int64_t call_start_us = esp_timer_get_time();
        int n = send(sock, buf + sent, len - sent, 0);
        int send_errno = (n < 0) ? errno : 0;
        int64_t call_elapsed_us = esp_timer_get_time() - call_start_us;

        calls++;
        if (call_elapsed_us > max_send_us) {
            max_send_us = call_elapsed_us;
        }

        if (n < 0 && send_errno == EINTR) {
            continue;
        }

        if (n < 0 &&
            (send_errno == EAGAIN || send_errno == EWOULDBLOCK) &&
            would_block_retries < 2) {
            would_block_retries++;
            ESP_LOGW(TAG,
                     "[TCP RETRY] sock=%d errno=%d retry=%u sent=%u/%u",
                     sock, send_errno, would_block_retries,
                     (unsigned int)sent, (unsigned int)len);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (n <= 0) {
            int64_t total_elapsed_us = esp_timer_get_time() - total_start_us;
            ESP_LOGE(TAG,
                     "[TCP ERROR] sock=%d n=%d errno=%d sent=%u/%u "
                     "calls=%u partials=%u max_send=%" PRId64
                     " us total=%" PRId64 " us",
                     sock, n, send_errno,
                     (unsigned int)sent, (unsigned int)len,
                     calls, partials, max_send_us, total_elapsed_us);
            return -1;
        }

        if ((size_t)n < len - sent) {
            partials++;
        }
        sent += (size_t)n;
        would_block_retries = 0;

        /* TCP SLOW log temporarily disabled for latency test.
        if (call_elapsed_us >= 20000) {
            ESP_LOGW(TAG,
                     "[TCP SLOW] sock=%d call=%u n=%d "
                     "call_ms=%" PRId64 " sent=%u/%u",
                     sock, calls, n, call_elapsed_us / 1000,
                     (unsigned int)sent, (unsigned int)len);
        }
        */
    }


    return 0;
}

static int send_rtsp_response(int sock, const char *response)
{
    return send_all(sock, (const uint8_t *)response, strlen(response));
}

/* Send one RTP packet wrapped in RTSP interleaved framing. */
static int send_rtp_payload(int sock,
                            const uint8_t *payload,
                            size_t payload_len,
                            bool marker)
{
    if (sock < 0 || payload == NULL ||
        payload_len == 0 || payload_len > RTP_PAYLOAD_MAX) {
        return -1;
    }

    if (s_rtp_mutex == NULL) {
        return -1;
    }

    /* ???룇 ?怨밴묶?? RTP ?怨밴묶??癰귣똾??*/
    if (xSemaphoreTake(s_rtp_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "[RTP STOP] mutex timeout");
        return -1;
    }

    if (s_stream_stop || s_client_sock != sock) {
        ESP_LOGW(TAG,
                 "[RTP STOP] inactive connection "
                 "tx_sock=%d active_sock=%d stop=%d",
                 sock, s_client_sock, (int)s_stream_stop);

        xSemaphoreGive(s_rtp_mutex);
        return -1;
    }

    uint8_t packet[4 + 12 + RTP_PAYLOAD_MAX];
    const size_t rtp_len = 12 + payload_len;
    const size_t total_len = 4 + rtp_len;

    /* RTSP interleaved frame header */
    packet[0] = '$';
    packet[1] = 0;
    packet[2] = (uint8_t)((rtp_len >> 8) & 0xFF);
    packet[3] = (uint8_t)(rtp_len & 0xFF);

    /* RTP header */
    packet[4] = 0x80;
    packet[5] = (uint8_t)((marker ? 0x80 : 0x00) | 96);
    packet[6] = (uint8_t)((s_rtp_seq >> 8) & 0xFF);
    packet[7] = (uint8_t)(s_rtp_seq & 0xFF);

    packet[8]  = (uint8_t)((s_rtp_timestamp >> 24) & 0xFF);
    packet[9]  = (uint8_t)((s_rtp_timestamp >> 16) & 0xFF);
    packet[10] = (uint8_t)((s_rtp_timestamp >> 8) & 0xFF);
    packet[11] = (uint8_t)(s_rtp_timestamp & 0xFF);

    packet[12] = (uint8_t)((s_rtp_ssrc >> 24) & 0xFF);
    packet[13] = (uint8_t)((s_rtp_ssrc >> 16) & 0xFF);
    packet[14] = (uint8_t)((s_rtp_ssrc >> 8) & 0xFF);
    packet[15] = (uint8_t)(s_rtp_ssrc & 0xFF);

    memcpy(&packet[16], payload, payload_len);

    if (!s_first_rtp_logged) {
        ESP_LOGI(TAG, "[TIME] 筌?RTP ???땅 ?袁⑸꽊 ??뺣즲 sock=%d", sock);
    }

    if (s_rtp_seq < 10 || (s_rtp_seq % 100) == 0) {
        ESP_LOGI(TAG,
                 "[RTP TX] sock=%d seq=%u ts=%" PRIu32
                 " payload=%u total=%u",
                 sock,
                 (unsigned)s_rtp_seq,
                 (uint32_t)s_rtp_timestamp,
                 (unsigned)payload_len,
                 (unsigned)total_len);
    }

    int ret = send_all(sock, packet, total_len);
    int saved_errno = errno;

    if (ret != 0) {
        /*
         * ?袁⑹삺 ?怨뚭퍙????る뻿 ??쎈솭筌??怨뚭퍙 ?怨밴묶??獄쏆꼷???몃빍??
         * ??삘뀲 ?怨뚭퍙?????룇 ?怨밴묶??癰궰野껋?釉?쭪? ??녿뮸??덈뼄.
         */
        if (s_client_sock == sock) {
            s_stream_stop = true;
        }

        ESP_LOGE(TAG,
                 "[RTP ERROR] sock=%d active_sock=%d "
                 "payload=%u total=%u errno=%d",
                 sock,
                 s_client_sock,
                 (unsigned)payload_len,
                 (unsigned)total_len,
                 saved_errno);

        xSemaphoreGive(s_rtp_mutex);
        return -1;
    }

    if (s_stream_stop || s_client_sock != sock) {
        ESP_LOGW(TAG,
                 "[RTP STOP] connection changed after send "
                 "tx_sock=%d active_sock=%d",
                 sock, s_client_sock);

        xSemaphoreGive(s_rtp_mutex);
        return -1;
    }

    if (!s_first_rtp_logged) {
        ESP_LOGI(TAG, "[TIME] 筌?RTP ???땅 ?袁⑸꽊 ?源껊궗 sock=%d", sock);
        s_first_rtp_logged = true;
    }

    s_rtp_seq++;

    xSemaphoreGive(s_rtp_mutex);
    return 0;
}

/* Send one complete H.264 NAL unit; fragment large NALs using RFC 6184 FU-A. */
static int send_one_nal(int sock, const uint8_t *nal, size_t len,
                        bool marker_on_nal)
{
    if (nal == NULL || len == 0) {
        return -1;
    }

    const uint8_t nal_header = nal[0];
    const uint8_t nal_type = nal_header & 0x1F;

    if (len <= RTP_PAYLOAD_MAX) {
        return send_rtp_payload(sock, nal, len, marker_on_nal);
    }

    /* FU-A payload = FU indicator + FU header + fragment data. */
    const uint8_t fu_indicator = (nal_header & 0xE0) | 28;
    const uint8_t fu_header_base = nal_type;
    const size_t fragment_max = RTP_PAYLOAD_MAX - 2;
    size_t offset = 1;
    bool first = true;

    while (offset < len) {
        size_t fragment_len = len - offset;
        if (fragment_len > fragment_max) {
            fragment_len = fragment_max;
        }

        uint8_t fu_payload[RTP_PAYLOAD_MAX];
        fu_payload[0] = fu_indicator;
        fu_payload[1] = fu_header_base;

        if (first) {
            fu_payload[1] |= 0x80;
        }

        const bool last = (offset + fragment_len == len);
        if (last) {
            fu_payload[1] |= 0x40;
        }

        memcpy(&fu_payload[2], nal + offset, fragment_len);

        if (send_rtp_payload(sock, fu_payload, fragment_len + 2,
                             last && marker_on_nal) != 0) {
            return -1;
        }

        offset += fragment_len;
        first = false;
    }

    return 0;
}

static size_t find_start_code(const uint8_t *data, size_t len, size_t pos,
                              size_t *start_code_len)
{
    for (size_t i = pos; i + 3 <= len; ++i) {
        if (i + 4 <= len &&
            data[i] == 0x00 && data[i + 1] == 0x00 &&
            data[i + 2] == 0x00 && data[i + 3] == 0x01) {
            *start_code_len = 4;
            return i;
        }

        if (data[i] == 0x00 && data[i + 1] == 0x00 &&
            data[i + 2] == 0x01) {
            *start_code_len = 3;
            return i;
        }
    }

    *start_code_len = 0;
    return len;
}

/* Public API: accepts an Annex-B byte stream containing one or more NAL units. */

/* Minimal RBSP bit reader for first_mb_in_slice (unsigned Exp-Golomb). */
typedef struct {
    const uint8_t *data;
    size_t len;
    size_t bit_pos;
} rbsp_reader_t;

static bool rbsp_read_bit(rbsp_reader_t *r, uint32_t *bit)
{
    if (r->bit_pos >= r->len * 8) return false;
    size_t byte_pos = r->bit_pos / 8;
    unsigned shift = 7U - (unsigned)(r->bit_pos % 8);
    *bit = (r->data[byte_pos] >> shift) & 1U;
    r->bit_pos++;
    return true;
}

static bool rbsp_read_ue(rbsp_reader_t *r, uint32_t *value)
{
    unsigned zeros = 0;
    uint32_t bit = 0;
    while (true) {
        if (!rbsp_read_bit(r, &bit)) return false;
        if (bit) break;
        if (++zeros > 31) return false;
    }

    uint32_t suffix = 0;
    for (unsigned i = 0; i < zeros; ++i) {
        if (!rbsp_read_bit(r, &bit)) return false;
        suffix = (suffix << 1) | bit;
    }
    *value = ((1U << zeros) - 1U) + suffix;
    return true;
}

static bool get_first_mb_in_slice(const uint8_t *nal, size_t len,
                                  uint32_t *first_mb)
{
    if (!nal || len < 2 || !first_mb) return false;

    /* Remove NAL header and emulation-prevention bytes. */
    uint8_t rbsp[64];
    size_t out = 0;
    unsigned zeros = 0;
    for (size_t i = 1; i < len && out < sizeof(rbsp); ++i) {
        uint8_t b = nal[i];
        if (zeros >= 2 && b == 0x03) {
            continue;
        }
        rbsp[out++] = b;
        zeros = (b == 0) ? zeros + 1 : 0;
    }

    rbsp_reader_t r = { .data = rbsp, .len = out, .bit_pos = 0 };
    return rbsp_read_ue(&r, first_mb);
}

typedef struct {
    size_t offset;
    size_t len;
    uint8_t type;
    bool vcl;
    bool starts_picture;
    bool marker;
} nal_desc_t;

static void rtsp_server_push_nal_internal(uint8_t *data, size_t len)
{
    const int sock = s_client_sock;
    if (sock < 0 || s_stream_stop || data == NULL || len == 0) return;
    if (sock != s_client_sock || s_stream_stop) return;

    size_t sc_len = 0;
    size_t sc_pos = find_start_code(data, len, 0, &sc_len);

    /* Allow a caller to pass one raw NAL without an Annex-B start code. */
    if (sc_len == 0) {
        const uint8_t type = data[0] & 0x1F;
        const bool vcl = (type == 1 || type == 5);

        if (send_one_nal(sock, data, len, vcl) != 0) {
            ESP_LOGW(TAG,
                    "[NAL STOP] type=%u len=%u",
                    (unsigned)type,
                    (unsigned)len);
            return;
        }

        return;
    }

    /*
     * Index NALs first. This lets us mark the last VCL NAL of a picture
     * before transmitting it. Memory usage scales with NAL count.
     */
    size_t capacity = 16, count = 0;
    nal_desc_t *nals = calloc(capacity, sizeof(*nals));
    if (!nals) {
        ESP_LOGE(TAG, "Cannot allocate NAL index");
        return;
    }

    size_t nal_start = sc_pos + sc_len;
    while (nal_start < len) {
        size_t next_sc_len = 0;
        size_t next_sc_pos = find_start_code(data, len, nal_start, &next_sc_len);
        size_t nal_end = next_sc_pos;

        while (nal_end > nal_start && data[nal_end - 1] == 0x00) nal_end--;

        if (nal_end > nal_start) {
            if (count == capacity) {
                size_t new_capacity = capacity * 2;
                nal_desc_t *tmp = realloc(nals, new_capacity * sizeof(*nals));
                if (!tmp) {
                    ESP_LOGE(TAG, "Cannot grow NAL index");
                    free(nals);
                    return;
                }
                nals = tmp;
                capacity = new_capacity;
            }

            nal_desc_t *d = &nals[count++];
            memset(d, 0, sizeof(*d));
            d->offset = nal_start;
            d->len = nal_end - nal_start;
            d->type = data[nal_start] & 0x1F;
            d->vcl = (d->type == 1 || d->type == 5);

            if (d->vcl) {
                uint32_t first_mb = UINT32_MAX;
                d->starts_picture =
                    get_first_mb_in_slice(data + d->offset, d->len, &first_mb)
                    && first_mb == 0;
            }
        }

        if (next_sc_len == 0) break;
        nal_start = next_sc_pos + next_sc_len;
    }

    if (count == 0) {
        free(nals);
        return;
    }

    /*
     * For this progressive test stream, first_mb_in_slice == 0 marks the
     * start of a picture. Mark only the final VCL NAL in each picture.
     */
    size_t last_vcl = SIZE_MAX;
    for (size_t i = 0; i < count; ++i) {
        if (!nals[i].vcl) continue;

        if (nals[i].starts_picture && last_vcl != SIZE_MAX) {
            nals[last_vcl].marker = true;
        }
        last_vcl = i;
    }
    if (last_vcl != SIZE_MAX) nals[last_vcl].marker = true;

    /*
     * Keep one RTP timestamp for all slices of a picture. The first picture
     * uses the current timestamp; increment 3000 ticks at later picture starts.
     */
    bool timestamp_started = false;

    for (size_t i = 0; i < count; ++i) {
        if (s_stream_stop || s_client_sock != sock) {
            ESP_LOGW(TAG,
                     "[NAL STOP] connection changed index=%u sock=%d active=%d",
                     (unsigned)i, sock, (int)s_client_sock);
            free(nals);
            return;
        }

        nal_desc_t *d = &nals[i];

        if (d->vcl && d->starts_picture) {
            if (timestamp_started) {
                /*
                * 30 fps 疫꿸퀣?:
                * ?袁⑥쟿?????????33ms ??疫꿸퀬釉??                * RTP timestamp??3000 tick 筌앹빓???쀪텚??
                * H.264 RTP clock rate??90000Hz????
                */
                vTaskDelay(pdMS_TO_TICKS(5));
                s_rtp_timestamp += 3000;
            } else {
                timestamp_started = true;
            }
        }

        if (send_one_nal(sock, data + d->offset, d->len, d->marker) != 0) {
            ESP_LOGW(TAG,
                    "[NAL STOP] index=%u type=%u len=%u "
                    "starts_picture=%d marker=%d ts=%" PRIu32,
                    (unsigned)i,
                    (unsigned)d->type,
                    (unsigned)d->len,
                    (int)d->starts_picture,
                    (int)d->marker,
                    (uint32_t)s_rtp_timestamp);
            free(nals);
            return;
        }
    }

    free(nals);

    static unsigned push_count = 0;
    push_count++;
    if ((push_count % 30) == 0) {
        ESP_LOGI(TAG, "Annex-B stream push calls: %u", push_count);
    }
}

void rtsp_server_push_nal(uint8_t *data, size_t len)
{
    if (s_push_mutex == NULL) {
        ESP_LOGW(TAG, "Stream push mutex is not ready");
        return;
    }

    if (xSemaphoreTake(s_push_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Stream push mutex acquisition failed");
        return;
    }

    rtsp_server_push_nal_internal(data, len);

    xSemaphoreGive(s_push_mutex);
}
void rtsp_server_push_camera_nal(uint8_t *data, size_t len)
{
    if (s_push_mutex == NULL) {
        ESP_LOGW(TAG, "Stream push mutex is not ready");
        return;
    }

    if (xSemaphoreTake(s_push_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Stream push mutex acquisition failed for camera");
        return;
    }

    rtsp_server_push_nal_internal(data, len);
    s_dummy_push_enabled = false;

    xSemaphoreGive(s_push_mutex);
}

void rtsp_server_set_dummy_enabled(bool enabled)
{
    if (s_push_mutex == NULL) {
        ESP_LOGE(TAG, "Cannot change dummy state: push mutex is not ready");
        return;
    }

    if (xSemaphoreTake(s_push_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire push mutex for dummy state change");
        return;
    }

    s_dummy_push_enabled = enabled;

    xSemaphoreGive(s_push_mutex);
}

void rtsp_server_push_dummy_nal(uint8_t *data, size_t len)
{
    if (s_push_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_push_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire push mutex for dummy stream");
        return;
    }

    if (s_dummy_push_enabled) {
        rtsp_server_push_nal_internal(data, len);
    }

    xSemaphoreGive(s_push_mutex);
}
void rtsp_server_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "RTSP server task started; waiting for network readiness.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1) {
        int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

        if (listen_sock < 0) {
            ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        int opt = 1;
        (void)setsockopt(
            listen_sock, SOL_SOCKET, SO_REUSEADDR,
            &opt, sizeof(opt));

        struct sockaddr_in server_addr = {0};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(8554);
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(listen_sock,
                 (struct sockaddr *)&server_addr,
                 sizeof(server_addr)) < 0) {
            ESP_LOGE(TAG, "bind port 8554 failed: errno=%d", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        if (listen(listen_sock, 5) < 0) {
            ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
            close(listen_sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "RTSP server listening on TCP port 8554.");

        while (1) {
            struct sockaddr_in client_addr = {0};
            socklen_t addr_len = sizeof(client_addr);

            int sock = accept(
                listen_sock,
                (struct sockaddr *)&client_addr,
                &addr_len);

            if (sock < 0) {
                ESP_LOGE(TAG, "accept() failed: errno=%d", errno);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            int nodelay = 1;
            int nd_ret = setsockopt(
                sock, IPPROTO_TCP, TCP_NODELAY,
                &nodelay, sizeof(nodelay));
            int nd_errno = (nd_ret < 0) ? errno : 0;
            ESP_LOGI(TAG,
                     "[SOCKET OPT] TCP_NODELAY ret=%d errno=%d",
                     nd_ret, nd_errno);

            /*
             * send()揶쎛 ?얜똾釉????疫꿸퀬釉?쭪? ??낅즲嚥???쀫립??몃빍??
             * 1??????살첒揶쎛 獄쏆뮇源??롢늺 ??る뻿??餓λ쵎???랁??類ｂ봺??몃빍??
             */
            struct timeval send_timeout = {
                .tv_sec = 1,
                .tv_usec = 0
            };

            int to_ret = setsockopt(
                sock, SOL_SOCKET, SO_SNDTIMEO,
                &send_timeout, sizeof(send_timeout));
            int to_errno = (to_ret < 0) ? errno : 0;
            ESP_LOGI(TAG,
                     "[SOCKET OPT] SO_SNDTIMEO ret=%d errno=%d",
                     to_ret, to_errno);

            char client_ip[16] = {0};
            (void)inet_ntoa_r(
                client_addr.sin_addr,
                client_ip,
                sizeof(client_ip));

            ESP_LOGI(TAG, "RTSP client connected: %s:%u",
                     client_ip,
                     (unsigned)ntohs(client_addr.sin_port));

            bool client_playing = false;
            char buffer[2048];
            rtsp_rx_state_t rx_state = {0};

            while (1) {
                int len = recv_rtsp_message(sock, &rx_state,
                                            buffer, sizeof(buffer));

                if (len <= 0) {
                    int recv_errno = errno;

                    if (len == 0) {
                        ESP_LOGW(TAG, "RTSP peer closed connection");
                    } else {
                        ESP_LOGW(TAG,
                                 "RTSP receive/parser failed: errno=%d",
                                 recv_errno);
                    }

                    break;
                }

                ESP_LOGI(TAG, "[RTSP RX] len=%d: %.512s", len, buffer);

                int cseq = parse_cseq(buffer);

                if (strstr(buffer, "OPTIONS")) {
                    char resp[320];

                    snprintf(resp, sizeof(resp),
                             "RTSP/1.0 200 OK\r\n"
                             "CSeq: %d\r\n"
                             "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n"
                             "\r\n",
                             cseq);

                    if (send_rtsp_response(sock, resp) != 0) {
                        ESP_LOGW(TAG, "OPTIONS response failed");
                        break;
                    }

                    ESP_LOGI(TAG, "OPTIONS CSeq=%d", cseq);
                }
                else if (strstr(buffer, "DESCRIBE")) {
                    const char *sdp =
                        "v=0\r\n"
                        "o=- 0 0 IN IP4 0.0.0.0\r\n"
                        "s=Waveon RTSP Stream\r\n"
                        "c=IN IP4 0.0.0.0\r\n"
                        "t=0 0\r\n"
                        "m=video 0 RTP/AVP 96\r\n"
                        "a=rtpmap:96 H264/90000\r\n"
                        "a=fmtp:96 packetization-mode=1;"
                        "sprop-parameter-sets=Z0LAH9oBQBbsBEAAAAMAQAAADyPGDKg=,aM4PyA==\r\n"
                        "a=control:track0\r\n";

                    char resp[768];

                    snprintf(resp, sizeof(resp),
                             "RTSP/1.0 200 OK\r\n"
                             "CSeq: %d\r\n"
                             "Content-Type: application/sdp\r\n"
                             "Content-Length: %u\r\n"
                             "\r\n%s",
                             cseq,
                             (unsigned)strlen(sdp),
                             sdp);

                    if (send_rtsp_response(sock, resp) != 0) {
                        ESP_LOGW(TAG, "DESCRIBE response failed");
                        break;
                    }

                    ESP_LOGI(TAG, "DESCRIBE CSeq=%d", cseq);
                }
                else if (strstr(buffer, "SETUP")) {
                    char *transport = strstr(buffer, "Transport:");
                    if (transport == NULL || strstr(transport, "RTP/AVP/TCP") == NULL) {
                        char reject[192];
                        snprintf(reject, sizeof(reject),
                                 "RTSP/1.0 461 Unsupported Transport\r\n"
                                 "CSeq: %d\r\n"
                                 "\r\n",
                                 cseq);
                        if (send_rtsp_response(sock, reject) != 0) {
                            ESP_LOGW(TAG, "Unsupported Transport response failed");
                            break;
                        }
                        ESP_LOGW(TAG, "SETUP rejected: unsupported Transport");
                        continue;
                    }
                    char resp[384];

                    snprintf(resp, sizeof(resp),
                             "RTSP/1.0 200 OK\r\n"
                             "CSeq: %d\r\n"
                             "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n"
                             "Session: 12345678\r\n"
                             "\r\n",
                             cseq);

                    if (send_rtsp_response(sock, resp) != 0) {
                        ESP_LOGW(TAG, "SETUP response failed");
                        break;
                    }

                    ESP_LOGI(TAG, "SETUP CSeq=%d", cseq);
                }
                else if (strstr(buffer, "PLAY")) {
                    char resp[320];

                    snprintf(resp, sizeof(resp),
                             "RTSP/1.0 200 OK\r\n"
                             "CSeq: %d\r\n"
                             "Session: 12345678\r\n"
                             "\r\n",
                             cseq);

                    ESP_LOGI(TAG, "[TIME] PLAY request");

                    /*
                     * PLAY ?臾먮뼗???믪눘? ?袁⑸꽊??몃빍??
                     * ?袁⑹춦 RTP ??る뻿?? ??뽮쉐?酉釉?쭪? ??녿뮸??덈뼄.
                     */
                    if (send_rtsp_response(sock, resp) != 0) {
                        ESP_LOGE(TAG, "PLAY response failed");
                        break;
                    }

                    /*
                     * ?怨뚭퍙 ?怨밴묶?? RTP ?怨밴묶????ｍ뜞 癰궰野껋?鍮??덈뼄.
                     */
                    if (xSemaphoreTake(
                            s_rtp_mutex,
                            pdMS_TO_TICKS(1000)) != pdTRUE) {
                        ESP_LOGE(TAG, "RTP mutex timeout on PLAY");
                        break;
                    }

                    if (s_client_sock >= 0 &&
                        s_client_sock != sock) {
                        xSemaphoreGive(s_rtp_mutex);

                        ESP_LOGW(TAG,
                                 "Another RTP client is active");
                        break;
                    }

                    s_rtp_seq = 0;
                    s_rtp_timestamp = 0;
                    s_first_rtp_logged = false;

                    s_client_sock = sock;
                    s_stream_stop = false;
                    client_playing = true;

                    xSemaphoreGive(s_rtp_mutex);

                    ESP_LOGI(TAG,
                             "PLAY CSeq=%d; RTP-over-TCP enabled",
                             cseq);
                }
                else if (strstr(buffer, "TEARDOWN")) {
                    char resp[256];

                    snprintf(resp, sizeof(resp),
                             "RTSP/1.0 200 OK\r\n"
                             "CSeq: %d\r\n"
                             "Session: 12345678\r\n"
                             "\r\n",
                             cseq);

                    /*
                     * ?믪눘? ??쎈뱜??餓λ쵎????遺욧퍕??몃빍??
                     */
                    s_stream_stop = true;
                    client_playing = false;

                    /*
                     * ?臾먮뼗??癰귣?沅→??遺욧퍕 ?룐뫂遊썹몴??ル굝利??몃빍??
                     */
                    (void)send_rtsp_response(sock, resp);

                    ESP_LOGI(TAG, "TEARDOWN CSeq=%d", cseq);
                    break;
                }
                else {
                    ESP_LOGW(TAG, "Unsupported RTSP request: %.120s", buffer);
                    char resp[192];

                    snprintf(resp, sizeof(resp),
                             "RTSP/1.0 501 Not Implemented\r\n"
                             "CSeq: %d\r\n"
                             "\r\n",
                             cseq);

                    if (send_rtsp_response(sock, resp) != 0) {
                        ESP_LOGW(TAG, "RTSP response failed");
                        break;
                    }
                }
            }

            /*
             * ?怨뚭퍙 ?ル굝利??⑤벏??筌ｌ꼶??
             * 1. ??る뻿 餓λ쵎???遺욧퍕
             * 2. ???????룇??????곴맒 ??뽮쉐 ???룇??곗쨮 ?醫뤾문??? 筌륁궢釉?칰???             * 3. 疫꿸퀣????る뻿??mutex???蹂?뱽 ???돱筌왖 疫꿸퀡?롧뵳?             * 4. ???룇 ?ル굝利?             */
            s_stream_stop = true;

            if (xSemaphoreTake(
                    s_rtp_mutex,
                    pdMS_TO_TICKS(2000)) == pdTRUE) {

                if (s_client_sock == sock) {
                    s_client_sock = -1;
                }

                /*
                 * mutex??癰귣똻????怨밴묶?癒?퐣 shutdown??뤿연
                 * ?????룇???????롫뮉 RTP ??る뻿????녿┛?酉鍮??덈뼄.
                 */
                shutdown(sock, SHUT_RDWR);

                xSemaphoreGive(s_rtp_mutex);

                close(sock);

                ESP_LOGI(TAG,
                         "RTSP client socket closed: sock=%d",
                         sock);
            } else {
                /*
                 * mutex ??얜굣 ??쎈솭 ??close()??揶쏅벤六??롢늺
                 * ??る뻿 ??뽯뮞??? 野껋?鍮??????됱몵沃샕嚥?嚥≪뮄?뉒몴???ｍ돥??덈뼄.
                 */
                ESP_LOGE(TAG,
                         "Could not acquire RTP mutex; "
                         "socket cleanup deferred: sock=%d",
                         sock);

                /*
                 * ??野껋럥以???類ㅺ맒?怨몄뵥 ?ル굝利뷸에??띯몿???롢늺 ????몃빍??
                 * ??る뻿 筌??됰뗀以??獄?mutex 癰귣똻? ??볦퍢???癒???곷튊 ??몃빍??
                 */
                shutdown(sock, SHUT_RDWR);
                close(sock);

                if (s_client_sock == sock) {
                    s_client_sock = -1;
                }
            }

            (void)client_playing;
        }

        close(listen_sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
