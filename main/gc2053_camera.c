#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_cam_sensor_xclk.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "esp_timer.h"

#include "esp_video_device.h"
#include "esp_video_device_internal.h"
#include "esp_video_csi_format.h"
#include "linux/videodev2.h"
#include "esp_video_ioctl.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

static const char *TAG = "GC2053_DETECT";

/*
 * Board evidence:
 *   ???�??smart_DOG_main_SCH_1006(2).pdf:
 *     Sheet 2 & Sheet 6:
 *       GPIO21 -> CAM_PWREN (Camera LDO EN: UC1, UC2, UC3)
 *       GPIO41 -> CAM_RST1 (Camera Hardware Reset, Active-Low)
 *       GPIO39 -> CSI_MCLK1 (Camera XCLK 24MHz)
 *       GPIO42 -> CAM_SDA1 (R20)
 *       GPIO43 -> CAM_SCL1 (R22)
 *
 * Bus number I2C_NUM_0 is a provisional software choice, not a verified
 * schematic assignment. Before enabling this task, check that no existing
 * component owns I2C_NUM_0 or these pins.
 *
 * Detection procedure follows:
 *   $IDF_PATH/examples/peripherals/camera/common_components/
 *   sensor_init/example_sensor_init.c
 */

#define GC2053_BOARD_PWREN_GPIO GPIO_NUM_21
#define GC2053_BOARD_RST_GPIO   GPIO_NUM_41

#define GC2053_BOARD_SDA_GPIO   GPIO_NUM_42
#define GC2053_BOARD_SCL_GPIO   GPIO_NUM_43
#define GC2053_SCCB_FREQ_HZ     50000

#define GC2053_BOARD_XCLK_GPIO  GPIO_NUM_39
#define GC2053_BOARD_XCLK_HZ    24000000

static esp_cam_sensor_xclk_handle_t s_xclk_handle;
static bool s_xclk_started;

/* ESP-Video V4L2 capture state */
#define GC2053_V4L2_DEVICE             ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define GC2053_V4L2_BUFFER_COUNT       4

typedef struct {
    void *start;
    size_t length;
} gc2053_v4l2_buffer_t;

static int s_v4l2_fd = -1;
static gc2053_v4l2_buffer_t s_v4l2_buffers[GC2053_V4L2_BUFFER_COUNT];
static uint32_t s_v4l2_buffer_count;
static uint32_t s_v4l2_width;
static uint32_t s_v4l2_height;
static size_t s_v4l2_frame_size;
static bool s_v4l2_device_created;
static bool s_v4l2_streaming;
static bool s_isp_video_device_created;

#if CONFIG_ESP_VIDEO_ENABLE_H264_VIDEO_DEVICE
static bool s_h264_video_device_created;
#endif

static esp_err_t gc2053_v4l2_device_release(void);
#if CONFIG_ESP_VIDEO_ENABLE_H264_VIDEO_DEVICE
#define GC2053_H264_BUFFER_COUNT 1

typedef struct {
    void *start;
    size_t length;
} gc2053_h264_buffer_t;

static int s_h264_fd = -1;
static gc2053_h264_buffer_t s_h264_capture_buffers[GC2053_H264_BUFFER_COUNT];
static uint32_t s_h264_capture_buffer_count;
static bool s_h264_capture_streaming;
static bool s_h264_output_streaming;
static TaskHandle_t s_h264_task_handle;
static volatile bool s_h264_task_stop;
static volatile bool s_h264_task_failed;
static SemaphoreHandle_t s_h264_task_exit_sem;

static esp_err_t gc2053_h264_encoder_init(void);
static esp_err_t gc2053_h264_encoder_release(void);
static void gc2053_h264_stream_task(void *arg);
#endif

extern void rtsp_server_push_nal(uint8_t *data, size_t len);
extern void rtsp_server_push_camera_nal(uint8_t *data, size_t len);
extern void dummy_video_set_enabled(bool enabled);

static i2c_master_bus_handle_t s_i2c_bus;
static esp_sccb_io_handle_t s_sensor_sccb;
static esp_cam_sensor_device_t *s_sensor;

static void gc2053_power_and_reset_hw(void)
{
    ESP_LOGI(TAG, "Configuring CAM_PWREN (GPIO%d) and CAM_RST (GPIO%d)...",
             GC2053_BOARD_PWREN_GPIO, GC2053_BOARD_RST_GPIO);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GC2053_BOARD_PWREN_GPIO) | (1ULL << GC2053_BOARD_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // 1. ??깃퐣 ?袁⑹??LDO (2.8V, 1.8V, 1.2V) ??�뮄??(CAM_PWREN = HIGH)
    gpio_set_level(GC2053_BOARD_PWREN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. ??깃퐣 ?귐딅?????�???(Active Low)
    gpio_set_level(GC2053_BOARD_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(15));
    gpio_set_level(GC2053_BOARD_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "Sensor power enabled (GPIO%d=1) and reset deasserted (GPIO%d=1).",
             GC2053_BOARD_PWREN_GPIO, GC2053_BOARD_RST_GPIO);
}

/*
 * Kept for the next camera-integration stage. A detected sensor retains
 * its SCCB handle and bus; do not delete either while the sensor uses them.
 */
static esp_err_t gc2053_xclk_cleanup(void)
{
    if (s_xclk_handle == NULL) {
        if (s_xclk_started) {
            ESP_LOGE(TAG, "XCLK state invalid: started without a handle");
            return ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }

    if (s_xclk_started) {
        esp_err_t err = esp_cam_sensor_xclk_stop(s_xclk_handle);

        /*
         * Preserve the handle on every stop error.
         * ESP_ERR_INVALID_STATE does not prove that the mapping
         * handle has been released.
         */
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "XCLK stop failed; retaining handle: %s",
                     esp_err_to_name(err));
            return err;
        }

        s_xclk_started = false;
    }

    esp_err_t err = esp_cam_sensor_xclk_free(s_xclk_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "XCLK handle release failed; retaining handle: %s",
                 esp_err_to_name(err));
        return err;
    }

    s_xclk_handle = NULL;
    return ESP_OK;
}


static esp_err_t /*
 * DMA_CSI_LIFECYCLE_APP_ANALYSIS
 *
 * [?類ㅼ�????�듼??
 * - ?袁⑹????�뤿??�???곷??CSI V4L2 ??????밴쉐??
 *   esp_video_create_csi_video_device()???袁⑹�??뺣뼄.
 * - ???�젫 CSI controller ??밴쉐?? ESP-Video?? *   esp_video_csi_device.c?????esp_cam_new_csi_ctlr()???紐꾪???뺣뼄.
 * - CSI controller??start/stop/disable/delete??ESP-Video??
 *   ESP-IDF CSI ??뺤뵬??????lifecycle???????�?귐됰�??
 *
 * [??�뤿??�???곷???륁젟 ?�뀀?? ????
 * - ???�껋?�以???�귢?�猷??esp_cam_ctlr_stop/disable/del??餓λ쵎???紐꾪???? ???�뮉??
 * - ??�뤿??�???곷?????dw_gdma_del_channel()??筌욊????紐꾪???? ???�뮉??
 * - ESP-Video ?�곌????FREE ??�밴�??DMA ?�? ?袁⑥?�嚥??�쏄?�竊??? ???�뮉??
 *
 * [?�꿸?????륁젟 ??�몄???�꿸??]
 * - ?�곌????�쏆�??????곸젫??CSI stop??DMA/ISR ???�┛??? ?類ㅼ�??????묐뻬??뺣뼄.
 * - ???�┛??? ?類ㅼ�??? ??? ?�껋?????�뤿??�???곷???袁⑹�?delay??semaphore?? *   ?곕떽???? 筌띾Þ??CSI/GDMA ??뺤뵬?????????�┛???�껋?�以?�겫?????륁젟??뺣뼄.
 */
gc2053_v4l2_create_csi_device(void)
{
    if (s_sensor == NULL) {
        ESP_LOGE(TAG, "GC2053 sensor handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_v4l2_device_created) {
        ESP_LOGW(TAG, "V4L2 CSI device already created");
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;

#if CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE
    if (!s_isp_video_device_created) {
        ret = esp_video_create_isp_video_device();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create ESP-Video ISP device: %s",
                     esp_err_to_name(ret));
            return ret;
        }

        s_isp_video_device_created = true;
        ESP_LOGI(TAG, "ESP-Video ISP device created");
    }
#endif

    const esp_video_csi_device_config_t csi_config = {
        .dont_init_ldo = true,
    };

    ret = esp_video_create_csi_video_device(s_sensor, &csi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ESP-Video CSI device: %s",
                 esp_err_to_name(ret));

#if CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE
        if (s_isp_video_device_created) {
            esp_err_t isp_ret = esp_video_destroy_isp_video_device();
            if (isp_ret == ESP_OK) {
                s_isp_video_device_created = false;
                ESP_LOGI(TAG, "ESP-Video ISP device rolled back");
            } else {
                ESP_LOGE(TAG, "ISP rollback failed: %s",
                         esp_err_to_name(isp_ret));
            }
        }
#endif
        return ret;
    }

    s_v4l2_device_created = true;

#if CONFIG_ESP_VIDEO_ENABLE_H264_VIDEO_DEVICE
    if (!s_h264_video_device_created) {
        ret = esp_video_create_h264_video_device(true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create ESP-Video H.264 device: %s",
                     esp_err_to_name(ret));

            esp_err_t cleanup_ret = gc2053_v4l2_device_release();
            if (cleanup_ret != ESP_OK) {
                ESP_LOGE(TAG, "CSI/ISP rollback after H.264 failure failed: %s",
                         esp_err_to_name(cleanup_ret));
            }

            return ret;
        }

        s_h264_video_device_created = true;
        ESP_LOGI(TAG, "ESP-Video H.264 device created");
    }
#endif
    ESP_LOGI(TAG, "ESP-Video CSI device created using existing GC2053 sensor");
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_check_format(
    const esp_cam_sensor_format_t *cur_fmt,
    esp_video_csi_isp_in_out_format_t *in_out_format)
{
    if (cur_fmt == NULL || in_out_format == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (cur_fmt->format != ESP_CAM_SENSOR_PIXFORMAT_RAW10) {
        ESP_LOGE(TAG, "V4L2 ISP path requires RAW10 sensor input");
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t ret = esp_video_csi_check_format(
        cur_fmt->format,
        V4L2_PIX_FMT_YUV420,
        in_out_format);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RAW10 to YUV420 format is not supported: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ESP-Video format check passed: RAW10 -> YUV420");
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_open_device(void)
{
    if (s_v4l2_fd >= 0) {
        return ESP_OK;
    }

    int fd = open(GC2053_V4L2_DEVICE, O_RDWR);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open %s", GC2053_V4L2_DEVICE);
        return ESP_FAIL;
    }

    struct v4l2_capability cap = {0};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        ESP_LOGE(TAG, "VIDIOC_QUERYCAP failed for %s", GC2053_V4L2_DEVICE);
        close(fd);
        return ESP_FAIL;
    }

    uint32_t device_caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
                               ? cap.device_caps
                               : cap.capabilities;

    if (!(device_caps & V4L2_CAP_VIDEO_CAPTURE) ||
        !(device_caps & V4L2_CAP_STREAMING)) {
        ESP_LOGE(TAG, "%s does not support VIDEO_CAPTURE + STREAMING",
                 GC2053_V4L2_DEVICE);
        close(fd);
        return ESP_ERR_NOT_SUPPORTED;
    }

    struct timeval dqbuf_timeout = {
        .tv_sec = 3,
        .tv_usec = 0,
    };
    if (ioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &dqbuf_timeout) < 0) {
        int saved_errno = errno;
        ESP_LOGE(TAG, "VIDIOC_S_DQBUF_TIMEOUT failed: errno=%d", saved_errno);
        close(fd);
        return ESP_FAIL;
    }

    struct timeval actual_timeout = {0};
    if (ioctl(fd, VIDIOC_G_DQBUF_TIMEOUT, &actual_timeout) < 0) {
        int saved_errno = errno;
        ESP_LOGE(TAG, "VIDIOC_G_DQBUF_TIMEOUT failed: errno=%d", saved_errno);
        close(fd);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "DQBUF timeout configured: requested=3 s actual=%ld.%06ld s",
             (long)actual_timeout.tv_sec, (long)actual_timeout.tv_usec);

    s_v4l2_fd = fd;
    ESP_LOGI(TAG, "V4L2 device opened: %s, driver=%s, card=%s",
             GC2053_V4L2_DEVICE, cap.driver, cap.card);
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_set_format(const esp_cam_sensor_format_t *cur_fmt)
{
    if (s_v4l2_fd < 0 || cur_fmt == NULL ||
        cur_fmt->width == 0 || cur_fmt->height == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    struct v4l2_format format = {0};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = cur_fmt->width;
    format.fmt.pix.height = cur_fmt->height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    format.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(s_v4l2_fd, VIDIOC_S_FMT, &format) < 0) {
        ESP_LOGE(TAG, "VIDIOC_S_FMT failed");
        return ESP_FAIL;
    }

    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420 ||
        format.fmt.pix.width != cur_fmt->width ||
        format.fmt.pix.height != cur_fmt->height) {
        ESP_LOGE(TAG,
                 "Unsupported negotiated format: %ux%u, pixfmt=0x%08lx",
                 (unsigned)format.fmt.pix.width,
                 (unsigned)format.fmt.pix.height,
                 (unsigned long)format.fmt.pix.pixelformat);
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_v4l2_width = format.fmt.pix.width;
    s_v4l2_height = format.fmt.pix.height;
    s_v4l2_frame_size = 0;

    ESP_LOGI(TAG, "V4L2 format configured: %ux%u YUV420, sizeimage=%u (buffer size deferred to QUERYBUF)",
             (unsigned)s_v4l2_width,
             (unsigned)s_v4l2_height,
             (unsigned)format.fmt.pix.sizeimage);
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_buffers_init(void)
{
    if (s_v4l2_fd < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    struct v4l2_requestbuffers req = {0};
    req.count = GC2053_V4L2_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(s_v4l2_fd, VIDIOC_REQBUFS, &req) < 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed");
        return ESP_FAIL;
    }

    if (req.count < 2 || req.count > GC2053_V4L2_BUFFER_COUNT) {
        ESP_LOGE(TAG, "Invalid V4L2 buffer count: %u", (unsigned)req.count);
        return ESP_ERR_NO_MEM;
    }

    s_v4l2_buffer_count = req.count;

    for (uint32_t i = 0; i < s_v4l2_buffer_count; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(s_v4l2_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF failed: index=%u", (unsigned)i);
            return ESP_FAIL;
        }

        if (buf.length == 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF returned zero length: index=%u",
                     (unsigned)i);
            return ESP_ERR_INVALID_SIZE;
        }

        if (i == 0) {
            s_v4l2_frame_size = buf.length;
        } else if (buf.length != s_v4l2_frame_size) {
            ESP_LOGE(TAG, "Inconsistent V4L2 buffer size: index=%u size=%u expected=%u",
                     (unsigned)i, (unsigned)buf.length,
                     (unsigned)s_v4l2_frame_size);
            return ESP_ERR_INVALID_SIZE;
        }


        void *mapped = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, s_v4l2_fd, buf.m.offset);
        if (mapped == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap failed: index=%u", (unsigned)i);
            return ESP_ERR_NO_MEM;
        }

        s_v4l2_buffers[i].start = mapped;
        s_v4l2_buffers[i].length = buf.length;

        ESP_LOGI(TAG, "V4L2 buffer[%u]: addr=%p size=%u",
                 (unsigned)i, mapped, (unsigned)buf.length);
    }

    return ESP_OK;
}
static esp_err_t gc2053_v4l2_buffers_release(void)
{
    esp_err_t result = ESP_OK;

    for (uint32_t i = 0; i < GC2053_V4L2_BUFFER_COUNT; i++) {
        if (s_v4l2_buffers[i].start != NULL) {
            if (munmap(s_v4l2_buffers[i].start,
                       s_v4l2_buffers[i].length) < 0) {
                ESP_LOGE(TAG, "munmap failed: index=%u addr=%p size=%u",
                         (unsigned)i, s_v4l2_buffers[i].start,
                         (unsigned)s_v4l2_buffers[i].length);
                result = ESP_FAIL;
                continue;
            }

            s_v4l2_buffers[i].start = NULL;
            s_v4l2_buffers[i].length = 0;
        }
    }

    if (result != ESP_OK) {
        return result;
    }

    s_v4l2_buffer_count = 0;
    s_v4l2_frame_size = 0;
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_queue_buffers(void)
{
    if (s_v4l2_fd < 0 || s_v4l2_buffer_count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint32_t i = 0; i < s_v4l2_buffer_count; i++) {
        if (s_v4l2_buffers[i].start == NULL ||
            s_v4l2_buffers[i].length == 0) {
            ESP_LOGE(TAG, "V4L2 buffer[%u] is not mapped", (unsigned)i);
            return ESP_ERR_INVALID_STATE;
        }

        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(s_v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed: index=%u", (unsigned)i);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "V4L2 capture buffers queued: %u",
             (unsigned)s_v4l2_buffer_count);
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_stream_on(void)
{
    if (s_v4l2_fd < 0 || s_v4l2_buffer_count < 2 ||
        s_v4l2_streaming) {
        return ESP_ERR_INVALID_STATE;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(s_v4l2_fd, VIDIOC_STREAMON, &type) < 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed");
        return ESP_FAIL;
    }

    s_v4l2_streaming = true;
    ESP_LOGI(TAG, "V4L2 CSI capture streaming started");
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_dequeue_frame(
    struct v4l2_buffer *out_buf)
{
    if (s_v4l2_fd < 0 || !s_v4l2_streaming ||
        out_buf == NULL || s_v4l2_buffer_count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    int64_t dqbuf_start_us = esp_timer_get_time();
    ESP_LOGI(TAG, "VIDIOC_DQBUF enter: timeout=3 s");

    if (ioctl(s_v4l2_fd, VIDIOC_DQBUF, &buf) < 0) {
        int saved_errno = errno;
        int64_t elapsed_us = esp_timer_get_time() - dqbuf_start_us;
        ESP_LOGE(TAG, "VIDIOC_DQBUF failed: errno=%d elapsed=%lld us",
                 saved_errno, (long long)elapsed_us);
        return ESP_FAIL;
    }

    int64_t elapsed_us = esp_timer_get_time() - dqbuf_start_us;
    ESP_LOGI(TAG,
             "VIDIOC_DQBUF returned: index=%u bytesused=%u sequence=%u elapsed=%lld us",
             (unsigned)buf.index, (unsigned)buf.bytesused,
             (unsigned)buf.sequence, (long long)elapsed_us);

    if (buf.index >= s_v4l2_buffer_count) {
        ESP_LOGE(TAG, "Invalid dequeued buffer index: %u",
                 (unsigned)buf.index);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (buf.bytesused == 0 ||
        buf.bytesused > s_v4l2_buffers[buf.index].length) {
        ESP_LOGE(TAG, "Invalid frame size: index=%u bytesused=%u capacity=%u",
                 (unsigned)buf.index,
                 (unsigned)buf.bytesused,
                 (unsigned)s_v4l2_buffers[buf.index].length);
        return ESP_ERR_INVALID_SIZE;
    }

    *out_buf = buf;
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_queue_frame(uint32_t index)
{
    if (s_v4l2_fd < 0 || !s_v4l2_streaming ||
        index >= s_v4l2_buffer_count) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_v4l2_buffers[index].start == NULL ||
        s_v4l2_buffers[index].length == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;

    if (ioctl(s_v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
        ESP_LOGE(TAG, "VIDIOC_QBUF failed while requeueing index=%u",
                 (unsigned)index);
        return ESP_FAIL;
    }

    return ESP_OK;
}
static esp_err_t gc2053_v4l2_stream_off(void)
{
    if (s_v4l2_fd < 0 || !s_v4l2_streaming) {
        return ESP_OK;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(s_v4l2_fd, VIDIOC_STREAMOFF, &type) < 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMOFF failed");
        return ESP_FAIL;
    }

    s_v4l2_streaming = false;
    ESP_LOGI(TAG, "V4L2 CSI capture streaming stopped");
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_test_first_frame(void)
{
    struct v4l2_buffer frame = {0};
    int64_t start_us = esp_timer_get_time();

    esp_err_t ret = gc2053_v4l2_stream_on();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gc2053_v4l2_dequeue_frame(&frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "First VIDIOC_DQBUF failed: %s", esp_err_to_name(ret));
        esp_err_t stop_ret = gc2053_v4l2_stream_off();
        if (stop_ret != ESP_OK) {
            ESP_LOGE(TAG, "STREAMOFF after DQBUF failure failed: %s",
                     esp_err_to_name(stop_ret));
        }
        return ret;
    }

    int64_t elapsed_us = esp_timer_get_time() - start_us;
    ESP_LOGI(TAG,
             "First CSI frame received: index=%u bytesused=%u sequence=%u elapsed=%lld us",
             (unsigned)frame.index,
             (unsigned)frame.bytesused,
             (unsigned)frame.sequence,
             (long long)elapsed_us);

    ret = gc2053_v4l2_queue_frame(frame.index);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "First frame QBUF failed: %s", esp_err_to_name(ret));
        esp_err_t stop_ret = gc2053_v4l2_stream_off();
        if (stop_ret != ESP_OK) {
            ESP_LOGE(TAG, "STREAMOFF after QBUF failure failed: %s",
                     esp_err_to_name(stop_ret));
        }
        return ret;
    }

    ESP_LOGI(TAG, "First frame returned to V4L2 queue; capture stream remains active");
    return ESP_OK;
}
static esp_err_t gc2053_v4l2_device_release(void)
{
#if CONFIG_ESP_VIDEO_ENABLE_H264_VIDEO_DEVICE
    esp_err_t h264_ret = gc2053_h264_encoder_release();
    if (h264_ret != ESP_OK) {
        ESP_LOGE(TAG, "V4L2 cleanup aborted: H.264 encoder release failed: %s",
                 esp_err_to_name(h264_ret));
        return h264_ret;
    }
#endif
    if (s_v4l2_fd >= 0 && s_v4l2_streaming) {
        esp_err_t ret = gc2053_v4l2_stream_off();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "V4L2 cleanup aborted: STREAMOFF failed");
            return ret;
        }
    }

    esp_err_t buffer_release_ret = gc2053_v4l2_buffers_release();
    if (buffer_release_ret != ESP_OK) {
        ESP_LOGE(TAG, "V4L2 cleanup aborted: MMAP buffer release failed");
        return buffer_release_ret;
    }

    if (s_v4l2_fd >= 0) {
        struct v4l2_requestbuffers req = {0};
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        req.count = 0;

        if (ioctl(s_v4l2_fd, VIDIOC_REQBUFS, &req) < 0) {
            ESP_LOGW(TAG, "VIDIOC_REQBUFS(0) failed");
        }

        close(s_v4l2_fd);
        s_v4l2_fd = -1;
    }

#if CONFIG_ESP_VIDEO_ENABLE_H264_VIDEO_DEVICE
    if (s_h264_video_device_created) {
        esp_err_t ret = esp_video_destroy_h264_video_device(true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to destroy ESP-Video H.264 device: %s",
                     esp_err_to_name(ret));
            return ret;
        }
        s_h264_video_device_created = false;
        ESP_LOGI(TAG, "ESP-Video H.264 device destroyed");
    }
#endif

    if (s_v4l2_device_created) {
        esp_err_t ret = esp_video_destroy_csi_video_device();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to destroy ESP-Video CSI device: %s",
                     esp_err_to_name(ret));
            return ret;
        }
        s_v4l2_device_created = false;
    }

#if CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE
    if (s_isp_video_device_created) {
        esp_err_t ret = esp_video_destroy_isp_video_device();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to destroy ESP-Video ISP device: %s",
                     esp_err_to_name(ret));
            return ret;
        }
        s_isp_video_device_created = false;
    }
#endif

    s_v4l2_streaming = false;
    s_v4l2_width = 0;
    s_v4l2_height = 0;

    return ESP_OK;
}
static esp_err_t gc2053_init_csi_pipeline(const esp_cam_sensor_format_t *cur_fmt)
{
    if (cur_fmt == NULL || cur_fmt->port != ESP_CAM_SENSOR_MIPI_CSI ||
        cur_fmt->width == 0 || cur_fmt->height == 0 ||
        cur_fmt->mipi_info.lane_num == 0 ||
        cur_fmt->mipi_info.lane_num > 2) {
        ESP_LOGE(TAG, "Unsupported or incomplete MIPI CSI sensor format");
        return ESP_ERR_INVALID_ARG;
    }

    esp_video_csi_isp_in_out_format_t format_info = {0};
    esp_err_t ret = gc2053_v4l2_check_format(cur_fmt, &format_info);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gc2053_v4l2_create_csi_device();
    if (ret != ESP_OK) {
        goto fail;
    }

    ret = gc2053_v4l2_open_device();
    if (ret != ESP_OK) {
        goto fail;
    }

    ret = gc2053_v4l2_set_format(cur_fmt);
    if (ret != ESP_OK) {
        goto fail;
    }

    ret = gc2053_v4l2_buffers_init();
    if (ret != ESP_OK) {
        goto fail;
    }

    ret = gc2053_v4l2_queue_buffers();
    if (ret != ESP_OK) {
        goto fail;
    }

    ESP_LOGI(TAG,
             "GC2053 ESP-Video CSI device prepared: %ux%u RAW10 -> YUV420",
             (unsigned)cur_fmt->width,
             (unsigned)cur_fmt->height);

    return ESP_OK;

fail:
    ESP_LOGE(TAG, "V4L2 CSI initialization failed: %s",
             esp_err_to_name(ret));

    {
        esp_err_t cleanup_ret = gc2053_v4l2_device_release();
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "V4L2 initialization rollback incomplete: %s",
                     esp_err_to_name(cleanup_ret));
        }
    }

    return ret;
}
static esp_err_t gc2053_release_detect_resources(void)
{
    /*
     * Stop and destroy the video pipeline before deleting the sensor.
     * A failed video cleanup must preserve the sensor and its dependencies.
     */
    esp_err_t ret = gc2053_v4l2_device_release();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Video resource cleanup failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    if (s_sensor != NULL) {
        ret = esp_cam_sensor_del_dev(s_sensor);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Sensor deletion failed; retaining sensor dependencies: %s",
                     esp_err_to_name(ret));
            return ret;
        }
        s_sensor = NULL;
    }

    /*
     * Sensor deletion has completed. XCLK can now be stopped independently
     * of SCCB and I2C cleanup.
     */
    esp_err_t first_err = gc2053_xclk_cleanup();
    if (first_err != ESP_OK) {
        ESP_LOGE(TAG, "XCLK cleanup incomplete: %s",
                 esp_err_to_name(first_err));
    }

    if (s_sensor_sccb != NULL) {
        ret = esp_sccb_del_i2c_io(s_sensor_sccb);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SCCB cleanup failed; retaining SCCB and I2C handles: %s",
                     esp_err_to_name(ret));
            return (first_err != ESP_OK) ? first_err : ret;
        }
        s_sensor_sccb = NULL;
    }

    if (s_i2c_bus != NULL) {
        ret = i2c_del_master_bus(s_i2c_bus);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C bus cleanup failed; retaining bus handle: %s",
                     esp_err_to_name(ret));
            return (first_err != ESP_OK) ? first_err : ret;
        }
        s_i2c_bus = NULL;
    }

    return first_err;
}
static esp_err_t gc2053_detect_once(void)
{
    /* ??롫굡??λ???袁⑹???�?(GPIO21) ???귐딅?????�???GPIO41) ???�뻬 */
    gc2053_power_and_reset_hw();

    /* Manual XCLK start disabled: GC2053 driver owns XCLK through sensor_config. */

    esp_err_t err;
    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GC2053_BOARD_SDA_GPIO,
        .scl_io_num = GC2053_BOARD_SCL_GPIO,
        .flags.enable_internal_pullup = true,
    };

    err = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus creation failed: %s", esp_err_to_name(err));
        s_i2c_bus = NULL;
        // Manual XCLK cleanup disabled: driver owns XCLK.
        return err;
    }

    esp_cam_sensor_config_t sensor_config = {
        .reset_pin = -1, // Reset is controlled by gc2053_power_and_reset_hw().
        .pwdn_pin = -1,
        .xclk_pin = GC2053_BOARD_XCLK_GPIO,
        .xclk_freq_hz = GC2053_BOARD_XCLK_HZ,
    };

    esp_cam_sensor_detect_fn_t *detect_fn_start = NULL;
    esp_cam_sensor_detect_fn_t *detect_fn_end = NULL;
    esp_cam_sensor_detect_get_array(&detect_fn_start, &detect_fn_end);

    for (esp_cam_sensor_detect_fn_t *p = detect_fn_start;
         p < detect_fn_end; ++p) {
        if (p->sccb_addr == 0) {
            continue;
        }

        sccb_i2c_config_t sccb_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = p->sccb_addr,
            .scl_speed_hz = GC2053_SCCB_FREQ_HZ,
        };

        esp_sccb_io_handle_t candidate_sccb = NULL;
        err = sccb_new_i2c_io(s_i2c_bus, &sccb_config, &candidate_sccb);
        if (err != ESP_OK) {
            continue;
        }

        sensor_config.sccb_handle = candidate_sccb;

        esp_cam_sensor_device_t *candidate =
            (p->detect)(&sensor_config);

        if (candidate != NULL) {
            s_sensor = candidate;
            s_sensor_sccb = candidate_sccb;
            ESP_LOGI(TAG, "Sensor detected: %s, SCCB address 0x%02x",
                     candidate->name ? candidate->name : "(unnamed)",
                     p->sccb_addr);

            /*
             * Report the driver's selected mode; detection alone does not
             * confirm CSI reception, ISP output layout, or encoder readiness.
             * Evidence: esp_cam_sensor_types.h defines cur_format and the
             * esp_cam_sensor_format_t fields name/format/port/xclk/width/height/fps.
             */
            /*
             * Apply the GC2053 mode register table before creating the CSI pipeline.
             * The direct CSI device creation path does not set the sensor format.
             */
            esp_err_t set_fmt_err = esp_cam_sensor_set_format(candidate, NULL);
            if (set_fmt_err != ESP_OK) {
                ESP_LOGE(TAG, "GC2053 sensor format initialization failed: %s",
                         esp_err_to_name(set_fmt_err));

                esp_err_t cleanup_ret = gc2053_release_detect_resources();
                if (cleanup_ret != ESP_OK) {
                    ESP_LOGE(TAG, "Cleanup after sensor format failure incomplete: %s",
                             esp_err_to_name(cleanup_ret));
                }
                return set_fmt_err;
            }

            esp_cam_sensor_format_t fmt_storage = {0};
            esp_err_t fmt_err = esp_cam_sensor_get_format(candidate, &fmt_storage);
            const esp_cam_sensor_format_t *fmt = (fmt_err == ESP_OK) ? &fmt_storage : NULL;
            if (fmt != NULL) {
                ESP_LOGI(TAG,
                         "Selected mode: %s, format=%d port=%d %ux%u %u fps xclk=%d",
                         fmt->name ? fmt->name : "(unnamed)",
                         (int)fmt->format, (int)fmt->port,
                         (unsigned)fmt->width, (unsigned)fmt->height,
                         (unsigned)fmt->fps, fmt->xclk);

                esp_err_t csi_ret = gc2053_init_csi_pipeline(fmt);
                if (csi_ret != ESP_OK) {
                    ESP_LOGE(TAG, "CSI Pipeline initialization failed: %s",
                             esp_err_to_name(csi_ret));

                    esp_err_t cleanup_ret = gc2053_release_detect_resources();
                    if (cleanup_ret != ESP_OK) {
                        ESP_LOGE(TAG, "Cleanup after CSI initialization failure incomplete: %s",
                                 esp_err_to_name(cleanup_ret));
                    }
                    return csi_ret;
                }

                csi_ret = gc2053_v4l2_test_first_frame();
                if (csi_ret != ESP_OK) {
                    ESP_LOGE(TAG, "First CSI frame validation failed: %s",
                             esp_err_to_name(csi_ret));
                    esp_err_t cleanup_ret = gc2053_release_detect_resources();
                    if (cleanup_ret != ESP_OK) {
                        ESP_LOGE(TAG, "Cleanup after frame failure incomplete: %s",
                                 esp_err_to_name(cleanup_ret));
                    }
                    return csi_ret;
                }
            } else {
                ESP_LOGW(TAG, "esp_cam_sensor_get_format failed: %s",
                         esp_err_to_name(fmt_err));

                esp_err_t cleanup_ret = gc2053_release_detect_resources();
                if (cleanup_ret != ESP_OK) {
                    ESP_LOGE(TAG, "Cleanup after get_format failure incomplete: %s",
                             esp_err_to_name(cleanup_ret));
                }
                return fmt_err;
            }

            return ESP_OK;
        }

        /*
         * Keep ownership of the SCCB handle if device removal fails.
         * Evidence: managed_components/espressif__esp_sccb_intf/sccb_i2c/src/sccb_i2c.c
         * s_sccb_i2c_destroy() returns before freeing its handle when
         * i2c_master_bus_rm_device() fails.
         */
        esp_err_t del_err = esp_sccb_del_i2c_io(candidate_sccb);
        if (del_err != ESP_OK) {
            /*
             * The SCCB destroy path can fail before removing its I2C device.
             * Preserve both handles; deleting the bus could fail while the
             * device remains attached.
             * Evidence: managed_components/espressif__esp_sccb_intf/sccb_i2c/src/sccb_i2c.c
             * s_sccb_i2c_destroy(), and IDF components/esp_driver_i2c/i2c_master.c
             * i2c_del_master_bus() rejects a bus with attached devices.
             */
            s_sensor_sccb = candidate_sccb;
            ESP_LOGE(TAG,
                     "SCCB cleanup failed; retaining SCCB and I2C bus handles: %s",
                     esp_err_to_name(del_err));
            // Manual XCLK cleanup disabled: driver owns XCLK.
            return del_err;
        }
    }

    err = i2c_del_master_bus(s_i2c_bus);
    if (err != ESP_OK) {
        /*
         * Preserve the handle on failure so ownership is not lost.
         * Do not retry or force-clear the handle without checking the
         * underlying driver's bus-deletion contract.
         */
        ESP_LOGE(TAG, "I2C bus cleanup failed; retaining bus handle: %s",
                 esp_err_to_name(err));
        // Manual XCLK cleanup disabled: driver owns XCLK.
        return err;
    }
    s_i2c_bus = NULL;

    // Manual XCLK cleanup disabled: driver owns XCLK.
    ESP_LOGW(TAG, "No MIPI CSI sensor detected; keep dummy stream unchanged");
    return ESP_ERR_NOT_FOUND;
}

#if CONFIG_ESP_VIDEO_ENABLE_H264_VIDEO_DEVICE
static esp_err_t gc2053_h264_encoder_release(void)
{
    esp_err_t result = ESP_OK;
    bool task_active = (s_h264_task_handle != NULL);

    s_h264_task_stop = true;

    /*
     * Stop CSI first to unblock a task waiting in blocking CSI DQBUF.
     * Do not release any buffers until the H.264 task has exited.
     */
    if (task_active && s_v4l2_fd >= 0 && s_v4l2_streaming) {
        esp_err_t ret = gc2053_v4l2_stream_off();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CSI STREAMOFF while stopping H.264 task failed");
            result = ret;
        }
    }

    if (task_active) {
        if (s_h264_task_exit_sem == NULL ||
            xSemaphoreTake(s_h264_task_exit_sem, pdMS_TO_TICKS(3000)) != pdTRUE) {
            ESP_LOGE(TAG, "Timed out waiting for H.264 task termination");
            return ESP_ERR_TIMEOUT;
        }

        s_h264_task_handle = NULL;
    }

    /*
     * The task no longer accesses the H.264 device.
     * It is now safe to stop the encoder streams and release their buffers.
     */
    if (s_h264_fd >= 0 && s_h264_output_streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        if (ioctl(s_h264_fd, VIDIOC_STREAMOFF, &type) < 0) {
            ESP_LOGE(TAG, "H.264 OUTPUT STREAMOFF failed: errno=%d", errno);
            result = ESP_FAIL;
        } else {
            s_h264_output_streaming = false;
        }
    }

    if (s_h264_fd >= 0 && s_h264_capture_streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(s_h264_fd, VIDIOC_STREAMOFF, &type) < 0) {
            ESP_LOGE(TAG, "H.264 CAPTURE STREAMOFF failed: errno=%d", errno);
            result = ESP_FAIL;
        } else {
            s_h264_capture_streaming = false;
        }
    }

    if (result != ESP_OK) {
        return result;
    }

    for (uint32_t i = 0; i < GC2053_H264_BUFFER_COUNT; i++) {
        if (s_h264_capture_buffers[i].start != NULL) {
            if (munmap(s_h264_capture_buffers[i].start,
                       s_h264_capture_buffers[i].length) < 0) {
                ESP_LOGE(TAG, "H.264 CAPTURE munmap failed: index=%u errno=%d",
                         (unsigned)i, errno);
                return ESP_FAIL;
            }

            s_h264_capture_buffers[i].start = NULL;
            s_h264_capture_buffers[i].length = 0;
        }
    }

    if (s_h264_fd >= 0) {
        struct v4l2_requestbuffers req = {0};

        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        req.count = 0;
        if (ioctl(s_h264_fd, VIDIOC_REQBUFS, &req) < 0) {
            ESP_LOGW(TAG, "H.264 CAPTURE REQBUFS(0) failed: errno=%d", errno);
        }

        req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        req.memory = V4L2_MEMORY_USERPTR;
        req.count = 0;
        if (ioctl(s_h264_fd, VIDIOC_REQBUFS, &req) < 0) {
            ESP_LOGW(TAG, "H.264 OUTPUT REQBUFS(0) failed: errno=%d", errno);
        }

        close(s_h264_fd);
        s_h264_fd = -1;
    }

    if (s_h264_task_exit_sem != NULL) {
        vSemaphoreDelete(s_h264_task_exit_sem);
        s_h264_task_exit_sem = NULL;
    }

    s_h264_capture_buffer_count = 0;
    s_h264_capture_streaming = false;
    s_h264_output_streaming = false;
    s_h264_task_stop = false;

    return result;
}

static esp_err_t gc2053_h264_encoder_init(void)
{
    esp_err_t ret = ESP_FAIL;
    struct v4l2_format format = {0};
    struct v4l2_requestbuffers req = {0};
    struct v4l2_buffer buf = {0};

    if (s_h264_fd >= 0 || s_h264_task_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_v4l2_fd < 0 || !s_v4l2_streaming ||
        s_v4l2_width == 0 || s_v4l2_height == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    s_h264_task_exit_sem = xSemaphoreCreateBinary();
    if (s_h264_task_exit_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_h264_fd = open(ESP_VIDEO_H264_DEVICE_NAME, O_RDWR | O_NONBLOCK);
    if (s_h264_fd < 0) {
        ESP_LOGE(TAG, "Failed to open H.264 device: errno=%d", errno);
        ret = ESP_FAIL;
        goto err;
    }

    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    format.fmt.pix.width = s_v4l2_width;
    format.fmt.pix.height = s_v4l2_height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    format.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(s_h264_fd, VIDIOC_S_FMT, &format) < 0) {
        ESP_LOGE(TAG, "Failed to set H.264 input format: errno=%d", errno);
        ret = ESP_FAIL;
        goto err;
    }

    req.count = GC2053_H264_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_USERPTR;
    if (ioctl(s_h264_fd, VIDIOC_REQBUFS, &req) < 0 ||
        req.count == 0 || req.count > GC2053_H264_BUFFER_COUNT) {
        ESP_LOGE(TAG, "Failed to request H.264 OUTPUT buffers: errno=%d", errno);
        ret = ESP_FAIL;
        goto err;
    }

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_USERPTR;
    buf.index = 0;
    if (ioctl(s_h264_fd, VIDIOC_QUERYBUF, &buf) < 0) {
        ESP_LOGE(TAG, "H.264 OUTPUT QUERYBUF failed: errno=%d", errno);
        ret = ESP_FAIL;
        goto err;
    }

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = s_v4l2_width;
    format.fmt.pix.height = s_v4l2_height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
    format.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(s_h264_fd, VIDIOC_S_FMT, &format) < 0) {
        ESP_LOGE(TAG, "Failed to set H.264 output format: errno=%d", errno);
        ret = ESP_FAIL;
        goto err;
    }

    memset(&req, 0, sizeof(req));
    req.count = GC2053_H264_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(s_h264_fd, VIDIOC_REQBUFS, &req) < 0 ||
        req.count == 0 || req.count > GC2053_H264_BUFFER_COUNT) {
        ESP_LOGE(TAG, "Failed to request H.264 CAPTURE buffers: errno=%d", errno);
        ret = ESP_FAIL;
        goto err;
    }

    s_h264_capture_buffer_count = req.count;

    for (uint32_t i = 0; i < s_h264_capture_buffer_count; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(s_h264_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            ESP_LOGE(TAG, "H.264 CAPTURE QUERYBUF failed: index=%u errno=%d",
                     (unsigned)i, errno);
            ret = ESP_FAIL;
            goto err;
        }

        if (buf.length == 0) {
            ESP_LOGE(TAG, "H.264 CAPTURE QUERYBUF returned zero length: index=%u",
                     (unsigned)i);
            ret = ESP_ERR_INVALID_SIZE;
            goto err;
        }

        void *mapped = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, s_h264_fd, buf.m.offset);
        if (mapped == MAP_FAILED) {
            ESP_LOGE(TAG, "H.264 CAPTURE mmap failed: index=%u errno=%d",
                     (unsigned)i, errno);
            ret = ESP_ERR_NO_MEM;
            goto err;
        }

        s_h264_capture_buffers[i].start = mapped;
        s_h264_capture_buffers[i].length = buf.length;
    }

    for (uint32_t i = 0; i < s_h264_capture_buffer_count; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(s_h264_fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE(TAG, "H.264 CAPTURE QBUF failed: index=%u errno=%d",
                     (unsigned)i, errno);
            ret = ESP_FAIL;
            goto err;
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_h264_fd, VIDIOC_STREAMON, &type) < 0) {
        ESP_LOGE(TAG, "H.264 CAPTURE STREAMON failed: errno=%d", errno);
        ret = ESP_FAIL;
        goto err;
    }
    s_h264_capture_streaming = true;

    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(s_h264_fd, VIDIOC_STREAMON, &type) < 0) {
        ESP_LOGE(TAG, "H.264 OUTPUT STREAMON failed: errno=%d", errno);
        ret = ESP_FAIL;
        goto err;
    }
    s_h264_output_streaming = true;

    s_h264_task_stop = false;
    ESP_LOGI(TAG, "H.264 encoder initialized: %ux%u",
             (unsigned)s_v4l2_width, (unsigned)s_v4l2_height);
    return ESP_OK;

err:
    {
        esp_err_t cleanup_ret = gc2053_h264_encoder_release();
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "H.264 cleanup after initialization failure failed: %s",
                     esp_err_to_name(cleanup_ret));
        }
    }
    return ret;
}
#endif
#if CONFIG_ESP_VIDEO_ENABLE_H264_VIDEO_DEVICE
static esp_err_t gc2053_h264_dequeue_retry(
    struct v4l2_buffer *buf)
{
    while (!s_h264_task_stop) {
        if (ioctl(s_h264_fd, VIDIOC_DQBUF, buf) == 0) {
            return ESP_OK;
        }

        int saved_errno = errno;
        if (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        if (!s_h264_task_stop) {
            ESP_LOGE(TAG, "H.264 DQBUF failed: type=%u errno=%d",
                     (unsigned)buf->type, saved_errno);
        }
        return ESP_FAIL;
    }

    return ESP_ERR_INVALID_STATE;
}

static void gc2053_h264_stream_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "H.264 stream task started");
    bool camera_handoff_done = false;

    while (!s_h264_task_stop) {
        struct v4l2_buffer csi_buf = {0};
        struct v4l2_buffer output_buf = {0};
        struct v4l2_buffer capture_buf = {0};

        esp_err_t ret = gc2053_v4l2_dequeue_frame(&csi_buf);
        if (ret != ESP_OK) {
            if (!s_h264_task_stop) {
                ESP_LOGE(TAG, "CSI frame dequeue failed: %s",
                         esp_err_to_name(ret));
            }
            break;
        }

        output_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        output_buf.memory = V4L2_MEMORY_USERPTR;
        output_buf.index = 0;
        output_buf.m.userptr =
            (unsigned long)s_v4l2_buffers[csi_buf.index].start;
        output_buf.length = csi_buf.bytesused;

        if (ioctl(s_h264_fd, VIDIOC_QBUF, &output_buf) < 0) {
            ESP_LOGE(TAG, "H.264 OUTPUT QBUF failed: errno=%d", errno);
            break;
        }

        capture_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        capture_buf.memory = V4L2_MEMORY_MMAP;

        ret = gc2053_h264_dequeue_retry(&capture_buf);
        if (ret != ESP_OK) {
            break;
        }

        output_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        output_buf.memory = V4L2_MEMORY_USERPTR;

        ret = gc2053_h264_dequeue_retry(&output_buf);
        if (ret != ESP_OK) {
            break;
        }

        /*
         * OUTPUT DQBUF confirms that the encoder has finished using
         * the CSI USERPTR input buffer. It is now safe to requeue CSI.
         */
        ret = gc2053_v4l2_queue_frame(csi_buf.index);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CSI frame requeue failed: index=%u",
                     (unsigned)csi_buf.index);
            break;
        }

        uint32_t capture_index = capture_buf.index;
        bool capture_valid =
            capture_buf.index < s_h264_capture_buffer_count &&
            s_h264_capture_buffers[capture_buf.index].start != NULL &&
            s_h264_capture_buffers[capture_buf.index].length > 0 &&
            (capture_buf.flags & V4L2_BUF_FLAG_DONE) != 0 &&
            capture_buf.bytesused > 0 &&
            capture_buf.bytesused <=
                s_h264_capture_buffers[capture_buf.index].length;

        if (capture_valid) {
            uint8_t *encoded_data =
                (uint8_t *)s_h264_capture_buffers[capture_buf.index].start;

            if (!camera_handoff_done) {
                rtsp_server_push_camera_nal(encoded_data, capture_buf.bytesused);
                dummy_video_set_enabled(false);
                camera_handoff_done = true;
                ESP_LOGI(TAG, "First valid H.264 frame received; dummy stream disabled");
            } else {
                rtsp_server_push_nal(encoded_data, capture_buf.bytesused);
            }
        } else {
            ESP_LOGW(TAG,
                     "Invalid H.264 CAPTURE buffer: index=%u bytesused=%u flags=0x%x",
                     (unsigned)capture_buf.index,
                     (unsigned)capture_buf.bytesused,
                     (unsigned)capture_buf.flags);
        }

        if (capture_buf.index >= s_h264_capture_buffer_count) {
            ESP_LOGE(TAG, "Invalid H.264 CAPTURE index: %u",
                     (unsigned)capture_buf.index);
            break;
        }

        memset(&capture_buf, 0, sizeof(capture_buf));
        capture_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        capture_buf.memory = V4L2_MEMORY_MMAP;
        capture_buf.index = capture_index;

        /*
         * The index must be preserved from the dequeued buffer.
         * It is restored below from the saved index.
         */
        if (ioctl(s_h264_fd, VIDIOC_QBUF, &capture_buf) < 0) {
            ESP_LOGE(TAG, "H.264 CAPTURE QBUF failed: errno=%d", errno);
            break;
        }
    }

    s_h264_task_failed = !s_h264_task_stop;
    ESP_LOGW(TAG, "H.264 stream task exiting");

    if (s_h264_task_exit_sem != NULL) {
        xSemaphoreGive(s_h264_task_exit_sem);
    }

    vTaskDelete(NULL);
}
#endif
/*
 * Called by app_main() in main/main.c.
 * Keep this task single-shot: it detects the sensor, then exits.
 */
void gc2053_camera_detect_task(void *pvParameters)
{
    (void)pvParameters;

    esp_err_t err = gc2053_detect_once();
    if (err != ESP_OK || s_sensor == NULL || s_sensor_sccb == NULL) {
        ESP_LOGW(TAG, "Detection stage failed (%s); dummy stream remains active",
                 esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

#if CONFIG_ESP_VIDEO_ENABLE_H264_VIDEO_DEVICE
    err = gc2053_h264_encoder_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "H.264 encoder initialization failed: %s",
                 esp_err_to_name(err));
        ESP_LOGW(TAG, "Dummy stream remains active");
        vTaskDelete(NULL);
        return;
    }

    BaseType_t task_ret = xTaskCreate(
        gc2053_h264_stream_task,
        "gc2053_h264",
        8192,
        NULL,
        5,
        &s_h264_task_handle);

    if (task_ret != pdPASS) {
        s_h264_task_handle = NULL;
        ESP_LOGE(TAG, "Failed to create H.264 stream task");

        esp_err_t release_ret = gc2053_h264_encoder_release();
        if (release_ret != ESP_OK) {
            ESP_LOGE(TAG, "H.264 cleanup after task creation failure failed: %s",
                     esp_err_to_name(release_ret));
        }

        ESP_LOGW(TAG, "Dummy stream remains active");
        vTaskDelete(NULL);
        return;
    }


#else
    ESP_LOGW(TAG, "H.264 video device disabled; dummy stream remains active");
#endif

    vTaskDelete(NULL);
}














