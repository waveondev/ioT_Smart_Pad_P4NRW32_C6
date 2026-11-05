#include "loadcell.h"

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "LOADCELL";

#define LOADCELL_PERIOD_MS  50
#define HX711_PULSE_US      1

typedef struct {
    gpio_num_t data_pin;
    gpio_num_t clock_pin;
    volatile int32_t raw;
    volatile bool valid;
} loadcell_channel_t;

static loadcell_channel_t s_channels[LOADCELL_COUNT] = {
    { GPIO_NUM_8,  GPIO_NUM_7,  0, false },
    { GPIO_NUM_9,  GPIO_NUM_2,  0, false },
    { GPIO_NUM_10, GPIO_NUM_3,  0, false },
    { GPIO_NUM_11, GPIO_NUM_5,  0, false },
};

static portMUX_TYPE s_loadcell_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_loadcell_task_handle;
static bool s_loadcell_initialized;

static esp_err_t loadcell_gpio_init(void)
{
    uint64_t data_mask = 0;
    uint64_t clock_mask = 0;

    for (int i = 0; i < LOADCELL_COUNT; i++) {
        data_mask |= (1ULL << s_channels[i].data_pin);
        clock_mask |= (1ULL << s_channels[i].clock_pin);
    }

    gpio_config_t data_config = {
        .pin_bit_mask = data_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&data_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DATA GPIO configuration failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    gpio_config_t clock_config = {
        .pin_bit_mask = clock_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&clock_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CLK GPIO configuration failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    for (int i = 0; i < LOADCELL_COUNT; i++) {
        gpio_set_level(s_channels[i].clock_pin, 0);
    }

    return ESP_OK;
}

static bool hx711_read_channel(loadcell_channel_t *channel, int32_t *value)
{
    if (gpio_get_level(channel->data_pin) != 0) {
        return false;
    }

    uint32_t data = 0;

    for (int bit = 0; bit < 24; bit++) {
        gpio_set_level(channel->clock_pin, 1);
        esp_rom_delay_us(HX711_PULSE_US);

        data = (data << 1) | (uint32_t)gpio_get_level(channel->data_pin);

        gpio_set_level(channel->clock_pin, 0);
        esp_rom_delay_us(HX711_PULSE_US);
    }

    /*
     * 25번째 펄스:
     * 다음 변환의 A 채널, Gain 128 선택
     */
    gpio_set_level(channel->clock_pin, 1);
    esp_rom_delay_us(HX711_PULSE_US);
    gpio_set_level(channel->clock_pin, 0);
    esp_rom_delay_us(HX711_PULSE_US);

    /* 24비트 2의 보수 부호 확장 */
    if (data & 0x800000U) {
        data |= 0xFF000000U;
    }

    *value = (int32_t)data;
    return true;
}

static void loadcell_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    int32_t values[LOADCELL_COUNT];

    ESP_LOGI(TAG, "HX711 measurement task started (50ms)");

    while (1) {
        for (int i = 0; i < LOADCELL_COUNT; i++) {
            if (hx711_read_channel(&s_channels[i], &values[i])) {
                portENTER_CRITICAL(&s_loadcell_mux);
                s_channels[i].raw = values[i];
                s_channels[i].valid = true;
                portEXIT_CRITICAL(&s_loadcell_mux);
            }
        }

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(LOADCELL_PERIOD_MS)
        );
    }
}

esp_err_t loadcell_start(void)
{
    if (s_loadcell_task_handle != NULL) {
        return ESP_OK;
    }

    esp_err_t err = loadcell_gpio_init();
    if (err != ESP_OK) {
        return err;
    }

    s_loadcell_initialized = true;

    BaseType_t result = xTaskCreate(
        loadcell_task,
        "loadcell",
        4096,
        NULL,
        5,
        &s_loadcell_task_handle
    );

    if (result != pdPASS) {
        s_loadcell_task_handle = NULL;
        s_loadcell_initialized = false;
        ESP_LOGE(TAG, "Failed to create HX711 task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "HX711 initialized: 4 channels, 50ms period");
    return ESP_OK;
}

esp_err_t loadcell_get_raw(uint8_t channel, int32_t *raw_value)
{
    if (channel >= LOADCELL_COUNT || raw_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_loadcell_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    bool valid;
    int32_t value;

    portENTER_CRITICAL(&s_loadcell_mux);
    valid = s_channels[channel].valid;
    value = s_channels[channel].raw;
    portEXIT_CRITICAL(&s_loadcell_mux);

    if (!valid) {
        return ESP_ERR_INVALID_STATE;
    }

    *raw_value = value;
    return ESP_OK;
}

static int64_t s_current_sum = 0;
static int64_t s_base_sum = 0;
static double s_scale = 0.010416;
static float  s_final_weight = 0.0f;

void loadcell_get_all_raw(int32_t out_raw[LOADCELL_COUNT])
{
    if (out_raw) {
        for (int i = 0; i < LOADCELL_COUNT; i++) {
            out_raw[i] = s_channels[i].raw;
        }
    }
}

int64_t loadcell_get_current_sum(void) { return s_current_sum; }
int64_t loadcell_get_base_sum(void)    { return s_base_sum; }
float   loadcell_get_weight(void)      { return s_final_weight; }
float   loadcell_get_scale(void)       { return (float)s_scale; }

void loadcell_set_scale(float scale)
{
    if (scale > 0.0f) {
        s_scale = (double)scale;
    }
}

void loadcell_tare(void)
{
    s_base_sum = s_current_sum;
}

