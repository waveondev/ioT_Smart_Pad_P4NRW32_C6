#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "loadcell.h"

static const char *TAG = "smart_pad_main";

static void Process_Command(char *cmd) 
{
    cmd[strcspn(cmd, "\r\n")] = '\0';
    if (strlen(cmd) == 0) return;

    if (strncmp(cmd, "get weight", 10) == 0) {
        int32_t raw[LOADCELL_COUNT] = {0};
        loadcell_get_all_raw(raw);
        int64_t cur_sum = loadcell_get_current_sum();
        int64_t base_sum = loadcell_get_base_sum();
        float weight = loadcell_get_weight();

        printf("\n=============== [ WEIGHT STATUS ] ===============\n");
        printf("  - Loadcell CH0 : %ld\n", (long)raw[0]);
        printf("  - Loadcell CH1 : %ld\n", (long)raw[1]);
        printf("  - Loadcell CH2 : %ld\n", (long)raw[2]);
        printf("  - Loadcell CH3 : %ld\n", (long)raw[3]);
        printf("  - Current Sum: %lld (Delta: %lld)\n", (long long)cur_sum, (long long)(cur_sum - base_sum));
        printf("  - Tare Base  : %lld\n", (long long)base_sum);
        printf("  - Final Calc : %.2f g\n", weight);
        printf("==================================================\n\n");
        fflush(stdout);
        return;
    }
    else if (strncmp(cmd, "get tare", 8) == 0) {
        printf("[GET TARE] Base Sum = %lld\n", (long long)loadcell_get_base_sum());
        fflush(stdout);
        return;
    }
    else if (strncmp(cmd, "get scale", 9) == 0) {
        printf("[GET SCALE] Scale Factor = %.8f\n", loadcell_get_scale());
        fflush(stdout);
        return;
    }
    else if (strncmp(cmd, "set tare", 8) == 0) {
        loadcell_tare();
        printf("[SET TARE OK] New Tare Set. Base Sum = %lld\n", (long long)loadcell_get_base_sum());
        fflush(stdout);
        return;
    }
    else if (strncmp(cmd, "set scale", 9) == 0) {
        float new_scale = 0.0f;
        if (sscanf(cmd, "set scale %f", &new_scale) == 1 && new_scale > 0.0f) {
            loadcell_set_scale(new_scale);
            printf("[SET SCALE OK] Scale updated to %.8f\n", new_scale);
        } else {
            printf("[ERROR] Usage: set scale [positive_float]\n");
        }
        fflush(stdout);
        return;
    }

    ESP_LOGI(TAG, "CLI Cmd Received -> '%s'", cmd);
}

static void init_uart0(void)
{
    if (!uart_is_driver_installed(UART_NUM_0)) {
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    }
}

static void uart_process_task(void *arg) 
{
    init_uart0();
    uint8_t data[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, data, sizeof(data) - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            Process_Command((char *)data);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================================");
    ESP_LOGI(TAG, "   ioT_Smart_Pad_P4C6 Starting...");
    ESP_LOGI(TAG, "========================================================");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_err_t loadcell_ret = loadcell_start();
    if (loadcell_ret != ESP_OK) {
        ESP_LOGE(TAG, "Loadcell init failed: %s", esp_err_to_name(loadcell_ret));
    } else {
        ESP_LOGI(TAG, "Loadcell 4-Corner Module Started");
    }

    xTaskCreate(uart_process_task, "uart_cli", 3072, NULL, 10, NULL);
}



