#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define LOADCELL_COUNT 4

esp_err_t loadcell_start(void);
esp_err_t loadcell_get_raw(uint8_t channel, int32_t *raw_value);
void      loadcell_get_all_raw(int32_t out_raw[LOADCELL_COUNT]);

int64_t   loadcell_get_current_sum(void);
int64_t   loadcell_get_base_sum(void);
float     loadcell_get_weight(void);

float     loadcell_get_scale(void);
void      loadcell_set_scale(float scale);
void      loadcell_tare(void);
