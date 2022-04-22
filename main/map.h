#pragma once
#include <stdlib.h>
#include "config.h"
#include "freertos/FreeRTOS.h"

enum ls_map_status_t {
    LS_MAP_STATUS_NOT_BUILT,
    LS_MAP_STATUS_OK,
    LS_MAP_STATUS_FAILED,
    LS_MAP_STATUS_IGNORE
}ls_map_status_t;

#define ls_map_is_excessive_misreads(misreads) (((misreads * 100) / (LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION)) > LS_MAP_ALLOWABLE_MISREAD_PERCENT)

uint32_t IRAM_ATTR ls_map_is_enabled_at(int32_t);
void ls_map_enable_at(int32_t);
void ls_map_disable_at(int32_t);
void ls_map_set_status(enum ls_map_status_t);
enum ls_map_status_t ls_map_get_status(void);
