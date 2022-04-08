#pragma once
#include <stdlib.h>
#include "freertos/FreeRTOS.h"

enum ls_map_status_t {
    LS_MAP_STATUS_NOT_BUILT,
    LS_MAP_STATUS_OK,
    LS_MAP_STATUS_FAILED
};

bool ls_map_is_enabled_at(int32_t stepper_position);
void ls_map_enable_at(int32_t stepper_position);
void ls_map_disable_at(int32_t stepper_position);

enum ls_map_status_t ls_map_status(void);
