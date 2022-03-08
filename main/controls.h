#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_controls_status{
    LS_CONTROLS_STATUS_DISCONNECTED,
    LS_CONTROLS_STATUS_CONNECTED,
    LS_CONTROLS_STATUS_INVALID
};

enum ls_controls_status ls_controls_get_current_status(void);
uint32_t ls_controls_get_speed(void);
uint32_t ls_controls_get_speed(void);
uint32_t ls_controls_get_range(void);

void ls_controls_task(void *pvParameter);
