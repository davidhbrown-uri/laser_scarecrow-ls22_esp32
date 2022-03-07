#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_controls_status{
    LS_CONTROLS_STATUS_DISCONNECTED,
    LS_CONTROLS_STATUS_CONNECTED,
    LS_CONTROLS_STATUS_INVALID
};

void ls_controls_task(void *pvParameter);