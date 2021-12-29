#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_buzzer_effects {
    LS_BUZZER_CLICK
};

QueueHandle_t ls_buzzer_queue;

void buzzer_init(void);

void buzzer_play(enum ls_buzzer_effects effect);

void buzzer_handler_task(void *pvParameter);
