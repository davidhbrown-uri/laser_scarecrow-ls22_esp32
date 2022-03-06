#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_buzzer_effects {
    LS_BUZZER_CLICK,
    LS_BUZZER_ALERT_1S,
    LS_BUZZER_ALTERNATE_HIGH,
}ls_buzzer_effects;

QueueHandle_t ls_buzzer_queue;

void buzzer_init(void);

void buzzer_play(enum ls_buzzer_effects effect);

void buzzer_handler_task(void *pvParameter);
