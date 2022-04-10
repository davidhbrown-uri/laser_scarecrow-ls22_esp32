#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_buzzer_effects {
    LS_BUZZER_CLICK,
    LS_BUZZER_ALERT_1S,
    LS_BUZZER_ALTERNATE_HIGH,
    LS_BUZZER_PLAY_TAPE_ENABLE, // enable tape read
    LS_BUZZER_PLAY_TAPE_DISABLE,  // disable tape read
    LS_BUZZER_PLAY_TAPE_MISREAD, // bad tape read
    LS_BUZZER_PLAY_HOME_SUCCESS, // home success fanfare
    LS_BUZZER_PLAY_MAP_FAIL, // descending minor cord (map failure)
}ls_buzzer_effects;

QueueHandle_t ls_buzzer_queue;

void ls_buzzer_init(void);

void ls_buzzer_play(enum ls_buzzer_effects effect);

void ls_buzzer_handler_task(void *pvParameter);
