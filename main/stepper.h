#pragma once
#include "config.h"

enum ls_stepper_mode {
    LS_STEPPER_MODE_SLEEP,
    LS_STEPPER_MODE_STOP,
    LS_STEPPER_MODE_HOME,
    LS_STEPPER_MODE_RANDOM
}ls_stepper_mode;

QueueHandle_t ls_stepper_queue;

// don't need 32 bits, but IRAM read/write must be 32-bit
static IRAM_ATTR volatile int32_t ls_stepper_position;

void ls_stepper_init(void);

void ls_stepper_task(void *pvParameter);

void ls_stepper_set_mode(enum ls_stepper_mode mode);
