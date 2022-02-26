#pragma once
#include "config.h"
#include "freertos/queue.h"

#define STEPPER_TIMER_DIVIDER (40)

enum ls_stepper_action {
    LS_STEPPER_ACTION_IDLE,
    LS_STEPPER_ACTION_DIRECTION_FORWARD,
    LS_STEPPER_ACTION_DIRECTION_REVERSE,
    LS_STEPPER_ACTION_SLEEP,
    LS_STEPPER_ACTION_STOP,
    LS_STEPPER_ACTION_RANDOM
}ls_stepper_action;

typedef struct ls_stepper_action_message {
    enum ls_stepper_action action;
    int32_t steps;
}ls_stepper_action_message;

QueueHandle_t ls_stepper_queue;
bool ls_stepper_direction;

// don't need 32 bits, but IRAM read/write must be 32-bit
static IRAM_ATTR volatile int32_t ls_stepper_position;

void ls_stepper_init(void);

void ls_stepper_task(void *pvParameter);


int32_t ls_stepper_get_position(void);
void ls_stepper_set_home_position(void);

void ls_stepper_stop(void);
void ls_stepper_forward(uint16_t steps);
void ls_stepper_reverse(uint16_t steps);
void ls_stepper_random(void);
