#pragma once
#include "freertos/queue.h"
#include "config.h"
#include "debug.h"

#define STEPPER_TIMER_DIVIDER (40)

enum ls_stepper_action {
    LS_STEPPER_ACTION_IDLE, // 0
    LS_STEPPER_ACTION_FORWARD_STEPS, // 1
    LS_STEPPER_ACTION_REVERSE_STEPS, // 2
    LS_STEPPER_ACTION_SLEEP, // 3
    LS_STEPPER_ACTION_STOP, // 4
    LS_STEPPER_ACTION_RANDOM // 5
}ls_stepper_action;

typedef struct ls_stepper_action_message {
    enum ls_stepper_action action;
    int32_t steps;
}ls_stepper_action_message;

QueueHandle_t ls_stepper_queue;
// A4988 datasheet gives decay mode and other information while DIR=H, so make FORWARD==1
enum ls_stepper_direction {LS_STEPPER_DIRECTION_REVERSE, LS_STEPPER_DIRECTION_FORWARD} ls_stepper_direction;

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

#ifdef LSDEBUG_STEPPER
void ls_stepper_debug_task(void *pvParameter);
#endif