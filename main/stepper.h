/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022  David H. Brown

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
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

typedef struct ls_stepper_move_t {
    bool direction;
    int32_t steps;
}ls_stepper_move_t;


typedef void (*StepperMoveStrategy)(struct ls_stepper_move_t *move);
void ls_stepper_move_strategy_random(struct ls_stepper_move_t *move);

StepperMoveStrategy _ls_stepper_move_strategy;
struct ls_stepper_move_t ls_stepper_move;


QueueHandle_t ls_stepper_queue;
// A4988 datasheet gives decay mode and other information while DIR=H, so make FORWARD==1
enum ls_stepper_direction {LS_STEPPER_DIRECTION_REVERSE, LS_STEPPER_DIRECTION_FORWARD} ls_stepper_direction;

// don't need 32 bits, but IRAM read/write must be 32-bit
static IRAM_ATTR volatile int32_t ls_stepper_position;

void ls_stepper_init(void);

void ls_stepper_task(void *pvParameter);

void ls_stepper_set_move_strategy(StepperMoveStrategy strategy);

bool ls_stepper_is_stopped(void);
#define ls_stepper_is_moving() (!ls_stepper_is_stopped())

enum ls_stepper_direction ls_stepper_get_direction();

BaseType_t ls_stepper_get_steps_taken(void);

int32_t ls_stepper_get_position(void);
void ls_stepper_set_home_position(void);

void ls_stepper_stop(void);
void ls_stepper_forward(uint16_t steps);
void ls_stepper_reverse(uint16_t steps);
void ls_stepper_random(void);
void ls_stepper_sleep(void);

void ls_stepper_set_random_reverse_per255(uint8_t value);

#define ls_stepper_off() ls_stepper_sleep()

void ls_stepper_set_maximum_steps_per_second(int);

#ifdef LSDEBUG_STEPPER
void ls_stepper_debug_task(void *pvParameter);
#endif