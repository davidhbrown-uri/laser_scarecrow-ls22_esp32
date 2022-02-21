#pragma once
#include "config.h"
#define STEPPER_TIMER_DIVIDER (40)

volatile bool ls_stepperstep_level = false;
volatile bool ls_stepper_direction = false;
volatile bool ls_stepper_sleep = false;

volatile int16_t ls_stepper_position = 0;

void ls_stepper_seek_home();

void ls_stepper_enable();

void ls_stepper_disable();

bool ls_stepper_position_valid();