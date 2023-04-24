/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2023 David H. Brown

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

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include "config.h"

QueueHandle_t ls_leds_queue;

// define some colors to use with these GRB leds
#define GRB_OFF 0
#define GRB_WHITE 0x00FFFFFF
#define GRB_RED 0x0000FF00
#define GRB_GREEN 0x00FF0000
#define GRB_BLUE 0x000000FF
#define GRB_YELLOW 0x00FFFF00
#define GRB_CYAN 0x00FF00FF
#define GRB_MAGENTA 0x0000FFFF

typedef struct ls_ledcycle
{
    uint8_t length;
    TickType_t speed;
    uint8_t offset;
    int *data;
} ls_ledcycle_t;

extern ls_ledcycle_t LEDCYCLE_STATIC, LEDCYCLE_RAINBOW, LEDCYCLE_WARNING, LEDCYCLE_SLEEP,
    LEDCYCLE_CONTROLS_UPPER, LEDCYCLE_CONTROLS_LOWER, LEDCYCLE_CONTROLS_BOTH,
    // LEDCYCLE_RED_FLASH, LEDCYCLE_GREEN_PULSE, LEDCYCLE_YELLOW_PULSE, 
    LEDCYCLE_FAIL_ROTATE, LEDCYCLE_FAIL_ACCELEROMETER, LEDCYCLE_FAIL_TILT, LEDCYCLE_FAIL_HOMING, LEDCYCLE_FAIL_SCANNING;

void ls_leds_init(void);

void ls_leds_handler_task(void *pvParameter);

void ls_leds_off(void);

void ls_leds_cycle(ls_ledcycle_t ledcycle);

void ls_leds_rgb(uint8_t red, uint8_t green, uint8_t blue);

void ls_leds_single(int which, int color);

void ls_leds_pulses_2sec(int count, int color);
