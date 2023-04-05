/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2023 Isaac Chen and David H. Brown

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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_servo_event_types {
    LS_SERVO_ON,            // Data: unused
    LS_SERVO_OFF,           // Data: unused
    LS_SERVO_JUMP_TO,       // Data: pulse_width
    LS_SERVO_MOVE_TO,       // Data: pulse_width
    LS_SERVO_MOVE_RANDOMLY, // Data: unused
    LS_SERVO_SWEEP          // Data: unused
};

enum _ls_servo_motion_modes {
    LS_SERVO_MODE_FIXED,  // Moves to a fixed position
    LS_SERVO_MODE_RANDOM, // Continuously moves to a random position
    LS_SERVO_MODE_SWEEP   // Continuously sweeps between the min and max positions
};

struct ls_servo_event {
    enum ls_servo_event_types event_type;
    uint16_t data;
};

QueueHandle_t ls_servo_queue;

void ls_servo_init(void);

void ls_servo_on(void);

void ls_servo_off(void);

void ls_servo_sweep(void);

void ls_servo_random(void);

void ls_servo_moveto(uint32_t pulsewidth_us);

void ls_servo_jumpto(uint32_t pulsewidth_us);

void ls_servo_task(void* pvParameter);