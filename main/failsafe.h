/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2024 David H. Brown

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
#include <stdlib.h>
#include "driver/gpio.h"

// bool ls_failsafe_has_heartbeat_at_poweron(void);
/**
 * Enables interrupt; sets up timer, assigns callback to alarm timer
 */
void ls_failsafe_init(void);
/**
 * Pauses the alarm timer and removes the ISR function that counts edges
 */
void ls_failsafe_pause(void);
/**
 * Adds ISR function to handler to count edges; starts alarm timer
 */
void ls_failsafe_start(void);
uint64_t ls_failsafe_edge_count(void);
#ifdef LSDEBUG_FAILSAFE
void ls_failsafe_debug_task(void *pvParameter);
#endif
