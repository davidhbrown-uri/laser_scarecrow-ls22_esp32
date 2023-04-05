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
#include <stdlib.h>
#include "freertos/FreeRTOS.h"

void ls_tape_sensor_enable(void);
void ls_tape_sensor_disable(void);
uint32_t ls_tape_sensor_read(void);
bool ls_tape_sensor_is_enabled(void);
void ls_tape_sensor_selftest_task(void *pvParameter);