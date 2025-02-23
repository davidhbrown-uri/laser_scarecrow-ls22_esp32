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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_controls_status{
    LS_CONTROLS_STATUS_OFF, // 0
    LS_CONTROLS_STATUS_UPPER, // 1
    LS_CONTROLS_STATUS_LOWER, // 2
    LS_CONTROLS_STATUS_BOTH, // 3
    LS_CONTROLS_STATUS_INVALID // 4
};

enum ls_controls_status ls_controls_get_current_status(void);

void ls_controls_task(void *pvParameter);
