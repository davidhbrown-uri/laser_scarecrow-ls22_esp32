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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_controls_status{
    LS_CONTROLS_STATUS_OFF,
    LS_CONTROLS_STATUS_UPPER,
    LS_CONTROLS_STATUS_LOWER,
    LS_CONTROLS_STATUS_BOTH,
    LS_CONTROLS_STATUS_INVALID
};

enum ls_controls_status ls_controls_get_current_status(void);

void ls_controls_task(void *pvParameter);
