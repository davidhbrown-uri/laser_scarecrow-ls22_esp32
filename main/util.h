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
#include "freertos/FreeRTOS.h"
#include "debug.h"
BaseType_t _map(BaseType_t x, BaseType_t in_min, BaseType_t in_max, BaseType_t out_min, BaseType_t out_max);
BaseType_t _constrain(BaseType_t x, BaseType_t min, BaseType_t max);
BaseType_t _difference_exceeds_threshold(BaseType_t previous, BaseType_t current, BaseType_t threshold);
uint16_t _make_log_response(uint16_t value, uint8_t bits);
