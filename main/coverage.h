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
#include "stepper.h"

// record 256 positions where laser was on
#define LS_COVERAGE_POSITIONS_COUNT 256
// check for position every so often
#define LS_COVERAGE_POSITIONS_MS 500
// if this much time has elapsed since the task last was running and the task is started, clear old position data
#define LS_COVERAGE_POSITIONS_INVALID_AFTER_SEC 180 

#ifdef LSDEBUG_COVERAGE_MEASURE
void ls_coverage_debug_task(void *pvParameter);
#endif

// a ring buffer of the most recent positions where the laser was shining
ls_stepper_position_t ls_laser_positions[LS_COVERAGE_POSITIONS_COUNT];

void ls_coverage_task(void *pvParameter);
bool ls_coverage_ready(void);
struct ls_map_SpanNode *ls_coverage_next_span(void);
