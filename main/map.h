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
#include <stdlib.h>
#include "config.h"
#include "freertos/FreeRTOS.h"

enum ls_map_status_t {
    LS_MAP_STATUS_NOT_BUILT,
    LS_MAP_STATUS_OK,
    LS_MAP_STATUS_FAILED,
    LS_MAP_STATUS_IGNORE
}ls_map_status_t;

 enum ls_state_map_reading {
    LS_STATE_MAP_READING_DISABLE,
    LS_STATE_MAP_READING_ENABLE,
    LS_STATE_MAP_READING_MISREAD,
    LS_STATE_MAP_READING_INIT
}ls_state_map_reading;

#define ls_map_is_excessive_misreads(misreads) (((misreads * 100) / (LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION)) > LS_MAP_ALLOWABLE_MISREAD_PERCENT)

uint32_t IRAM_ATTR ls_map_is_enabled_at(int32_t);
void ls_map_enable_at(int32_t);
void ls_map_disable_at(int32_t);
void ls_map_set_status(enum ls_map_status_t);
enum ls_map_status_t ls_map_get_status(void);

void _ls_state_map_build_histogram(uint16_t min_adc, uint16_t max_adc);
void _ls_state_map_build_histogram_get_peaks_edges(int *low_peak_bin, int *low_edge_bin, int *high_peak_bin, int *high_edge_bin);
void _ls_state_map_build_set_map(int *enable_count, int *disable_count, int *misread_count, uint16_t low_threshold, uint16_t high_threshold);
void _ls_state_map_build_read_and_set_map(int *enable_count, int *disable_count, int *misread_count);

uint16_t ls_map_min_adc(void);
uint16_t ls_map_max_adc(void);
