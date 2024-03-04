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
#include "driver/gpio.h"

enum ls_laser_mode_t {
    LS_LASER_OFF,
    LS_LASER_ON,
    LS_LASER_MAPPED,
    LS_LASER_SCAN
}ls_laser_mode_t;

void ls_laser_set_mode(enum ls_laser_mode_t);
#define ls_laser_set_mode_off() ls_laser_set_mode(LS_LASER_OFF)
#define ls_laser_set_mode_on() ls_laser_set_mode(LS_LASER_ON)
#define ls_laser_set_mode_mapped() ls_laser_set_mode(LS_LASER_MAPPED)
#define ls_laser_set_mode_scan() ls_laser_set_mode(LS_LASER_SCAN)

uint32_t IRAM_ATTR ls_laser_mode_is_mappped(void);
uint32_t IRAM_ATTR ls_laser_mode_is_scan(void);
void ls_laser_pulse_init(void); //used only by self-test