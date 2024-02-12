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
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "config.h"

extern SemaphoreHandle_t i2c_mux;

typedef enum ls_oled_status_t {
    LS_OLED_NOT_FOUND,
    LS_OLED_PRESENT,
    LS_OLED_UNKNOWN
} ls_oled_status_t;

enum ls_oled_status_t ls_oled_status(void);
void ls_oled_init(void);
void ls_oled_show_logo(void);
void ls_oled_blank_screen(void);
void ls_oled_goto_line(uint8_t line_number);
void ls_oled_clear_line(uint8_t line_number);
void ls_oled_println(char * format, ...);
