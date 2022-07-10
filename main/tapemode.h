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
#include "debug.h"

enum ls_tapemode_mode
{
    LS_TAPEMODE_SELFTEST,
    LS_TAPEMODE_BLACK_SAFE,
    LS_TAPEMODE_BLACK,
    LS_TAPEMODE_IGNORE,
    LS_TAPEMODE_REFLECT,
    LS_TAPEMODE_REFLECT_SAFE,
    LS_TAPEMODE_NOT_INITIALIZED
};

/**
 * @brief Call once on startup, then use ls_tapemode();
 * 
 */
void ls_tapemode_init(void);

/**
 * @brief the tape mode selected when ls_tapemode_init() was called
 * 
 * @return enum ls_tapemode_mode 
 */
enum ls_tapemode_mode ls_tapemode(void);

/**
 * @brief the currently selected tape mode
 * 
 * The tape mode would not normally change during operation
 * 
 * @return enum ls_tapemode_mode 
 */
enum ls_tapemode_mode ls_tapemode_current(void);

void ls_tapemode_selftest_task(void *pvParameter);

#ifdef LSDEBUG_TAPEMODE
void ls_tapemode_debug_task(void *pvParameter);
#endif