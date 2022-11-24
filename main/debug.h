#pragma once
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"


#define LSDEBUG_ENABLE

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
#ifdef LSDEBUG_ENABLE

extern SemaphoreHandle_t print_mux; // in ls2022_esp32.c

//variadic macro help from https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html
#define ls_debug_printf(args...) { xSemaphoreTake(print_mux, 1); printf(args); xSemaphoreGive(print_mux); }

// Uncomment any desired classes of debug output to enable output via ls_debug_printf

//#define LSDEBUG_STEPPER

//#define LSDEBUG_HOMING

//#define LSDEBUG_STATES

//#define LSDEBUG_CONTROLS

//#define LSDEBUG_TAPEMODE

//#define LSDEBUG_BUZZER

//#define LSDEBUG_MAP

#define LSDEBUG_LIGHTSENSE
#define LSDEBUG_LIGHTSENSE_ATTEN

//#define LSDEBUG_SERVO

//#define LSDEBUG_SETTINGS

//#define LSDEBUG_I2C

//#define LSDEBUG_TILT

#endif
