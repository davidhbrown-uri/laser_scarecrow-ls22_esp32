#pragma once
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"


#define LSDEBUG_ENABLE

#ifdef LSDEBUG_ENABLE

extern SemaphoreHandle_t print_mux; // in ls2022_esp32.c

//variadic macro help from https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html
#define ls_debug_printf(args...) { xSemaphoreTake(print_mux, portMAX_DELAY); printf(args); xSemaphoreGive(print_mux); }

// Uncomment any desired classes of debug output to enable output via ls_debug_printf

//#define LSDEBUG_STEPPER

//#define LSDEBUG_HOMING

//#define LSDEBUG_STATES

//#define LSDEBUG_CONTROLS

//#define LSDEBUG_TAPEMODE

//#define LSDEBUG_BUZZER

//#define LSDEBUG_MAP

//#define LSDEBUG_LIGHTSENSE

//#define LSDEBUG_SERVO

//#define LSDEBUG_SETTINGS

#define LSDEBUG_I2C

#endif
