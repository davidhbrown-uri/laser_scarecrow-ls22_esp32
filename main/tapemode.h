#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "debug.h"

enum ls_tapemode{
    LS_TAPEMODE_SELFTEST,
    LS_TAPEMODE_BLACK_SAFE,
    LS_TAPEMODE_BLACK,
    LS_TAPEMODE_IGNORE,
    LS_TAPEMODE_REFLECT,
    LS_TAPEMODE_REFLECT_SAFE
};

enum ls_tapemode ls_tapemode_get(void);

#ifdef LSDEBUG_TAPEMODE
void ls_tapemode_debug_task(void *pvParameter);
#endif