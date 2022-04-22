#pragma once
#include "freertos/FreeRTOS.h"
#include "debug.h"
BaseType_t _map(BaseType_t x, BaseType_t in_min, BaseType_t in_max, BaseType_t out_min, BaseType_t out_max);
BaseType_t _constrain(BaseType_t x, BaseType_t min, BaseType_t max);
