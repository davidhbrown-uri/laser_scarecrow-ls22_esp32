#pragma once
#include <stdlib.h>
#include "freertos/FreeRTOS.h"

void ls_tape_sensor_enable(void);
void ls_tape_sensor_disable(void);
uint32_t ls_tape_sensor_read(void);
bool ls_tape_sensor_is_enabled(void);
void ls_tape_sensor_selftest_task(void *pvParameter);