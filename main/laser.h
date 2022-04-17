#pragma once
#include <stdlib.h>
#include "driver/gpio.h"

enum ls_laser_mode_t {
    LS_LASER_OFF,
    LS_LASER_ON,
    LS_LASER_MAPPED,
    LS_LASER_PULSE,
}ls_laser_mode_t;

void ls_laser_set_mode(enum ls_laser_mode_t);
#define ls_laser_set_mode_off() ls_laser_set_mode(LS_LASER_OFF)
#define ls_laser_set_mode_on() ls_laser_set_mode(LS_LASER_ON)
#define ls_laser_set_mode_mapped() ls_laser_set_mode(LS_LASER_MAPPED)
#define ls_laser_set_mode_pulse() ls_laser_set_mode(LS_LASER_PULSE)

uint32_t IRAM_ATTR ls_laser_mode_is_mappped(void);
