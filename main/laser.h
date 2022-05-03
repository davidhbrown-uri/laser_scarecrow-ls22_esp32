#pragma once
#include <stdlib.h>
#include "driver/gpio.h"

enum ls_laser_mode_t {
    LS_LASER_OFF,
    LS_LASER_ON,
    LS_LASER_MAPPED,
}ls_laser_mode_t;

void ls_laser_set_mode(enum ls_laser_mode_t);
#define ls_laser_set_mode_off() ls_laser_set_mode(LS_LASER_OFF)
#define ls_laser_set_mode_on() ls_laser_set_mode(LS_LASER_ON)
#define ls_laser_set_mode_mapped() ls_laser_set_mode(LS_LASER_MAPPED)

uint32_t IRAM_ATTR ls_laser_mode_is_mappped(void);
void ls_laser_pulse_init(void); //used only by self-test