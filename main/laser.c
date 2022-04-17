#include "laser.h"
#include "config.h"
#include "driver/gpio.h"

enum ls_laser_mode_t IRAM_ATTR _ls_laser_mode = LS_LASER_OFF;

void ls_laser_set_mode(enum ls_laser_mode_t requested_mode)
{
    switch (requested_mode)
    {
    case LS_LASER_OFF:
        gpio_set_level(LSGPIO_LASERPOWERENABLE, 0);
        break;
    case LS_LASER_ON:
        gpio_set_level(LSGPIO_LASERPOWERENABLE, 1);
        break;
    case LS_LASER_MAPPED:
        gpio_set_level(LSGPIO_LASERPOWERENABLE, 0);
        break;
    case LS_LASER_PULSE:
        break;
    }
    _ls_laser_mode = requested_mode;
}

/**
 * @brief IRAM access function for stepper timer ISR callback
 * 
 * @return uint32_t 
 */
uint32_t IRAM_ATTR ls_laser_mode_is_mappped(void)
{
    return (LS_LASER_MAPPED == _ls_laser_mode) ? 1 : 0;
}

