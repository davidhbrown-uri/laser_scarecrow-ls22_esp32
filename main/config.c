#include "config.h"

gpio_num_t IRAM_ATTR _lsgpio_laserpowerenable = 32; // Nov '21 = 33
gpio_num_t _lsgpio_laserheaterenable = 26;          // Nov '21 = 32
gpio_num_t _lsgpio_stepperdirection = 19;           // Nov '21 = 5
gpio_num_t _lsgpio_steppersleep = 5;                // Nov '21 = 19
gpio_num_t _lsgpio_servopowerenable = 25;           // Nob '21 = 26
gpio_num_t _lsgpio_servopulse = 33;                 // Nov '21 = 25

gpio_num_t IRAM_ATTR lsgpio_laserpowerenable()
{
    return _lsgpio_laserpowerenable;
}
gpio_num_t lsgpio_laserheaterenable()
{
    return _lsgpio_laserheaterenable;
}
gpio_num_t lsgpio_stepperdirection()
{
    return _lsgpio_stepperdirection;
}
gpio_num_t lsgpio_steppersleep()
{
    return _lsgpio_steppersleep;
}
gpio_num_t lsgpio_servopowerenable(void)
{
    return _lsgpio_servopowerenable;
}
gpio_num_t lsgpio_servopulse(void)
{
    return _lsgpio_servopulse;
}

void ls_config_set_gpio_nov21(void)
{
    _lsgpio_laserpowerenable = 33;
    _lsgpio_laserheaterenable = 32;
    _lsgpio_stepperdirection = 5;
    _lsgpio_steppersleep = 19;
    _lsgpio_servopowerenable = 26;
    _lsgpio_servopulse = 25;
}
