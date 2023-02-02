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
#include "config.h"
/*
gpio_num_t IRAM_ATTR _lsgpio_laserpowerenable = 32; // Nov '21 = 33
gpio_num_t _lsgpio_laserheaterenable = 26;          // Nov '21 = 32
gpio_num_t _lsgpio_stepperdirection = 19;           // Nov '21 = 5
gpio_num_t _lsgpio_steppersleep = 5;                // Nov '21 = 19
gpio_num_t _lsgpio_servopowerenable = 25;           // Nov '21 = 26
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
*/