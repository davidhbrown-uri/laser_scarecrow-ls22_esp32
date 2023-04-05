/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2023 David H. Brown

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
#include "laser.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/timer.h"

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

BaseType_t IRAM_ATTR _ls_laser_pulse_counter = 0;
//timer alarms every 25ms; 120 alarms => 3 seconds
static bool IRAM_ATTR ls_laser_pulse_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    _ls_laser_pulse_counter++;
    if (_ls_laser_pulse_counter > 120)
    {
        _ls_laser_pulse_counter = 0;
    }
    gpio_set_level(LSGPIO_LASERPOWERENABLE, 0 == _ls_laser_pulse_counter);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}
void ls_laser_pulse_init(void)
{
    gpio_set_level(LSGPIO_LASERPOWERENABLE, 0); // don't do anything while we get ready
    timer_config_t laser_pulse_timer_config = {
        .divider = 40000, // base is 80MHz => 20kHz
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_1, &laser_pulse_timer_config));
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0ULL));
    // alarm every 25ms
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, APB_CLK_FREQ / 40000 / 40));
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_1));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, ls_laser_pulse_isr_callback, NULL, 0));
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_1));
}