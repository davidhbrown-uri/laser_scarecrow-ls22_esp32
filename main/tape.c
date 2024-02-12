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
#include "tape.h"
#include "config.h"
#include "events.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "buzzer.h"

extern SemaphoreHandle_t adc1_mux;
static bool _ls_tape_sensor_enabled = false;

bool ls_tape_sensor_is_enabled(void)
{
    return _ls_tape_sensor_enabled;
}
uint32_t ls_tape_sensor_read(void)
{
    bool was_disabled = ! ls_tape_sensor_is_enabled();
    if(was_disabled) 
    {
        ls_tape_sensor_enable();
    }
    xSemaphoreTake(adc1_mux, portMAX_DELAY);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LSADC1_REFLECTANCESENSE, ADC_ATTEN_DB_11);
    uint32_t adc_reading = 0;
    for (int i = 0; i < 4; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_REFLECTANCESENSE);
    }
    adc_reading /= 4;
    xSemaphoreGive(adc1_mux);
    if(was_disabled)
    {
        ls_tape_sensor_disable();
    }
    return (int) adc_reading;
}

void ls_tape_sensor_enable(void)
{
    gpio_set_level(LSGPIO_REFLECTANCEENABLE, 1);
    _ls_tape_sensor_enabled = true;
}

void ls_tape_sensor_disable(void)
{
    gpio_set_level(LSGPIO_REFLECTANCEENABLE, 0);
    _ls_tape_sensor_enabled = false;
}

void ls_tape_sensor_selftest_task(void *pvParameter)
{
    ls_tape_sensor_enable();
    uint32_t last_tape_reading = ls_tape_sensor_read();
    while (1)
    {
        int tape_reading = ls_tape_sensor_read();
        if(tape_reading < LS_REFLECTANCE_ADC_MAX_LIGHT && last_tape_reading >= LS_REFLECTANCE_ADC_MAX_LIGHT)
        {
            ls_buzzer_effect(LS_BUZZER_PLAY_ROOT);
            ls_event_enqueue(LSEVT_SELFTEST_TAPE_LIGHT);
        }
        if(tape_reading > LS_REFLECTANCE_ADC_MIN_DARK&& last_tape_reading <= LS_REFLECTANCE_ADC_MIN_DARK)
        {
            ls_buzzer_effect(LS_BUZZER_PLAY_OCTAVE);
            ls_event_enqueue(LSEVT_SELFTEST_TAPE_DARK);
        }
        last_tape_reading = tape_reading;
        vTaskDelay(1);
    }
}