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
#include "tapemode.h"
#include "buzzer.h"
#include "config.h"
#include "debug.h"
#include "events.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/semphr.h"
#include "esp_adc_cal.h"

extern SemaphoreHandle_t adc1_mux;
extern SemaphoreHandle_t print_mux;
static enum ls_tapemode_mode _ls_tapemode = LS_TAPEMODE_NOT_INITIALIZED;

static enum ls_tapemode_mode ls_tapemode_from_adc(uint32_t adc_reading)
{
    if (adc_reading > LS_TAPEMODE_THRESHOLD_5)
    {
        return LS_TAPEMODE_LIGHT_SAFE;
    }
    if (adc_reading > LS_TAPEMODE_THRESHOLD_4)
    {
        return LS_TAPEMODE_LIGHT;
    }
    if (adc_reading > LS_TAPEMODE_THRESHOLD_3)
    {
        return LS_TAPEMODE_IGNORE;
    }
    if (adc_reading > LS_TAPEMODE_THRESHOLD_2)
    {
        return LS_TAPEMODE_DARK;
    }
    if (adc_reading > LS_TAPEMODE_THRESHOLD_1)
    {
        return LS_TAPEMODE_DARK_SAFE;
    }
    return LS_TAPEMODE_SELFTEST;
}

void ls_tapemode_init(void)
{
    _ls_tapemode = ls_tapemode_current();
}

enum ls_tapemode_mode ls_tapemode(void)
{
    return _ls_tapemode;
}

enum ls_tapemode_mode ls_tapemode_current(void)
{
    xSemaphoreTake(adc1_mux, portMAX_DELAY);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LSADC1_TAPESETTING, ADC_ATTEN_DB_12);
    uint32_t adc_reading = 0;
    for (int i = 0; i < 4; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_TAPESETTING);
    }
    adc_reading /= 4;
    xSemaphoreGive(adc1_mux);
    return ls_tapemode_from_adc(adc_reading);
}

void ls_tapemode_selftest_task(void *pvParameter)
{
    enum ls_tapemode_mode previous = ls_tapemode_current();
    int stable_count = 0;
    while (1)
    {
        enum ls_tapemode_mode current = ls_tapemode_current();
        if (current != previous)
        {
            if (stable_count >= 15)
            {
                stable_count = 0;
                previous = current;
                switch (current)
                {
                case LS_TAPEMODE_DARK_SAFE:
                    ls_buzzer_note(LS_BUZZER_SCALE_A, pdMS_TO_TICKS(500));
                    ls_event_enqueue(LSEVT_SELFTEST_MODE_DARKSAFE);
                    break;
                case LS_TAPEMODE_DARK:
                    ls_buzzer_note(LS_BUZZER_SCALE_G, pdMS_TO_TICKS(500));
                    ls_event_enqueue(LSEVT_SELFTEST_MODE_DARK);
                    break;
                case LS_TAPEMODE_IGNORE:
                    ls_buzzer_note(LS_BUZZER_SCALE_Fx, pdMS_TO_TICKS(500));
                    ls_event_enqueue(LSEVT_SELFTEST_MODE_IGNORE);
                    break;
                case LS_TAPEMODE_LIGHT:
                    ls_buzzer_note(LS_BUZZER_SCALE_E, pdMS_TO_TICKS(500));
                    ls_event_enqueue(LSEVT_SELFTEST_MODE_LIGHT);
                    break;
                case LS_TAPEMODE_LIGHT_SAFE:
                    ls_buzzer_note(LS_BUZZER_SCALE_D, pdMS_TO_TICKS(500));
                    ls_event_enqueue(LSEVT_SELFTEST_MODE_LIGHTSAFE);
                    break;
                default:; // don't play anything on return to selftest; we know that works or we wouldn't be here!
                }         // switch
            }  // if stable
            else
            {
                stable_count++;
            }
        }
        vTaskDelay(3);
    }
}

#ifdef LSDEBUG_TAPEMODE
void ls_tapemode_debug_task(void *pvParameter)
{
    while (1)
    {
        xSemaphoreTake(adc1_mux, portMAX_DELAY);
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(LSADC1_TAPESETTING, ADC_ATTEN_DB_12);
        uint32_t adc_reading = 0;
        for (int i = 0; i < 4; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_TAPESETTING);
        }
        adc_reading /= 4;
        xSemaphoreGive(adc1_mux);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Tape mode %d (raw: %d)\n", (int)ls_tapemode_from_adc(adc_reading), adc_reading);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif