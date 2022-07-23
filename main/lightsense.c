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
#include "lightsense.h"
#include "config.h"
#include "debug.h"
#include "events.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/semphr.h"
#include "esp_adc_cal.h"
#include "settings.h"
#include "states.h"

extern SemaphoreHandle_t adc1_mux;
extern SemaphoreHandle_t print_mux;

static enum ls_lightsense_mode_t _ls_lightsense_current_mode = LS_LIGHTSENSE_MODE_STARTUP;
enum ls_lightsense_mode_t ls_lightsense_current_mode(void)
{
    return _ls_lightsense_current_mode;
}

static enum ls_lightsense_level_t _ls_lightsense_level_from_adc(uint32_t adc_reading)
{
    if (adc_reading >= ls_settings_get_light_threshold_on())
    {
        return LS_LIGHTSENSE_LEVEL_DAY;
    }
    if (adc_reading <= ls_settings_get_light_threshold_off())
    {
        return LS_LIGHTSENSE_LEVEL_NIGHT;
    }
    return LS_LIGHTSENSE_LEVEL_INDETERMINATE;
}

int ls_lightsense_read_adc(void)
{
    xSemaphoreTake(adc1_mux, portMAX_DELAY);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LSADC1_LIGHTSENSE, ADC_ATTEN_DB_11);
    uint32_t adc_reading = 0;
    for (int i = 0; i < 4; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_LIGHTSENSE);
    }
    adc_reading /= 4;
    xSemaphoreGive(adc1_mux);
    return (int)adc_reading;
}
/**
 * @brief Set current mode if event can be queued; otherwise let it try again next read
 *
 * @param mode
 */
static void _ls_lightsense_set_mode(enum ls_lightsense_mode_t mode)
{
    ls_event lightsense_event;
    lightsense_event.value = NULL;
    switch (mode)
    {
    case LS_LIGHTSENSE_MODE_NIGHT:
        lightsense_event.type = LSEVT_LIGHT_NIGHT;
        break;
    case LS_LIGHTSENSE_MODE_DAY:
        lightsense_event.type = LSEVT_LIGHT_DAY;
        break;
    default:
        return; // without doing anything
    }
#ifdef LSDEBUG_LIGHTSENSE
    ls_debug_printf("Lightsense queueing event=%d for mode=%d\n", (int)lightsense_event.type, (int)mode);
#endif
    if (pdTRUE == xQueueSendToBack(ls_event_queue, (void *)&lightsense_event, 0))
    {
        _ls_lightsense_current_mode = mode; // queued event, so set the mode
    }
    else
    {
        ; // avoid syntax error if not def LSDEBUG_LIGHTSENSE
#ifdef LSDEBUG_LIGHTSENSE
        ls_debug_printf("Lightsense could not queue event\n");
#endif
    }
}

void ls_lightsense_read_task(void *pvParameter)
{
    enum ls_lightsense_level_t levels[LS_LIGHTSENSE_READINGS_TO_SWITCH];
    for (int i = 0; i < LS_LIGHTSENSE_READINGS_TO_SWITCH; i++)
    {
        levels[i] = LS_LIGHTSENSE_LEVEL_INDETERMINATE;
    }
    int level_index = 0;
    while (1)
    {
        int adc_reading = ls_lightsense_read_adc();
        levels[level_index] = _ls_lightsense_level_from_adc(adc_reading);
#ifdef LSDEBUG_LIGHTSENSE
        ls_debug_printf("Light sense adc=%d; level=%d\n", adc_reading, (int)levels[level_index]);
#endif
        bool all_agree = true;
        for (int i = 1; all_agree && i < LS_LIGHTSENSE_READINGS_TO_SWITCH; i++)
        {
            if (levels[i] != levels[i - 1])
            {
                all_agree = false;
            }
        }
        if (all_agree)
        {
            switch (levels[0])
            {
            case LS_LIGHTSENSE_LEVEL_DAY:
                if (ls_lightsense_current_mode() != LS_LIGHTSENSE_MODE_DAY)
                {
                    _ls_lightsense_set_mode(LS_LIGHTSENSE_MODE_DAY);
                }
                break;
            case LS_LIGHTSENSE_LEVEL_NIGHT:
                if (ls_lightsense_current_mode() != LS_LIGHTSENSE_MODE_NIGHT ||
                (ls_state_current.func != ls_state_sleep && ls_state_current.func != ls_state_settings))
                {
                    _ls_lightsense_set_mode(LS_LIGHTSENSE_MODE_NIGHT);
                }
                break;
            default:; // do not set mode for indeterminate or invalid levels
            }
        }
        // move to next level reading
        if (++level_index > LS_LIGHTSENSE_READINGS_TO_SWITCH)
        {
            level_index = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(LS_LIGHTSENSE_READING_INTERVAL_MS));
    }
}