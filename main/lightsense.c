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
#include "util.h"
#include "tapemode.h"
#ifdef LSDEBUG_LIGHTSENSE
#include "oled.h"
#endif

extern SemaphoreHandle_t adc1_mux;
extern SemaphoreHandle_t print_mux;

static enum ls_lightsense_mode_t _ls_lightsense_current_mode = LS_LIGHTSENSE_MODE_STARTUP;

int ls_lightsense_threshold_on_mv(int index)
{
    index = _constrain(index, 0, LS_LIGHTSENSE_THRESHOLDS_COUNT - 1);
    return ((int[]){ LS_LIGHTSENSE_THRESHOLDS_ON_MV })[index];
}
int ls_lightsense_threshold_off_mv(int index)
{
    index = _constrain(index, 0, LS_LIGHTSENSE_THRESHOLDS_COUNT - 1);
    return ((int[]){ LS_LIGHTSENSE_THRESHOLDS_OFF_MV })[index];
}

enum ls_lightsense_mode_t ls_lightsense_current_mode(void)
{
    return _ls_lightsense_current_mode;
}

static enum ls_lightsense_level_t _ls_lightsense_level_from_adc(int adc_reading)
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

/**
 * @brief Returns value in mv at requested attenuation using best available calibration
 * 
 * @param channel to read
 * @param attenuation to use
 */
int ls_lightsense_read_adc_channel(adc1_channel_t channel, adc_atten_t attenuation)
{
    xSemaphoreTake(adc1_mux, portMAX_DELAY);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, attenuation);
    int sum = 0;
    for (int i = 0; i < LS_LIGHTSENSE_READINGS_TO_AVERAGE; i++)
    {
        sum += adc1_get_raw( channel );
    }
    xSemaphoreGive(adc1_mux);
    esp_adc_cal_characteristics_t adc_cal;
    esp_adc_cal_characterize(ADC_UNIT_1, attenuation, ADC_WIDTH_12Bit, 1100, &adc_cal);
    return (int)esp_adc_cal_raw_to_voltage(sum / LS_LIGHTSENSE_READINGS_TO_AVERAGE, &adc_cal);
}

/**
 * @brief ADC of channel in mV, using either 0dB or 11dB attenuation if @0dB is > 900 mV
 * 
 * @param channel to read
 */
int ls_lightsense_read_channel_hdr(adc1_channel_t channel)
{
    int mV = ls_lightsense_read_adc_channel(channel, ADC_ATTEN_0db);
    if (mV > 900)
    {
        mV = ls_lightsense_read_adc_channel(channel, ADC_ATTEN_11db);
    }
    return mV;
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

        int adc_reading1 = ls_lightsense_read_channel_hdr(LSADC1_LIGHTSENSE);
        levels[level_index] = _ls_lightsense_level_from_adc(adc_reading1);
#ifdef LS_HAS_LIGHTSENSE2
        int adc_reading2 = ls_lightsense_read_channel_hdr(LSADC1_LIGHTSENSE2);
        int level2 = _ls_lightsense_level_from_adc(adc_reading2);
        levels[level_index] = min(level2, levels[level_index]);
#endif 
#ifdef LSDEBUG_LIGHTSENSE
#ifdef LS_HAS_LIGHTSENSE2
        ls_debug_printf("Light sense 1=%dmV, 2=%dmV; level=%d\n", adc_reading1, adc_reading2, levels[level_index]);
        ls_oled_println("%dmV (%d)", min(adc_reading1, adc_reading2), levels[level_index]);
#else
        ls_debug_printf("Light sense %dmV; level=%d\n", adc_reading1, levels[level_index]);
        ls_oled_println("%d mV (%d)", adc_reading1, levels[level_index]);
#endif
#endif

        bool all_agree = true;
        for (int i = 1; all_agree && i < LS_LIGHTSENSE_READINGS_TO_SWITCH; i++)
        {
            if (levels[i] != levels[i - 1])
            {
                all_agree = false;
            }
        }
        if (all_agree || ls_tapemode() == LS_TAPEMODE_SELFTEST)
        {
            switch (levels[level_index])
            {
            case LS_LIGHTSENSE_LEVEL_DAY:
                if (ls_lightsense_current_mode() != LS_LIGHTSENSE_MODE_DAY)
                {
                    _ls_lightsense_set_mode(LS_LIGHTSENSE_MODE_DAY);
                }
                break;
            case LS_LIGHTSENSE_LEVEL_NIGHT:
                if (ls_lightsense_current_mode() != LS_LIGHTSENSE_MODE_NIGHT ||
                    (ls_state_current.func != ls_state_sleep && ls_state_current.func != ls_state_settings_upper && ls_state_current.func != ls_state_settings_lower && ls_state_current.func != ls_state_settings_both))
                {
                    _ls_lightsense_set_mode(LS_LIGHTSENSE_MODE_NIGHT);
                }
                break;
            default:; // do not set mode for indeterminate or invalid levels
            }
        }
        // move to next level reading
        level_index++;
        level_index %= LS_LIGHTSENSE_READINGS_TO_SWITCH;
        vTaskDelay(pdMS_TO_TICKS(LS_LIGHTSENSE_READING_INTERVAL_MS));
    }
}