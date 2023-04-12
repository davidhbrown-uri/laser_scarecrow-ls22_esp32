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
#include "controls.h"
#include "config.h"
#include "debug.h"
#include "events.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/semphr.h"
#include "esp_adc_cal.h"
#include "util.h"

extern SemaphoreHandle_t adc2_mux;
extern SemaphoreHandle_t print_mux;

static enum ls_controls_status ls_controls_current_status = LS_CONTROLS_STATUS_INVALID;
static BaseType_t _ls_controls_current_slider1 = -1;
static BaseType_t _ls_controls_current_slider2 = -1;
enum ls_controls_status _ls_controls_status(int adc)
{
    if (adc < LS_CONTROLS_SWITCH_THRESHOLD_UPPER)
    {
        return LS_CONTROLS_STATUS_OFF;
    }
    if (adc < LS_CONTROLS_SWITCH_THRESHOLD_LOWER)
    {
        return LS_CONTROLS_STATUS_UPPER;
    }
    if (adc < LS_CONTROLS_SWITCH_THRESHOLD_BOTH)
    {
        return LS_CONTROLS_STATUS_LOWER;
    }
    return LS_CONTROLS_STATUS_BOTH;
}

enum ls_controls_status ls_controls_get_current_status(void)
{
    return ls_controls_current_status;
}

#define _ls_controls_task_controls_havent_moved()                 \
    {                                                             \
        for (int i = 0; i < LS_CONTROLS_TASK_CONTROLS_COUNT; i++) \
            moved_control[i] = false;                             \
    }

int _ls_controls_get_averaged_adc_mv(adc2_channel_t controls_channel, adc_atten_t controls_atten, esp_adc_cal_characteristics_t *controls_adc_cal)
{
    int adc_reading = 0;
    uint32_t adc_sum = 0;
    xSemaphoreTake(adc2_mux, pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(adc2_config_channel_atten(controls_channel, controls_atten));
    for (int i = 0; i < LS_CONTROLS_READINGS_TO_AVERAGE; i++)
    {
        ESP_ERROR_CHECK(adc2_get_raw(controls_channel, ADC_WIDTH_12Bit, &adc_reading));
        adc_sum += (uint32_t)adc_reading;
    }
    xSemaphoreGive(adc2_mux);
    return esp_adc_cal_raw_to_voltage((int)(adc_sum / LS_CONTROLS_READINGS_TO_AVERAGE), controls_adc_cal);
}

void ls_controls_task(void *pvParameter)
{
#define LS_CONTROLS_TASK_CONTROLS_COUNT 3
#define LS_CONTROLS_TASK_SWITCH_READINGS 3
    uint8_t switch_reading = 0;
    adc2_channel_t controls_channels[] = {LSADC2_SWITCHES, LSADC2_SLIDER1, LSADC2_SLIDER2};
    adc_atten_t controls_atten[] = {LSADCATTEN_SWITCHES, LSADCATTEN_SLIDER, LSADCATTEN_SLIDER};
    esp_adc_cal_characteristics_t controls_adc_cal[LS_CONTROLS_TASK_CONTROLS_COUNT];
    uint32_t controls_readings[LS_CONTROLS_TASK_CONTROLS_COUNT];
    bool moved_control[LS_CONTROLS_TASK_CONTROLS_COUNT];
    _ls_controls_task_controls_havent_moved();
    int fastreads = 0;

    enum ls_controls_status switch_readings[LS_CONTROLS_TASK_SWITCH_READINGS];
    for (int i = 0; i < LS_CONTROLS_TASK_SWITCH_READINGS; i++)
    {
        switch_readings[i] = LS_CONTROLS_STATUS_INVALID;
        esp_adc_cal_characterize(ADC_UNIT_1, controls_atten[i], ADC_WIDTH_12Bit, 1100, &controls_adc_cal[i]);
    }

    while (1)
    {
        // read controls
        for (int control_number = 0; control_number < LS_CONTROLS_TASK_CONTROLS_COUNT; control_number++)
        {
            controls_readings[control_number] = _ls_controls_get_averaged_adc_mv(controls_channels[control_number], controls_atten[control_number], &controls_adc_cal[control_number]);
        }

#ifdef LSDEBUG_CONTROLS_VERBOSE
        ls_debug_printf("Controls:  switches=%dmV slider1=%dmV slider2=%dmV\n", controls_readings[0], controls_readings[1], controls_readings[2]);
#endif
        // update switch status

        switch_reading = (switch_reading + 1) % LS_CONTROLS_TASK_SWITCH_READINGS;
        switch_readings[switch_reading] = _ls_controls_status(esp_adc_cal_raw_to_voltage(controls_readings[0], &controls_adc_cal[0]));
        bool switch_readings_match = true;
        for (int i = 1; i < LS_CONTROLS_TASK_SWITCH_READINGS; i++)
        {
            switch_readings_match = switch_readings_match && switch_readings[i] == switch_readings[0];
        }
        if (switch_readings_match && ls_controls_current_status != switch_readings[0])
        {
#ifdef LSDEBUG_CONTROLS
            ls_debug_printf("Switches changed status from %d to %d\n", ls_controls_current_status, switch_readings[0]);
            ls_debug_printf("Controls:  switches=%dmV slider1=%dmV slider2=%dmV\n", controls_readings[0], controls_readings[1], controls_readings[2]);
#endif
            _ls_controls_current_slider1 = controls_readings[1];
            _ls_controls_current_slider2 = controls_readings[2];
            _ls_controls_task_controls_havent_moved();
            ls_controls_current_status = switch_readings[0];
            ls_event switch_event;
            switch_event.value = NULL;
            switch (ls_controls_current_status)
            {
            case LS_CONTROLS_STATUS_OFF:
                switch_event.type = LSEVT_CONTROLS_OFF;
                break;
            case LS_CONTROLS_STATUS_UPPER:
                switch_event.type = LSEVT_CONTROLS_UPPER;
                break;
            case LS_CONTROLS_STATUS_LOWER:
                switch_event.type = LSEVT_CONTROLS_LOWER;
                break;
            case LS_CONTROLS_STATUS_BOTH:
                switch_event.type = LSEVT_CONTROLS_BOTH;
                break;
            case LS_CONTROLS_STATUS_INVALID:
                switch_event.type = LSEVT_NOOP;
                break;
            }
            xQueueSendToBack(ls_event_queue, (void *)&switch_event, pdMS_TO_TICKS(1000));
        } // if switches changed

        if (ls_controls_get_current_status() != LS_CONTROLS_STATUS_OFF)
        {
            if (moved_control[1] || _difference_exceeds_threshold(_ls_controls_current_slider1, controls_readings[1], LS_CONTROLS_READING_MOVE_THRESHOLD))
            {
                if (!moved_control[1])
                {
                    fastreads = LS_CONTROLS_FASTREADS_AFTER_MOVE;
                }
                _ls_controls_task_controls_havent_moved();
                moved_control[1] = fastreads > 0;
                fastreads--;
                _ls_controls_current_slider1 = controls_readings[1];
                ls_event event;
                event.type = LSEVT_CONTROLS_SLIDER1;
                event.value = (void *)&_ls_controls_current_slider1;
                xQueueSendToBack(ls_event_queue, (void *)&event, 0);
#ifdef LSDEBUG_CONTROLS
                ls_debug_printf("Controls new value slider1=%dmV\n", _ls_controls_current_slider1);
#endif
            }
            if (moved_control[2] || _difference_exceeds_threshold(_ls_controls_current_slider2, controls_readings[2], LS_CONTROLS_READING_MOVE_THRESHOLD))
            {
                if (!moved_control[2])
                {
                    fastreads = LS_CONTROLS_FASTREADS_AFTER_MOVE;
                }
                _ls_controls_task_controls_havent_moved();
                moved_control[2] = fastreads > 0;
                fastreads--;
                _ls_controls_current_slider2 = controls_readings[2];
                ls_event event;
                event.type = LSEVT_CONTROLS_SLIDER2;
                event.value = (void *)&_ls_controls_current_slider2;
                xQueueSendToBack(ls_event_queue, (void *)&event, 0);
#ifdef LSDEBUG_CONTROLS
                ls_debug_printf("Controls new value slider2=%dmV\n", _ls_controls_current_slider2);
#endif
            }
        }

        vTaskDelay(ls_controls_current_status == LS_CONTROLS_STATUS_OFF ? pdMS_TO_TICKS(600) : pdMS_TO_TICKS(100));
    } // while 1
} // ls_controls_task