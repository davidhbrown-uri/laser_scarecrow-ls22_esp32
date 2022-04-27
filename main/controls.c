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

// "Knobs" is what's printed next to the external control connector of the January '22 boards,
// referencing rotary potentiometers used on earlier designs. The external controls are
// linear sliders in this design, but I still think of them as "knobs."" Sorry. -DHB

#define LS_CONTROLS_WIGGLE_KNOB 0
#define LS_CONTROLS_ANGLE_KNOB 1
#define LS_CONTROLS_RANGE_KNOB 2
#define LS_CONTROLS_CONNECTION_KNOB LSADC_KNOB6

static enum ls_controls_status ls_controls_current_status = LS_CONTROLS_STATUS_INVALID;
static BaseType_t _ls_controls_current_speed = 0;
static BaseType_t _ls_controls_current_topangle = 0;
static BaseType_t _ls_controls_current_bottomangle = 0;
#define _ls_controls_connected(adc) (adc > LS_CONTROLS_ADC_MIN_CONNECT && adc < LS_CONTROLS_ADC_MAX_CONNECT)
enum ls_controls_status ls_controls_get_current_status(void)
{
    return ls_controls_current_status;
}

#define _ls_controls_task_knobs_havent_moved()                                    \
    {                                                                             \
        for (int i = 0; i < LS_CONTROLS_TASK_KNOB_COUNT; moved_knob[i++] = false) \
            ;                                                                     \
    }
void ls_controls_task(void *pvParameter)
{
#define LS_CONTROLS_TASK_KNOB_COUNT 4
#define LS_CONTROLS_TASK_CONNECTION_READINGS 3
    uint8_t connection_reading = 0;
    adc2_channel_t knob_channels[] = {LSADC2_KNOB3, LSADC2_KNOB4, LSADC2_KNOB5, LSADC2_KNOB6};
    uint32_t knob_readings[LS_CONTROLS_TASK_KNOB_COUNT];
    bool moved_knob[LS_CONTROLS_TASK_KNOB_COUNT];
    _ls_controls_task_knobs_havent_moved();
    int fastreads = 0;
    enum ls_controls_status connection_readings[LS_CONTROLS_TASK_CONNECTION_READINGS];
    for (int i = 0; i < LS_CONTROLS_TASK_CONNECTION_READINGS; i++)
    {
        connection_readings[i] = LS_CONTROLS_STATUS_INVALID;
    }
    while (1)
    {
        // read knobs
        xSemaphoreTake(adc2_mux, pdMS_TO_TICKS(1000));
        for (int knob = 0; knob < LS_CONTROLS_TASK_KNOB_COUNT; knob++)
        {
            int adc_reading = 0;
            uint32_t adc_sum = 0;
            // atten 11 by default... shouldn't need to focus on lower voltages?
            ESP_ERROR_CHECK(adc2_config_channel_atten(knob_channels[knob], ADC_ATTEN_11db));
            for (int i = 0; i < LS_CONTROLS_READINGS_TO_AVERAGE; i++)
            {
                ESP_ERROR_CHECK(adc2_get_raw(knob_channels[knob], ADC_WIDTH_12Bit, &adc_reading));
                adc_sum += adc_reading;
            }
            knob_readings[knob] = adc_sum / LS_CONTROLS_READINGS_TO_AVERAGE;
        }
        xSemaphoreGive(adc2_mux);

        // update status
        connection_reading = (connection_reading + 1) % LS_CONTROLS_TASK_CONNECTION_READINGS;
        if (knob_readings[3] < LS_CONTROLS_ADC_MAX_DISCONNECT)
        {
            connection_readings[connection_reading] = LS_CONTROLS_STATUS_DISCONNECTED;
        }
        else if (_ls_controls_connected(knob_readings[3]))
        {
            connection_readings[connection_reading] = LS_CONTROLS_STATUS_CONNECTED;
        }
        else
        {
            connection_readings[connection_reading] = LS_CONTROLS_STATUS_INVALID;
        }
        if ((connection_readings[0] == connection_readings[1]) && (connection_readings[0] == connection_readings[2]))
        {
            if (ls_controls_current_status != connection_readings[0])
            {
                ls_controls_current_status = connection_readings[0];
                ls_event connection_event;
                connection_event.value = NULL;
                switch (ls_controls_current_status)
                {
                case LS_CONTROLS_STATUS_CONNECTED:
                    connection_event.type = LSEVT_CONTROLS_CONNECTED;
#ifdef LSDEBUG_CONTROLS
                    ls_debug_printf("Controls status CONNECTED\n");
#endif
                    _ls_controls_current_speed = knob_readings[0];
                    _ls_controls_current_topangle = knob_readings[1];
                    _ls_controls_current_bottomangle = knob_readings[2];
                    _ls_controls_task_knobs_havent_moved();
                    break;
                case LS_CONTROLS_STATUS_DISCONNECTED:
                    connection_event.type = LSEVT_CONTROLS_DISCONNECTED;
#ifdef LSDEBUG_CONTROLS
                    ls_debug_printf("Controls status DISCONNECTED\n");
#endif
                    break;
                case LS_CONTROLS_STATUS_INVALID:
                    connection_event.type = LSEVT_CONTROLS_DISCONNECTED;
#ifdef LSDEBUG_CONTROLS
                    ls_debug_printf("Controls status INVALID\n");
#endif
                    break;
                }
                xQueueSendToBack(ls_event_queue, (void *)&connection_event, pdMS_TO_TICKS(1000));
            }
        }
#ifdef LSDEBUG_CONTROLS
        ls_debug_printf("External control knobs: %d\t %d\t %d\t %d\n", knob_readings[0], knob_readings[1], knob_readings[2], knob_readings[3]);
#endif
        if (ls_controls_get_current_status() == LS_CONTROLS_STATUS_CONNECTED && _ls_controls_connected(knob_readings[3]))
        {
            if (moved_knob[0] || _difference_exceeds_threshold(_ls_controls_current_speed, knob_readings[0], LS_CONTROLS_READING_MOVE_THRESHOLD))
            {
                if (!moved_knob[0])
                {
                    fastreads = LS_CONTROLS_FASTREADS_AFTER_MOVE;
                }
                _ls_controls_task_knobs_havent_moved();
                moved_knob[0] = (fastreads-- > 0 ? true : false);
                _ls_controls_current_speed = knob_readings[0];
                ls_event event;
                event.type = LSEVT_CONTROLS_SPEED;
                event.value = (void *)&_ls_controls_current_speed;
                xQueueSendToBack(ls_event_queue, (void *)&event, 0);
#ifdef LSDEBUG_CONTROLS
                ls_debug_printf("Controls new value speed=%d\n", _ls_controls_current_speed);
#endif
            }
            if (moved_knob[1] || _difference_exceeds_threshold(_ls_controls_current_topangle, knob_readings[1], LS_CONTROLS_READING_MOVE_THRESHOLD))
            {
                _ls_controls_task_knobs_havent_moved();
                moved_knob[1] = (fastreads-- > 0 ? true : false);
                _ls_controls_current_topangle = knob_readings[1];
                ls_event event;
                event.type = LSEVT_CONTROLS_TOPANGLE;
                event.value = (void *)&_ls_controls_current_topangle;
                xQueueSendToBack(ls_event_queue, (void *)&event, 0);
#ifdef LSDEBUG_CONTROLS
                ls_debug_printf("Controls new value topangle=%d\n", _ls_controls_current_topangle);
#endif
            }
            if (moved_knob[2] || _difference_exceeds_threshold(_ls_controls_current_bottomangle, knob_readings[2], LS_CONTROLS_READING_MOVE_THRESHOLD))
            {
                _ls_controls_task_knobs_havent_moved();
                moved_knob[2] = (fastreads-- > 0 ? true : false);
                _ls_controls_current_bottomangle = knob_readings[2];
                ls_event event;
                event.type = LSEVT_CONTROLS_BOTTOMANGLE;
                event.value = (void *)&_ls_controls_current_bottomangle;
                xQueueSendToBack(ls_event_queue, (void *)&event, 0);
#ifdef LSDEBUG_CONTROLS
                ls_debug_printf("Controls new value bottomangle=%d\n", _ls_controls_current_bottomangle);
#endif
            }
        }
        vTaskDelay(ls_controls_current_status == LS_CONTROLS_STATUS_CONNECTED ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(600));
    }
}