#include "controls.h"
#include "config.h"
#include "debug.h"
#include "events.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/semphr.h"
#include "esp_adc_cal.h"

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
static uint32_t ls_controls_current_speed = 0;
static uint32_t ls_controls_current_angle = 0;
static uint32_t ls_controls_current_range = 0;
enum ls_controls_status ls_controls_get_current_status(void)
{
    return ls_controls_current_status;
}

uint32_t ls_controls_get_speed(void)
{
    return ls_controls_current_speed;
}
uint32_t ls_controls_get_angle(void)
{
    return ls_controls_current_angle;
}
uint32_t ls_controls_get_range(void)
{
    return ls_controls_current_range;
}

void ls_controls_task(void *pvParameter)
{
#define LS_CONTROLS_TASK_KNOB_COUNT 4
#define LS_CONTROLS_TASK_CONNECTION_READINGS 3
    uint8_t connection_reading = 0;
    adc2_channel_t knob_channels[] = {LSADC2_KNOB3, LSADC2_KNOB4, LSADC2_KNOB5, LSADC2_KNOB6};
    uint32_t knob_readings[LS_CONTROLS_TASK_KNOB_COUNT];
    enum ls_controls_status connection_readings[LS_CONTROLS_TASK_CONNECTION_READINGS];
    for (int i = 0; i < LS_CONTROLS_TASK_CONNECTION_READINGS; i++)
    {
        connection_readings[i] = LS_CONTROLS_STATUS_INVALID;
    }
    while (1)
    {
        for (int knob = 0; knob < LS_CONTROLS_TASK_KNOB_COUNT; knob++)
        {
            xSemaphoreTake(adc2_mux, pdMS_TO_TICKS(1000));
            int adc_reading = 0;
            uint32_t adc_sum = 0;
            // atten 11 by default... shouldn't need to focus on lower voltages?
            ESP_ERROR_CHECK(adc2_config_channel_atten(knob_channels[knob], ADC_ATTEN_11db));
            for (int i = 0; i < 4; i++)
            {
                ESP_ERROR_CHECK(adc2_get_raw(knob_channels[knob], ADC_WIDTH_12Bit, &adc_reading));
                adc_sum += adc_reading;
            }
            knob_readings[knob] = adc_sum / 4;
            xSemaphoreGive(adc2_mux);
            vTaskDelay(2); // yield some time to other tasks... we're in no particular hurry
        }
        // update status
        connection_reading = (connection_reading+1) % LS_CONTROLS_TASK_CONNECTION_READINGS;
        if (knob_readings[3] < LS_CONTROLS_ADC_MAX_DISCONNECT)
        {
            connection_readings[connection_reading] = LS_CONTROLS_STATUS_DISCONNECTED;
        }
        else if (knob_readings[3] > LS_CONTROLS_ADC_MIN_CONNECT && knob_readings[3] < LS_CONTROLS_ADC_MAX_CONNECT)
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
                    xSemaphoreTake(print_mux, portMAX_DELAY);
                    printf("Controls status CONNECTED\n");
                    xSemaphoreGive(print_mux);
#endif
                    break;
                case LS_CONTROLS_STATUS_DISCONNECTED:
                    connection_event.type = LSEVT_CONTROLS_DISCONNECTED;
#ifdef LSDEBUG_CONTROLS
                    xSemaphoreTake(print_mux, portMAX_DELAY);
                    printf("Controls status DISCONNECTED\n");
                    xSemaphoreGive(print_mux);
#endif
                    break;
                case LS_CONTROLS_STATUS_INVALID:
                    connection_event.type = LSEVT_CONTROLS_DISCONNECTED;
#ifdef LSDEBUG_CONTROLS
                    xSemaphoreTake(print_mux, portMAX_DELAY);
                    printf("Controls status INVALID\n");
                    xSemaphoreGive(print_mux);
#endif
                    break;
                }
                xQueueSendToBack(ls_event_queue, (void *)&connection_event, pdMS_TO_TICKS(1000));
            }
        }
#ifdef LSDEBUG_CONTROLS
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("External control knobs: %d\t %d\t %d\t %d\n", knob_readings[0], knob_readings[1], knob_readings[2], knob_readings[3]);
        xSemaphoreGive(print_mux);
#endif
        if (ls_controls_get_current_status() == LS_CONTROLS_STATUS_CONNECTED)
        {
            ls_controls_current_speed = knob_readings[0];
            ls_controls_current_angle = knob_readings[1];
            ls_controls_current_range = knob_readings[2];
        }
        vTaskDelay(ls_controls_current_status == LS_CONTROLS_STATUS_CONNECTED ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(600));
    }
}