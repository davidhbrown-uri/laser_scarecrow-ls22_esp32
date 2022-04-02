#include "tapemode.h"
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

enum ls_tapemode ls_tapemode_get(void)
{
    xSemaphoreTake(adc1_mux, pdMS_TO_TICKS(1000));
    xSemaphoreTake(adc1_mux, portMAX_DELAY);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LSADC1_TAPESETTING, ADC_ATTEN_DB_11);
    uint32_t adc_reading = 0;
    for (int i = 0; i < 4; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_TAPESETTING);
    }
    adc_reading /= 4;
    if (adc_reading > LS_TAPEMODE_THRESHOLD_5)
    {
        return LS_TAPEMODE_REFLECT_SAFE;
    }
    if (adc_reading > LS_TAPEMODE_THRESHOLD_4)
    {
        return LS_TAPEMODE_REFLECT;
    }
    if (adc_reading > LS_TAPEMODE_THRESHOLD_3)
    {
        return LS_TAPEMODE_IGNORE;
    }
    if (adc_reading > LS_TAPEMODE_THRESHOLD_2)
    {
        return LS_TAPEMODE_BLACK;
    }
    if (adc_reading > LS_TAPEMODE_THRESHOLD_1)
    {
        return LS_TAPEMODE_BLACK_SAFE;
    }
    return LS_TAPEMODE_SELFTEST;
}

#ifdef LSDEBUG_TAPEMODE
void ls_tapemode_debug_task(void *pvParameter)
{
    while (1)
    {
        xSemaphoreTake(adc1_mux, portMAX_DELAY);
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(LSADC1_TAPESETTING, ADC_ATTEN_DB_11);
        uint32_t adc_reading = 0;
        for (int i = 0; i < 4; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_TAPESETTING);
        }
        adc_reading /= 4;
        xSemaphoreGive(adc1_mux);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Tape setting raw: %d\n", adc_reading);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif