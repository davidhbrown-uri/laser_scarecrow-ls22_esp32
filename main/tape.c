#include "tape.h"
#include "config.h"
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
        if(tape_reading < LS_REFLECTANCE_ADC_MAX_WHITE_BUCKET && last_tape_reading >= LS_REFLECTANCE_ADC_MAX_WHITE_BUCKET)
        {
            ls_buzzer_effect(LS_BUZZER_PLAY_ROOT);
        }
        if(tape_reading > LS_REFLECTANCE_ADC_MIN_BLACK_BUCKET && last_tape_reading <= LS_REFLECTANCE_ADC_MIN_BLACK_BUCKET)
        {
            ls_buzzer_effect(LS_BUZZER_PLAY_OCTAVE);
        }
        last_tape_reading = tape_reading;
        vTaskDelay(1);
    }
}