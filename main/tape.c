#include "tape.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"

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
    adc1_config_channel_atten(LSADC1_TAPESETTING, ADC_ATTEN_DB_11);
    uint32_t adc_reading = 0;
    for (int i = 0; i < 4; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_TAPESETTING);
    }
    adc_reading /= 4;
    xSemaphoreGive(adc1_mux);
    if(was_disabled)
    {
        ls_tape_sensor_disable();
    }
    return adc_reading;
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