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
#include "config.h"
#include "init.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define LSGPIO_OUTPUT_PIN_SEL (\
    (1ULL<<LSGPIO_LASERPOWERENABLE) | \
    (1ULL<<LSGPIO_LASERHEATERENABLE) | \
    (1ULL<<LSGPIO_SERVOPOWERENABLE) | \
    (1ULL<<LSGPIO_REFLECTANCEENABLE) | \
    (1ULL<<LSGPIO_STEPPERENABLE) | \
    (1ULL<<LSGPIO_STEPPERSTEP) | \
    (1ULL<<LSGPIO_STEPPERDIRECTION) | \
    (1ULL<<LSGPIO_BUZZERENABLE) \
    )

#define LSGPIO_INPUT_PIN_SEL (\
    (1ULL<<LSGPIO_SLIDER1) | \
    (1ULL<<LSGPIO_SLIDER2) | \
    (1ULL<<LSGPIO_SWITCHES) | \
    (1ULL<<LSGPIO_LASERTEMP) | \
    (1ULL<<LSGPIO_LIGHTSENSE) | \
    (1ULL<<LSGPIO_MAGNETSENSE) | \
    (1ULL<<LSGPIO_REFLECTANCESENSE) | \
    (1ULL<<LSGPIO_TAPESETTING) \
)


void ls_gpio_initialize(void)
{
    // a structure to hold all the GPIO configuration data
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = LSGPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    // ... and turn off (set low) the outputs
    gpio_set_level(LSGPIO_LASERPOWERENABLE, 0);
    gpio_set_level(LSGPIO_LASERHEATERENABLE, 0);
    gpio_set_level(LSGPIO_REFLECTANCEENABLE, 0);
    gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_DISABLE); //TMS2209 Enable is active low
    // ... might as well set these low, too, just for consistency
    gpio_set_level(LSGPIO_STEPPERDIRECTION, 0);
    gpio_set_level(LSGPIO_STEPPERSTEP, 0);
    gpio_set_level(LSGPIO_BUZZERENABLE, 0);
    gpio_set_level(LSGPIO_SERVOPULSE, 0);

    // now our inputs
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = LSGPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

// from adc1_example_main.c
void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    // Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}
