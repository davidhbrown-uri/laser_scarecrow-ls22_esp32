#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "config.h"

SemaphoreHandle_t ls_adc1_semaphore = NULL;
static esp_adc_cal_characteristics_t *adc_chars;

void adc_read_tape_setting_task(void *pvParameter)
{
  while(1)
  {
    printf("Check tape setting jumpers...\n");
    xSemaphoreTake(ls_adc1_semaphore, pdMS_TO_TICKS(1000));
    adc1_config_width(ADC_WIDTH_BIT_12); 
    adc1_config_channel_atten(LSADC1_TAPESETTING, ADC_ATTEN_DB_11);
    uint32_t adc_reading = 0;
    for(int i=0; i<4; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t) LSADC1_TAPESETTING);
    }
    xSemaphoreGive(ls_adc1_semaphore);
    adc_reading /= 4;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    printf("Tape setting raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    vTaskDelay(pdMS_TO_TICKS(3000));
   }
}

void adc_read_light_sensor_task(void *pvParameter)
{
  while(1)
  {
    printf("Check ambient light sensor...\n");
    xSemaphoreTake(ls_adc1_semaphore, pdMS_TO_TICKS(1000));
    adc1_config_width(ADC_WIDTH_BIT_12); 
    // atten 11 for office use... probably 0 in field
    adc1_config_channel_atten(LSADC1_LIGHTSENSE, ADC_ATTEN_11db); 
    uint32_t adc_reading = 0;
    for(int i=0; i<4; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t) LSADC1_LIGHTSENSE);
    }
    adc_reading /= 4;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    printf("Ambient light raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    xSemaphoreGive(ls_adc1_semaphore);
    vTaskDelay(pdMS_TO_TICKS(1000));
   }
}

// from adc1_example_main.c
static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void app_main(void)
{
    check_efuse();
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
    print_char_val_type(val_type);

    printf("Checked ADC efuse (wait 5s)\n");
    vTaskDelay(pdMS_TO_TICKS(5000));
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	ls_adc1_semaphore = xSemaphoreCreateMutex();
    lsgpio_initialize();
    printf("Initialized GPIO (wait 5s)\n");
    vTaskDelay(pdMS_TO_TICKS(5000));
    // so, do you just have to figure out the usStackDepth parameter (here, configMINIMAL_STACK_SIZE*2) by trial and error?
    xTaskCreate(&adc_read_tape_setting_task, "adcr_tapesetting", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL);
    xTaskCreate(&adc_read_light_sensor_task, "adcr_light", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL);
}
