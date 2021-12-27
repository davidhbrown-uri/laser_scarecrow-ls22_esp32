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
#include "mpu6050.h"

#include "config.h"

SemaphoreHandle_t adc1_mux = NULL;
SemaphoreHandle_t i2c_mux = NULL;
SemaphoreHandle_t print_mux = NULL;

static esp_adc_cal_characteristics_t *adc_chars;

void adc_read_tape_setting_task(void *pvParameter)
{
  while(1)
  {
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("Check tape setting jumpers...\n");
    xSemaphoreGive(print_mux);
    xSemaphoreTake(adc1_mux, portMAX_DELAY);
    adc1_config_width(ADC_WIDTH_BIT_12); 
    adc1_config_channel_atten(LSADC1_TAPESETTING, ADC_ATTEN_DB_11);
    uint32_t adc_reading = 0;
    for(int i=0; i<4; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t) LSADC1_TAPESETTING);
    }
    adc_reading /= 4;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    xSemaphoreGive(adc1_mux);
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("Tape setting raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    xSemaphoreGive(print_mux);
    vTaskDelay(pdMS_TO_TICKS(3000));
   }
}

void adc_read_light_sensor_task(void *pvParameter)
{
  while(1)
  {
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("Check ambient light sensor...\n");
    xSemaphoreGive(print_mux);
    xSemaphoreTake(adc1_mux, pdMS_TO_TICKS(1000));
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
    xSemaphoreGive(adc1_mux);
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("Ambient light raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    xSemaphoreGive(print_mux);
    vTaskDelay(pdMS_TO_TICKS(1000));
   }
}

void i2c_read_tilt_task(void *pvParameter)
{
    while(1)
    {
    xSemaphoreTake(i2c_mux, portMAX_DELAY);
    float tilt = mpu6050_read_accel_z();
    xSemaphoreGive(i2c_mux);
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("Tilt (AccelZ)= %0.2fg\n", tilt);
    xSemaphoreGive(print_mux);
    vTaskDelay(pdMS_TO_TICKS(400));
    }
}

void i2c_read_temp_task(void *pvParameter)
{
    while(1)
    {
    xSemaphoreTake(i2c_mux, portMAX_DELAY);
    float temp = mpu6050_read_temp();
    xSemaphoreGive(i2c_mux);
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("Temperature= %0.1f°C\n", temp);
    xSemaphoreGive(print_mux);
    vTaskDelay(pdMS_TO_TICKS(5400));
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

    printf("Checked ADC efuse (wait 1s)\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	adc1_mux = xSemaphoreCreateMutex();
	i2c_mux = xSemaphoreCreateMutex();
    print_mux = xSemaphoreCreateMutex();
    printf("Initializing GPIO\n");
    lsgpio_initialize();
    printf("Initialized GPIO (wait 1s)\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Initializing I2C\n");
    ESP_ERROR_CHECK(lsi2c_master_init());
    printf("Initialized I2C (wait 1s)\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Initializing MPU6050\n");
    mpu6050_begin();
    printf("Initialized MPU6050 i2c device (wait 2s)\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
    // so, do you just have to figure out the usStackDepth parameter (here, configMINIMAL_STACK_SIZE*2) by trial and error?
    xTaskCreate(&adc_read_tape_setting_task, "adcr_tapesetting", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL);
    xTaskCreate(&adc_read_light_sensor_task, "adcr_light", configMINIMAL_STACK_SIZE*3, NULL, 1, NULL);
    xTaskCreate(&i2c_read_tilt_task, "i2c_tilt", configMINIMAL_STACK_SIZE*3, NULL, 1, NULL);
    xTaskCreate(&i2c_read_temp_task, "i2c_temp", configMINIMAL_STACK_SIZE*3, NULL, 1, NULL);
}
