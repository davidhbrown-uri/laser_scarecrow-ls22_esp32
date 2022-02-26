#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_adc_cal.h"
#include "config.h"
#include "init.h"
#include "mpu6050.h"
#include "events.h"
#include "buzzer.h"
#include "stepper.h"
#include "magnet.h"
#include "states.h"


SemaphoreHandle_t adc1_mux = NULL;
SemaphoreHandle_t i2c_mux = NULL;
SemaphoreHandle_t print_mux = NULL;

static esp_adc_cal_characteristics_t *adc_chars;

void adc_read_tape_setting_task(void *pvParameter)
{
    while (1)
    {
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Check tape setting jumpers...\n");
        xSemaphoreGive(print_mux);
        xSemaphoreTake(adc1_mux, portMAX_DELAY);
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(LSADC1_TAPESETTING, ADC_ATTEN_DB_11);
        uint32_t adc_reading = 0;
        for (int i = 0; i < 4; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_TAPESETTING);
        }
        adc_reading /= 4;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        xSemaphoreGive(adc1_mux);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Tape setting raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void adc_read_light_sensor_task(void *pvParameter)
{
    while (1)
    {
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Check ambient light sensor...\n");
        xSemaphoreGive(print_mux);
        xSemaphoreTake(adc1_mux, pdMS_TO_TICKS(1000));
        adc1_config_width(ADC_WIDTH_BIT_12);
        // atten 11 for office use... probably 0 in field
        adc1_config_channel_atten(LSADC1_LIGHTSENSE, ADC_ATTEN_11db);
        uint32_t adc_reading = 0;
        for (int i = 0; i < 4; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_LIGHTSENSE);
        }
        adc_reading /= 4;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        xSemaphoreGive(adc1_mux);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Ambient light raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void i2c_read_tilt_task(void *pvParameter)
{
    while (1)
    {
        xSemaphoreTake(i2c_mux, portMAX_DELAY);
        float tilt = mpu6050_read_accel_z();
        xSemaphoreGive(i2c_mux);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Tilt (AccelZ)= %0.2fg\n", tilt);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void i2c_read_temp_task(void *pvParameter)
{
    while (1)
    {
        xSemaphoreTake(i2c_mux, portMAX_DELAY);
        float temp = mpu6050_read_temp();
        xSemaphoreGive(i2c_mux);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Temperature= %0.1f°C\n", temp);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/**
 * @brief White bucket reads <<500 (usually closer to 200) and black tape reads >>2000 (usually closer to 3000)
 *
 * @param pvParameter
 */
void adc_read_reflectance_sensor_task(void *pvParameter)
{
    while (1)
    {
        // turn on the reflectance sensor (no additional delay was needed)
        gpio_set_level(LSGPIO_REFLECTANCEENABLE, 1);
        xSemaphoreTake(adc1_mux, pdMS_TO_TICKS(1000));
        adc1_config_width(ADC_WIDTH_BIT_12);
        // atten 11 by default... shouldn't need to focus on lower voltages?
        adc1_config_channel_atten(LSADC1_REFLECTANCESENSE, ADC_ATTEN_11db);
        uint32_t adc_reading = 0;
        for (int i = 0; i < 4; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_REFLECTANCESENSE);
        }
        adc_reading /= 4;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        xSemaphoreGive(adc1_mux);
        // turn off the reflectance sensor
        gpio_set_level(LSGPIO_REFLECTANCEENABLE, 0);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Reflectance sensor raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}



static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

void app_main(void)
{
    ls_event_queue_init();
    check_efuse();
    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
    print_char_val_type(val_type);
    printf("Checked ADC efuse\n");
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc1_mux = xSemaphoreCreateMutex();
    print_mux = xSemaphoreCreateMutex();
    printf("Initializing GPIO\n");
    ls_gpio_initialize();
    printf("Initialized GPIO\n");
    /*
    printf("Initializing I2C\n");
    i2c_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(lsi2c_master_init());
    printf("Initialized I2C\n");
    printf("Initializing MPU6050\n");
    mpu6050_begin();
    printf("Initialized MPU6050 i2c device\n");
    // so, do you just have to figure out the usStackDepth parameter (here, configMINIMAL_STACK_SIZE*2) by trial and error?
    xTaskCreate(&adc_read_tape_setting_task, "adcr_tapesetting", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&adc_read_light_sensor_task, "adcr_light", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&i2c_read_tilt_task, "i2c_tilt", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&i2c_read_temp_task, "i2c_temp", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);

    xTaskCreate(&adc_read_reflectance_sensor_task, "reflectance", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&stepper_task, "stepper", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    printf("Started all test tasks\n");
    */
    buzzer_init();
    printf("Initialized buzzer\n");
    xTaskCreate(&buzzer_handler_task, "buzzer_handler", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
    ls_stepper_init();
    xTaskCreate(&ls_stepper_task, "stepper", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    ls_magnet_isr_begin();
//    ls_state_current.func = ls_state_home_to_magnet; 
    ls_state_current.func = ls_state_active; ls_stepper_random();
    xTaskCreate(&event_handler_state_machine, "event_handler_state_machine", configMINIMAL_STACK_SIZE * 3, NULL, 2, NULL);
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("app_main()) has finished.\n");
    xSemaphoreGive(print_mux);
}
