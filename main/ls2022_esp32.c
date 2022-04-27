#include <stdio.h>
#include <string.h>
#include <stdlib.h>
// FreeRTOS.h defines bool type
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_adc_cal.h"
#include "config.h"
#include "debug.h"
#include "init.h"
#include "mpu6050.h"
#include "events.h"
#include "buzzer.h"
#include "stepper.h"
#include "magnet.h"
#include "states.h"
#include "controls.h"
#include "tapemode.h"
#include "tape.h"
#include "map.h"
#include "lightsense.h"
#include "servo.h"
#include "settings.h"

SemaphoreHandle_t adc1_mux = NULL;
SemaphoreHandle_t adc2_mux = NULL;
SemaphoreHandle_t i2c_mux = NULL;
SemaphoreHandle_t print_mux = NULL;

static esp_adc_cal_characteristics_t *adc_chars;



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

/**
 * @brief Handle essential setup, then transfer to event loop
 *
 * Phase 1: essential initialization of hardware
 * Phase 2: initialization of queues, mutexes, etc
 * Phase 3: setup of output (laser, stepper, servo, buzzer)
 * Phase 4: setup state machine to handle events (read saved settings in poweron state?)
 * Phase 5: setup of inputs (light, tape, tilt, controls)
 */
void app_main(void)
{
    adc1_mux = xSemaphoreCreateMutex();
    adc2_mux = xSemaphoreCreateMutex();
    print_mux = xSemaphoreCreateMutex();
    printf("Initializing GPIO\n");
    ls_gpio_initialize();
    /** @todo: servo setup */
    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
    print_char_val_type(val_type);
    check_efuse();
    printf("Initialized Hardware\n");
    ls_settings_set_defaults();
    ls_settings_read();
    printf("Loaded settings\n");
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    ls_state_current.func = ls_state_poweron;
    // ls_state_current.func = ls_state_active; ls_stepper_random();
    ls_event_queue_init();
    ls_buzzer_init();
    ls_stepper_init();
    ls_servo_init();
    ls_state_init();
    /*
    printf("Initializing I2C\n");
    i2c_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(lsi2c_master_init());
    printf("Initialized I2C\n");
    printf("Initializing MPU6050\n");
    mpu6050_begin();
    printf("Initialized MPU6050 i2c device\n");
    // so, do you just have to figure out the usStackDepth parameter (here, configMINIMAL_STACK_SIZE*2) by trial and error?
    xTaskCreate(&adc_read_light_sensor_task, "adcr_light", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&i2c_read_tilt_task, "i2c_tilt", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&i2c_read_temp_task, "i2c_temp", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);

    printf("Started all test tasks\n");
    */
    printf("Initialized queues / semaphores / IRQs\n");

    // higher priority tasks get higher priority values

    // highest priority (30-31)

    // high-priority; time/performance sensitive (20-29)
    xTaskCreate(&ls_stepper_task, "stepper", configMINIMAL_STACK_SIZE * 3, NULL, 25, NULL);

    // medium-priority (10-19)
    xTaskCreate(&event_handler_state_machine, "event_handler_state_machine", configMINIMAL_STACK_SIZE * 3, NULL, 15, NULL);
    xTaskCreate(&ls_controls_task, "controls_task", configMINIMAL_STACK_SIZE * 3, NULL, 10, NULL);

    // lowest priority (1-9)
    xTaskCreate(&ls_buzzer_handler_task, "buzzer_handler", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL);
    xTaskCreate(&ls_servo_task, "servo_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);

#ifdef LSDEBUG_TAPEMODE
    xTaskCreate(&ls_tapemode_debug_task, "tapemode_debug_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
#endif
#ifdef LSDEBUG_STEPPER
    xTaskCreate(&ls_stepper_debug_task, "stepper_debug", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
#endif
    xTaskCreate(&ls_lightsense_read_task, "lightsense_read", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    ls_magnet_isr_begin();

    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("app_main() has finished.\n");
    xSemaphoreGive(print_mux);
}