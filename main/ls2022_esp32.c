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
#include "i2c.h"

SemaphoreHandle_t adc1_mux = NULL;
SemaphoreHandle_t adc2_mux = NULL;
SemaphoreHandle_t i2c_mux = NULL;
SemaphoreHandle_t print_mux = NULL;

static esp_adc_cal_characteristics_t *adc_chars;


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
    vTaskDelay(pdMS_TO_TICKS(2000)); // let voltages settle, USB connect
    adc1_mux = xSemaphoreCreateMutex();
    adc2_mux = xSemaphoreCreateMutex();
    print_mux = xSemaphoreCreateMutex();
    printf("Initializing I2C...\n");
    ls_i2c_init();
    if(ls_i2c_accelerometer_device() == LS_I2C_ACCELEROMETER_MPU6050)
    {
        printf("November '21 test board detected via MPU6050 accelerometer\n");

        ls_config_set_gpio_nov21();
    }
    printf("Initializing GPIO...\n");
    ls_gpio_initialize();
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
    print_char_val_type(val_type);
    check_efuse();
    printf("Initialized Hardware\n");
    ls_settings_set_defaults();
    ls_settings_read();
    printf("Loaded settings\n");
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    ls_state_current.func = ls_state_poweron; // default
    if(ls_i2c_accelerometer_device()==LS_I2C_ACCELEROMETER_NONE)
    {
        printf("No accelerometer detected!\n");
        ls_state_current.func = ls_state_error_noaccel;
    }
    ls_event_queue_init();
    ls_buzzer_init();
    ls_stepper_init();
    ls_servo_init();
    ls_state_init();

    // do not set magnet ISR up before event queue
    ls_magnet_isr_begin();

    printf("Initialized queues / semaphores / IRQs\n");

    // higher priority tasks get higher priority values

    // highest priority (30-31)
    xTaskCreate(&ls_stepper_task, "stepper", configMINIMAL_STACK_SIZE * 3, NULL, 30, NULL);

    // high-priority; time/performance sensitive (20-29)
    xTaskCreate(&ls_controls_task, "controls_task", configMINIMAL_STACK_SIZE * 3, NULL, 20, NULL);

    // medium-priority (10-19)
    xTaskCreate(&event_handler_state_machine, "event_handler_state_machine", configMINIMAL_STACK_SIZE * 3, NULL, 15, NULL);

    // lowest priority (1-9)
    xTaskCreate(&ls_tilt_task, "tilt_task", configMINIMAL_STACK_SIZE * 3, NULL, 7, NULL);
    xTaskCreate(&ls_buzzer_handler_task, "buzzer_handler", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL);
    xTaskCreate(&ls_servo_task, "servo_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);

#ifdef LSDEBUG_TAPEMODE
    xTaskCreate(&ls_tapemode_debug_task, "tapemode_debug_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
#endif
#ifdef LSDEBUG_STEPPER
    xTaskCreate(&ls_stepper_debug_task, "stepper_debug", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
#endif
    xTaskCreate(&ls_lightsense_read_task, "lightsense_read", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);


    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("app_main() has finished.\n");
    xSemaphoreGive(print_mux);
}