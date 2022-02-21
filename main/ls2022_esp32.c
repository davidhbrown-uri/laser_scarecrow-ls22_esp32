#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "config.h"
#include "mpu6050.h"
#include "events.h"
#include "buzzer.h"
#include "stepper.h"
#include "magnet.h"

#define STEPPER_TIMER_DIVIDER (40)

extern QueueHandle_t ls_event_queue;

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

/**
 * @brief
 * @todo switch to using a queue with the full ls_event structure.
 * @see https://controllerstech.com/freertos-tutorial-5-using-queue/
 *
 * @param pvParameter
 */
void event_handler_task(void *pvParameter)
{
    enum ls_event_types received;
    while (1)
    {
        if (xQueueReceive(ls_event_queue, &received, portMAX_DELAY) != pdTRUE)
        {
            printf("No events received after maximum delay... getting very bored.");
        }
        else
        {
            switch (received)
            {
            case LSEVT_MAGNET_ENTER:
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Magnet Entered detection area\n");
                xSemaphoreGive(print_mux);
                buzzer_play(LS_BUZZER_CLICK);
                break;
            case LSEVT_MAGNET_LEAVE:
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Magnet left detection area\n");
                xSemaphoreGive(print_mux);
                buzzer_play(LS_BUZZER_CLICK);
                break;
            default:
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Unknown event %d -- I'm confused", received);
                xSemaphoreGive(print_mux);
            }
        }
    }
}



// from adc1_example_main.c
static void check_efuse(void)
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


volatile uint32_t stepperstep_level = 0;  
static bool IRAM_ATTR stepper_step_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    gpio_set_level(LSGPIO_STEPPERSTEP, stepperstep_level++%2);
   /* See timer_group_example for how to use this: */
//    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR

}
static void stepper_task(void *pvParameter)
{
    gpio_set_level(LSGPIO_STEPPERSLEEP, 0);//don't do anything while we get ready
    // see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/timer.html
    // defaut ESP32 clock source is 80MHz (1MHz rate = 1μs period)
    // Stepper is set to 1/16 microsteps, so 200*16=3200 steps per rotation
    // if we presume 1s/rotation (60RPM) is a reasonable maximum speed, then
    // we want to step at a maximum period of 1s/3200steps=312.5μs/step.
    // Up to 100x slowdown to allow acceleration from stopped sounds good.
    // Per ch 18 of the ESP32 Technical Reference manual, the minimum clock divisor is 2
    // So with the clock divider at 2, we'd need timer values of
    //  - 160 (minimum) to meet A4988's STEP minimum, HIGH pulse width (LOW is the same)
    //  - 50,000 for the fastest step
    //  - 5,500,000 for the slowest step
    // The timers are 64-bit, so counting this number of steps should not be an issue
    // Tather than aiming for 1ms pulses, toggling at the total timer count for 
    // a square(ish) wave would make sense.
    timer_config_t stepper_step_timer_config = {
        .divider = STEPPER_TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &stepper_step_timer_config));
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0ULL));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, APB_CLK_FREQ / STEPPER_TIMER_DIVIDER / 1500 ));
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, stepper_step_isr_callback, NULL, 0));
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
    while (1)
    {
        gpio_set_level(LSGPIO_STEPPERDIRECTION, 0);
        gpio_set_level(LSGPIO_STEPPERSLEEP, 1);
        //should delay 1ms here before stepping, but deal with that in production code
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Stepper forward (%d)\n", stepperstep_level);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(8000));
        gpio_set_level(LSGPIO_STEPPERDIRECTION, 1);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Stepper reverse (%d)\n", stepperstep_level);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(3000));
        xSemaphoreTake(print_mux, portMAX_DELAY);
        gpio_set_level(LSGPIO_STEPPERSLEEP, 0);
        printf("Stepper asleep (%d)\n", stepperstep_level);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main(void)
{ /*
    check_efuse();
    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
    print_char_val_type(val_type);
    printf("Checked ADC efuse\n");
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc1_mux = xSemaphoreCreateMutex();
    i2c_mux = xSemaphoreCreateMutex();
    print_mux = xSemaphoreCreateMutex();
    printf("Initializing GPIO\n");
    lsgpio_initialize();
    printf("Initialized GPIO\n");
    printf("Initializing I2C\n");
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
    xTaskCreate(&event_handler_task, "event_handler", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
    buzzer_init();
    printf("Initialized buzzer\n");
    xTaskCreate(&buzzer_handler_task, "buzzer_handler", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
  
    xTaskCreate(&adc_read_reflectance_sensor_task, "reflectance", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&stepper_task, "stepper", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    printf("Started all test tasks\n");
*/}
