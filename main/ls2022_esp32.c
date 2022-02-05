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
#include "driver/mcpwm.h"
#include "esp_adc_cal.h"
#include "config.h"
#ifdef LSBOARD_TESTNOV21
#include "mpu6050.h"
#else
#include "kxtj3.h"
#endif
#include "events.h"
#include "buzzer.h"

#define STEPPER_TIMER_DIVIDER (40)

QueueHandle_t ls_event_queue = NULL;

SemaphoreHandle_t adc1_mux = NULL;
SemaphoreHandle_t adc2_mux = NULL;
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
#ifdef LSBOARD_TESTNOV21
        float tilt = mpu6050_read_accel_z();
#else
        float tilt = kxtj3_read_accel_z();
#endif
        xSemaphoreGive(i2c_mux);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Tilt (AccelZ)= %0.2fg\n", tilt);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
#ifdef LSBOARD_TESTNOV21
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
#endif

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

void IRAM_ATTR magnet_event_isr(void *pvParameter)
{
    // note that the sensor pulls low when triggered
    enum ls_event_types event = gpio_get_level(LSGPIO_MAGNETSENSE) ? LSEVT_MAGNET_LEAVE : LSEVT_MAGNET_ENTER;
    xQueueSendFromISR(ls_event_queue, (void *)&event, NULL);
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
    gpio_set_level(LSGPIO_STEPPERSTEP, stepperstep_level++ % 2);
    /* See timer_group_example for how to use this: */
    //    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}
static void stepper_task(void *pvParameter)
{
    gpio_set_level(LSGPIO_STEPPERSLEEP, 0); // don't do anything while we get ready
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
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, APB_CLK_FREQ / STEPPER_TIMER_DIVIDER / 1500));
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, stepper_step_isr_callback, NULL, 0));
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
    while (1)
    {
        gpio_set_level(LSGPIO_STEPPERDIRECTION, 0);
        gpio_set_level(LSGPIO_STEPPERSLEEP, 1);
        // should delay 1ms here before stepping, but deal with that in production code
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

void adc2_read_knobs_task(void *pvParameter)
{
#define KNOB_COUNT 4
    adc2_channel_t knob_channels[] = {LSADC2_KNOB3, LSADC2_KNOB4, LSADC2_KNOB5, LSADC2_KNOB6};
    uint32_t knob_readings[KNOB_COUNT];
    while (1)
    {
        for (int knob = 0; knob < KNOB_COUNT; knob++)
        {
            xSemaphoreTake(adc2_mux, pdMS_TO_TICKS(1000));
            int adc_reading = 0;
            uint32_t adc_sum = 0;
            // atten 11 by default... shouldn't need to focus on lower voltages?
            ESP_ERROR_CHECK(adc2_config_channel_atten(knob_channels[knob], ADC_ATTEN_11db));
            for (int i = 0; i < 4; i++)
            {
                ESP_ERROR_CHECK(adc2_get_raw(knob_channels[knob], ADC_WIDTH_12Bit, &adc_reading));
                adc_sum += adc_reading;
            }
            knob_readings[knob] = adc_sum / 4;
            xSemaphoreGive(adc2_mux);
            vTaskDelay(2); // yield some time to other tasks... we're in no particular hurry
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Knobs: %d\t %d\t %d\t %d\n", knob_readings[0], knob_readings[1], knob_readings[2], knob_readings[3]);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}
void adc_heater_task(void *pvParameter)
{
    while (1)
    {
        // turn on the heater (or test LED)
        gpio_set_level(LSGPIO_LASERHEATERENABLE, 1);
        xSemaphoreTake(adc1_mux, pdMS_TO_TICKS(1000));
        adc1_config_width(ADC_WIDTH_BIT_12);
        // atten 11 by default... shouldn't need to focus on lower voltages?
        adc1_config_channel_atten(LSADC1_LASERTEMP, ADC_ATTEN_11db);
        uint32_t adc_reading = 0;
        for (int i = 0; i < 4; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)LSADC1_LASERTEMP);
        }
        adc_reading /= 4;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        xSemaphoreGive(adc1_mux);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Laser thermistor sensor raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(1500));
        // turn off the heater after a time
        gpio_set_level(LSGPIO_LASERHEATERENABLE, 0);
        // and wait some more time before doing it again
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}
void servo_task(void *pvParameter)
{
    // Set PWM0A to LSGPIO_SERVOPULSE (from the example code)
    mcpwm_gpio_init(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_IO_SIGNALS, LSGPIO_SERVOPULSE);
    mcpwm_config_t pwm_config = {
        .frequency = 50,    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,    //duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };
    mcpwm_init(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    while (1)
    {
        // turn on the servo and move to midpoint for 5 seconds
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Servo to midpoint (5sec)\n");
        xSemaphoreGive(print_mux);
        gpio_set_level(LSGPIO_SERVOPOWERENABLE, 1);
        mcpwm_start(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER);
        mcpwm_set_duty_in_us(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER, LS_SERVO_MCPWM_GENERATOR, LS_SERVO_US_MID);
        vTaskDelay(pdMS_TO_TICKS(5000));
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Servo pulse getting longer\n");
        xSemaphoreGive(print_mux);
        for(int i = LS_SERVO_US_MID; i <= LS_SERVO_US_MAX; i+=15) {
            mcpwm_set_duty_in_us(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER, LS_SERVO_MCPWM_GENERATOR, i);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Servo pulse getting shorter\n");
        xSemaphoreGive(print_mux);
        for(int i = LS_SERVO_US_MAX; i >= LS_SERVO_US_MIN; i-=15) {
            mcpwm_set_duty_in_us(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER, LS_SERVO_MCPWM_GENERATOR, i);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Servo moving back to midpoint\n");
        xSemaphoreGive(print_mux);
        for(int i = LS_SERVO_US_MIN; i <= LS_SERVO_US_MID; i+=15) {
            mcpwm_set_duty_in_us(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER, LS_SERVO_MCPWM_GENERATOR, i);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Servo off (2sec)\n");
        xSemaphoreGive(print_mux);
        mcpwm_stop(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER);
        gpio_set_level(LSGPIO_SERVOPOWERENABLE, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void app_main(void)
{
    ls_event_queue = xQueueCreate(32, sizeof(enum ls_event_types));

    check_efuse();
    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, adc_chars);
    print_char_val_type(val_type);
    printf("Checked ADC efuse\n");
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    adc1_mux = xSemaphoreCreateMutex();
    adc2_mux = xSemaphoreCreateMutex();
    i2c_mux = xSemaphoreCreateMutex();
    print_mux = xSemaphoreCreateMutex();
    printf("Initializing GPIO\n");
    lsgpio_initialize();
    printf("Initialized GPIO\n");
    printf("Initializing I2C\n");
    ESP_ERROR_CHECK(lsi2c_master_init());
    printf("Initialized I2C\n");
    #ifdef LSBOARD_TESTNOV21
    printf("Initializing MPU6050\n");
    mpu6050_begin();
    printf("Initialized MPU6050 i2c accelerometer\n");
    xTaskCreate(&i2c_read_temp_task, "i2c_temp", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    #else
    printf("Initializing KXTJ3-1057\n");
    kxtj3_begin();
    printf("Initialized KXTJ3 i2c accelerometer\n");
    #endif
    xTaskCreate(&i2c_read_tilt_task, "i2c_tilt", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    // so, do you just have to figure out the usStackDepth parameter (here, configMINIMAL_STACK_SIZE*2) by trial and error?
    xTaskCreate(&adc_read_tape_setting_task, "adcr_tapesetting", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&adc_read_light_sensor_task, "adcr_light", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&event_handler_task, "event_handler", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
    buzzer_init();
    xTaskCreate(&buzzer_handler_task, "buzzer_handler", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    printf("Initialized buzzer\n");
    buzzer_play(LS_BUZZER_ALTERNATE_HIGH);

    // set the magnet sensor to trigger an interrupt as it enters and as it leaves
    gpio_set_intr_type(LSGPIO_MAGNETSENSE, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0); // default, no flags.
    gpio_isr_handler_add(LSGPIO_MAGNETSENSE, &magnet_event_isr, NULL);
    printf("Started magnet sense ISR\n");
    xTaskCreate(&adc_read_reflectance_sensor_task, "reflectance", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&stepper_task, "stepper", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&adc2_read_knobs_task, "knobs", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&adc_heater_task, "heater", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(&servo_task, "servo", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    printf("Started all test tasks\n");
}
