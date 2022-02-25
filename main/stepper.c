#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "bootloader_random.h"
#include "esp_random.h"
#include "stepper.h"

#define LS_STEPPER_TIMER_DIVIDER (20)
    // see https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/timer.html
    // defaut ESP32 clock source is 80MHz (1MHz rate = 1μs period)
    // divide by 20 to get ....
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

extern SemaphoreHandle_t print_mux; // in ls2022_esp32.c

volatile int32_t ls_stepper_steps_remaining;
volatile int32_t ls_stepper_steps_taken;
volatile bool ls_stepperstep_level = false;
bool ls_stepper_direction = false;
bool ls_stepper_sleep = false;

static double ls_stepper_acceleration_slope = (double) (LS_STEPPER_STEPS_PER_SECOND_MAX - LS_STEPPER_STEPS_PER_SECOND_MIN) / (double) LS_STEPPER_STEPS_FULLSPEED;

static bool IRAM_ATTR ls_stepper_step_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    if(ls_stepper_steps_remaining > 0) // only do a step if any remain 
    {
        ls_stepperstep_level = !ls_stepperstep_level;
        gpio_set_level(LSGPIO_STEPPERSTEP, ls_stepperstep_level ? 1 : 0);
        if (ls_stepperstep_level) { // beginning a step pulse
            ls_stepper_position += ls_stepper_direction*2-1;
            while(ls_stepper_position < 0)
            {
                ls_stepper_position += LS_STEPPER_STEPS_PER_ROTATION;
            }
            while(ls_stepper_position > LS_STEPPER_STEPS_PER_ROTATION)
            {
                ls_stepper_position -= LS_STEPPER_STEPS_PER_ROTATION;
            }
        } else { // ending the step pulse
            ls_stepper_steps_remaining--;
            ls_stepper_steps_taken++;
        }
    } 
    /* See timer_group_example for how to use this: */
    //    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

void ls_stepper_init(void)
{
    ls_stepper_position = 0;
    bootloader_random_enable();
    gpio_set_level(LSGPIO_STEPPERSLEEP, 0); // don't do anything while we get ready
    ls_stepper_queue = xQueueCreate(8, sizeof(enum ls_stepper_mode));
    ls_stepper_steps_remaining = 0;
    ls_stepper_steps_taken = 0;
    timer_config_t stepper_step_timer_config = {
        .divider = LS_STEPPER_TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &stepper_step_timer_config));
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0ULL));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, APB_CLK_FREQ / LS_STEPPER_TIMER_DIVIDER / LS_STEPPER_STEPS_PER_SECOND_MIN));
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, ls_stepper_step_isr_callback, NULL, 0));
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
}

static void ls_stepper_set_speed(void)
{
    // lesser of steps remaining (deceleration) or steps taken (acceleration)
    int16_t steps = ls_stepper_steps_remaining < ls_stepper_steps_taken ? ls_stepper_steps_remaining : ls_stepper_steps_taken;
    // but not more than full speed
    steps = steps < LS_STEPPER_STEPS_FULLSPEED ? steps : LS_STEPPER_STEPS_FULLSPEED;
    uint64_t rate = LS_STEPPER_STEPS_PER_SECOND_MIN + (uint64_t) (ls_stepper_acceleration_slope * (double) steps);
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, APB_CLK_FREQ / LS_STEPPER_TIMER_DIVIDER / rate));
}

void ls_stepper_task(void *pvParameter)
{
    enum ls_stepper_mode received;
    enum ls_stepper_mode current_mode = LS_STEPPER_MODE_STOP;

    while (1)
    {
        if (uxQueueMessagesWaiting(ls_stepper_queue)>0)
        {
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("ls_stepper_queue has %d messages waiting\n", uxQueueMessagesWaiting(ls_stepper_queue));
            xSemaphoreGive(print_mux);
            bool success = xQueueReceive(ls_stepper_queue, &received, 0);
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("ls_stepper_queue message was%s received successfully\n", success ? "" : " NOT");
            xSemaphoreGive(print_mux);
            // do something if we need to when switching modes
            current_mode = received;
        }
        switch(current_mode){
            case LS_STEPPER_MODE_STOP:
                gpio_set_level(LSGPIO_STEPPERSLEEP, 1);
                if (ls_stepper_steps_remaining > LS_STEPPER_STEPS_FULLSPEED)
                {
                    ls_stepper_steps_remaining = LS_STEPPER_STEPS_FULLSPEED;
                }
                ls_stepper_set_speed();
            break;
            case LS_STEPPER_MODE_RANDOM:
                gpio_set_level(LSGPIO_STEPPERSLEEP, 1);
                if(ls_stepper_steps_remaining<=0)
                {
                    uint32_t random = esp_random();
                    ls_stepper_steps_taken = 0;
                    ls_stepper_direction = ((uint8_t) random & 0xFF) > LS_STEPPER_MOVEMENT_REVERSE_PER255 ? false : true;
                    gpio_set_level(LSGPIO_STEPPERDIRECTION, ls_stepper_direction ? 1 : 0);
                    ls_stepper_steps_remaining = LS_STEPPER_MOVEMENT_STEPS_MIN + ((random >> 16) * (LS_STEPPER_MOVEMENT_STEPS_MAX - LS_STEPPER_MOVEMENT_STEPS_MIN) / 65536);
                    xSemaphoreTake(print_mux, portMAX_DELAY);
                    printf("From %d, moving %d steps %s\n", ls_stepper_position, ls_stepper_steps_remaining, ls_stepper_direction ? "forward" : "reverse");
                    xSemaphoreGive(print_mux);

                }
                ls_stepper_set_speed();
            break;
            case LS_STEPPER_MODE_SLEEP:
            /** @todo */
                break;
            case LS_STEPPER_MODE_HOME:
            /** @todo */
                break;
            case LS_STEPPER_MODE_DOUBLE_ROTATION:
                gpio_set_level(LSGPIO_STEPPERSLEEP, 1);
                if(ls_stepper_steps_remaining<=0)
                {
                    ls_stepper_steps_taken = 0;
                    ls_stepper_direction = true;
                    ls_stepper_steps_remaining = LS_STEPPER_STEPS_PER_ROTATION * 2;
                }
                ls_stepper_set_speed();
                break;
        }
        vTaskDelay(2);
    }
}

int32_t ls_stepper_get_position(void)
{
    return ls_stepper_position;
};