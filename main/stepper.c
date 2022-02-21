#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "stepper.h"

volatile extern bool ls_stepperstep_level;
volatile extern bool ls_stepper_direction;
volatile extern bool ls_stepper_sleep;

static bool IRAM_ATTR stepper_step_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    ls_stepperstep_level = !ls_stepperstep_level;
    gpio_set_level(LSGPIO_STEPPERSTEP, ls_stepperstep_level ? 1 : 0);
    if (ls_stepperstep_level) {
;
    } else {
        ls_stepper_position += ls_stepper_direction ? 1 : -1;
    }
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
        // xSemaphoreTake(print_mux, portMAX_DELAY);
        // printf("Stepper forward (%d)\n", stepperstep_level);
        // xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(8000));
        gpio_set_level(LSGPIO_STEPPERDIRECTION, 1);
        // xSemaphoreTake(print_mux, portMAX_DELAY);
        // printf("Stepper reverse (%d)\n", stepperstep_level);
        // xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(3000));
        gpio_set_level(LSGPIO_STEPPERSLEEP, 0);
        // xSemaphoreTake(print_mux, portMAX_DELAY);
        // printf("Stepper asleep (%d)\n", stepperstep_level);
        // xSemaphoreGive(print_mux);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}