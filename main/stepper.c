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
#include "laser.h"
#include "events.h"
#include "config.h"
#include "debug.h"
#include "map.h"
#include "util.h"
#include "settings.h"

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

volatile BaseType_t IRAM_ATTR ls_stepper_steps_remaining;
volatile BaseType_t IRAM_ATTR ls_stepper_steps_taken;
volatile static BaseType_t IRAM_ATTR _ls_stepperstep_phase = 0;

enum ls_stepper_direction IRAM_ATTR ls_stepper_direction = LS_STEPPER_ACTION_FORWARD_STEPS;
bool ls_stepper_sleep = false;
static bool _ls_stepper_enable_skipping = false;
static int _ls_stepper_steps_per_second_max = LS_STEPPER_STEPS_PER_SECOND_DEFAULT;

static int _ls_stepper_speed_when_skipping = LS_STEPPER_STEPS_PER_SECOND_MAX;
static int _ls_stepper_speed_not_skipping;
static int _ls_stepper_speed_current_rate;

// how many steps it will take to decelerate from full speed
static int _ls_stepper_steps_to_decelerate(int current_rate)
{
    return (current_rate * current_rate) / (2 * LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_SECOND) +
           (current_rate / 20); // not sure where the 20 comes from; see spreadsheet
}

void ls_stepper_set_maximum_steps_per_second(int steps_per_second)
{
    // set and constrain new current speed limit
    _ls_stepper_steps_per_second_max = _constrain(steps_per_second, LS_STEPPER_STEPS_PER_SECOND_MIN, LS_STEPPER_STEPS_PER_SECOND_MAX);
    // #ifdef LSDEBUG_STEPPER
    //     ls_debug_printf("Stepper speed set to %d max steps/s (%d requested); %d steps to decelerate.\n",
    //                     _ls_stepper_steps_per_second_max, steps_per_second, _ls_stepper_steps_to_decelerate(_ls_stepper_steps_per_second_max));
    // #endif
}

static bool IRAM_ATTR ls_stepper_step_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    if (ls_stepper_steps_remaining > 0) // only do a step if any remain
    {
        _ls_stepperstep_phase = 1 - _ls_stepperstep_phase;
        gpio_set_level(LSGPIO_STEPPERSTEP, 1 - _ls_stepperstep_phase);
        if (0 == _ls_stepperstep_phase)
        { // beginning a step pulse (high)
            // LS_STEPPER_DIRECTION_REVERSE == 0; LS_STEPPER_DIRECTION_REVERSE == 1
            ls_stepper_position += ls_stepper_direction * 2 - 1;
            while (ls_stepper_position < 0)
            {
                ls_stepper_position += LS_STEPPER_STEPS_PER_ROTATION;
            }
            while (ls_stepper_position >= LS_STEPPER_STEPS_PER_ROTATION)
            {
                ls_stepper_position -= LS_STEPPER_STEPS_PER_ROTATION;
            }
            if (ls_laser_mode_is_mappped())
            {
                gpio_set_level(LSGPIO_LASERPOWERENABLE, ls_map_is_enabled_at(ls_stepper_position));
            }
        }
        else
        { // ending the step pulse (low)
            ls_stepper_steps_remaining--;
            ls_stepper_steps_taken++;
            if (ls_stepper_steps_remaining == 0)
            {
                ls_event event;
                event.type = LSEVT_STEPPER_FINISHED_MOVE;
                event.value = 0;
                xQueueSendToFrontFromISR(ls_event_queue, (void *)&event, NULL);
            }
        }
    }
    /* See timer_group_example for how to use this: */
    //    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

void ls_stepper_init(void)
{
    ls_stepper_position = 0;
    ls_stepper_direction = LS_STEPPER_DIRECTION_FORWARD;
    bootloader_random_enable();
    gpio_set_level(LSGPIO_STEPPERSLEEP, 0); // don't do anything while we get ready
    ls_stepper_queue = xQueueCreate(8, sizeof(ls_stepper_action_message));
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

static void _ls_stepper_set_speed(void)
{
    bool skipping = _ls_stepper_enable_skipping && !ls_map_is_enabled_at(ls_stepper_position); 
    if (_ls_stepper_enable_skipping)
    {
        ls_stepper_set_maximum_steps_per_second(skipping ? _ls_stepper_speed_when_skipping : _ls_stepper_speed_not_skipping);
    }
    if (skipping)
    {
        ls_stepper_steps_remaining = _constrain(ls_stepper_steps_remaining + _ls_stepper_speed_current_rate / pdMS_TO_TICKS(1000), 
        0, _ls_stepper_steps_to_decelerate(_ls_stepper_speed_not_skipping));
    }
    if (!skipping && ls_stepper_steps_remaining < ls_stepper_steps_taken // accelerate at least halfway
        && ls_stepper_steps_remaining < _ls_stepper_steps_to_decelerate(_ls_stepper_speed_current_rate))
    {
    //decelerate
        _ls_stepper_speed_current_rate -= LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_TICK;
    }
    // accelerate?
    else if (_ls_stepper_speed_current_rate < _ls_stepper_steps_per_second_max)
    {
        _ls_stepper_speed_current_rate += LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_TICK;
    }
    _ls_stepper_speed_current_rate = _constrain(_ls_stepper_speed_current_rate, LS_STEPPER_STEPS_PER_SECOND_MIN, LS_STEPPER_STEPS_PER_SECOND_MAX);
#ifdef LSDEBUG_STEPPER
//    ls_debug_printf(" >> Stepper rate: %d  (taken: %d; remaining: %d)\n",
//    _ls_stepper_speed_current_rate, ls_stepper_steps_taken, ls_stepper_steps_remaining);
#endif
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, APB_CLK_FREQ / LS_STEPPER_TIMER_DIVIDER / _ls_stepper_speed_current_rate));
}

void ls_stepper_task(void *pvParameter)
{
    ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_DEFAULT);
    enum ls_stepper_action current_action = LS_STEPPER_ACTION_IDLE;
    ls_stepper_action_message message;

    while (1)
    {
        if (uxQueueMessagesWaiting(ls_stepper_queue) > 0)
        {
            // peek at message queue because we need to stop if we have to change direction first
            bool success = xQueuePeek(ls_stepper_queue, &message, 0);
            if (ls_stepper_steps_remaining > 0 && success && ((LS_STEPPER_ACTION_FORWARD_STEPS == message.action && LS_STEPPER_DIRECTION_REVERSE == ls_stepper_direction) || (LS_STEPPER_ACTION_REVERSE_STEPS == message.action && LS_STEPPER_DIRECTION_FORWARD == ls_stepper_direction)))
            {
#ifdef LSDEBUG_STEPPER
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Stepper stopping before change in direction (dir=%d; upcoming action=%d)\n", ls_stepper_direction, message.action);
                xSemaphoreGive(print_mux);
#endif
                current_action = LS_STEPPER_ACTION_STOP;
            }
            else
            { // dequeue the message
                success = xQueueReceive(ls_stepper_queue, &message, 0);
                if (success)
                {
                    current_action = message.action;
#ifdef LSDEBUG_STEPPER
                    xSemaphoreTake(print_mux, portMAX_DELAY);
                    printf("Stepper dequeued action %d (%d steps); ls_stepper_steps_remaining=%d\n",
                           current_action, message.steps, ls_stepper_steps_remaining);
                    xSemaphoreGive(print_mux);
#endif
                }
            } // else
        }     // if there was anything in the message queue
        switch (current_action)
        {
        case LS_STEPPER_ACTION_IDLE:
            _ls_stepper_set_speed();
            break;
        case LS_STEPPER_ACTION_FORWARD_STEPS:
            gpio_set_level(LSGPIO_STEPPERSLEEP, 1);
            if (ls_stepper_steps_remaining <= 0)
            {
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("Begin forward move of %d step(s)\n", message.steps);
#endif
                ls_stepper_direction = LS_STEPPER_DIRECTION_FORWARD;
                gpio_set_level(LSGPIO_STEPPERDIRECTION, ls_stepper_direction);
                ls_stepper_steps_taken = 0;
                ls_stepper_steps_remaining = message.steps;
            }
            else
            {
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("Extending forward move by %d step(s)\n", message.steps);
#endif
                ls_stepper_steps_remaining += message.steps;
            }
            current_action = LS_STEPPER_ACTION_IDLE;
            _ls_stepper_set_speed();
            break;
        case LS_STEPPER_ACTION_REVERSE_STEPS:
            gpio_set_level(LSGPIO_STEPPERSLEEP, 1);
            if (ls_stepper_steps_remaining <= 0)
            {
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("Begin reverse move of %d step(s)\n", message.steps);
#endif
                ls_stepper_direction = LS_STEPPER_DIRECTION_REVERSE;
                gpio_set_level(LSGPIO_STEPPERDIRECTION, ls_stepper_direction);
                ls_stepper_steps_taken = 0;
                ls_stepper_steps_remaining = message.steps;
            }
            else
            {
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("Extending reverse move by %d step(s)\n", message.steps);
#endif
                ls_stepper_steps_remaining += message.steps;
            }
            current_action = LS_STEPPER_ACTION_IDLE;
            _ls_stepper_set_speed();
            break;
        case LS_STEPPER_ACTION_STOP:
#ifdef LSDEBUG_STEPPER
            ls_debug_printf("Stepper stopping\n");
#endif
            gpio_set_level(LSGPIO_STEPPERSLEEP, 1);
            ls_stepper_steps_remaining = _constrain(ls_stepper_steps_remaining, 0, _ls_stepper_steps_to_decelerate(_ls_stepper_speed_current_rate));
            _ls_stepper_set_speed();
            if (ls_stepper_steps_remaining <= 0)
            {
                current_action = LS_STEPPER_ACTION_IDLE;
            }
            break;
        case LS_STEPPER_ACTION_RANDOM:
            gpio_set_level(LSGPIO_STEPPERSLEEP, 1);
            if (ls_stepper_steps_remaining <= 0)
            {
                uint32_t random = esp_random();
                ls_stepper_steps_taken = 0;
                ls_stepper_direction = ((uint8_t)random & 0xFF) > LS_STEPPER_MOVEMENT_REVERSE_PER255 ? false : true;
                gpio_set_level(LSGPIO_STEPPERDIRECTION, ls_stepper_direction ? 1 : 0);
                ls_stepper_steps_remaining = LS_STEPPER_MOVEMENT_STEPS_MIN + ((random >> 16) * (ls_settings_get_stepper_random_max() - LS_STEPPER_MOVEMENT_STEPS_MIN) / 65536);
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("From %d, moving %d steps %s\n", ls_stepper_position, ls_stepper_steps_remaining, ls_stepper_direction ? "-->" : "<--");
#endif
            }
            _ls_stepper_set_speed();
            break;
        case LS_STEPPER_ACTION_SLEEP:
#ifdef LSDEBUG_STEPPER
            ls_debug_printf("Stepper sleeping\n");
#endif
            gpio_set_level(LSGPIO_STEPPERSLEEP, 0);
            current_action = LS_STEPPER_ACTION_IDLE;
            break;
        }
#ifdef LSDEBUG_STEPPER
// TMI
// xSemaphoreTake(print_mux, portMAX_DELAY);
// printf("Stepper action=%d; steps_remaining=%d; steps_taken=%d\n", current_action, ls_stepper_steps_remaining, ls_stepper_steps_taken);
// xSemaphoreGive(print_mux);
#endif
        vTaskDelay(1);
    }
}

void ls_stepper_stop(void)
{
    _ls_stepper_enable_skipping = false;
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_STOP;
    message.steps = 0;
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
}

void ls_stepper_forward(uint16_t steps)
{
    _ls_stepper_enable_skipping = false;
    if (steps > 0)
    {
        ls_stepper_action_message message;
        message.action = LS_STEPPER_ACTION_FORWARD_STEPS;
        message.steps = steps;
        xQueueSend(ls_stepper_queue, (void *)&message, 0);
    }
    else
    {
        ls_stepper_stop();
    }
}

void ls_stepper_reverse(uint16_t steps)
{
    _ls_stepper_enable_skipping = false;
    if (steps > 0)
    {
        ls_stepper_action_message message;
        message.action = LS_STEPPER_ACTION_REVERSE_STEPS;
        message.steps = steps;
        xQueueSend(ls_stepper_queue, (void *)&message, 0);
    }
    else
    {
        ls_stepper_stop();
    }
}

void ls_stepper_random(void)
{
    _ls_stepper_enable_skipping = LS_MAP_STATUS_OK == ls_map_get_status();
    if (_ls_stepper_enable_skipping)
    {
        _ls_stepper_speed_not_skipping = _ls_stepper_steps_per_second_max;
    }
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_RANDOM;
    message.steps = 0;
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
}

int32_t IRAM_ATTR ls_stepper_get_position(void)
{
    return ls_stepper_position;
};
void IRAM_ATTR ls_stepper_set_home_position(void)
{
    ls_stepper_position = 0;
}

bool ls_stepper_is_stopped(void)
{
    return 0 == ls_stepper_steps_remaining ? true : false;
}

BaseType_t ls_stepper_get_steps_taken(void)
{
    return ls_stepper_steps_taken;
}

#ifdef LSDEBUG_STEPPER
void ls_stepper_debug_task(void *pvParameter)
{
    while (1)
    {
        ls_debug_printf("STEPPER DEBUG: position=%d; remaining=%d; taken=%d; direction=%d, step_phase=%d\n", ls_stepper_position, ls_stepper_steps_remaining, ls_stepper_steps_taken, ls_stepper_direction, _ls_stepperstep_phase);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif