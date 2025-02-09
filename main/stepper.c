/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2024 David H. Brown

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
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
#include "tapemode.h"
#include "math.h"

#define LS_STEPPER_TIMER_DIVIDER (10)

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
// Rather than aiming for 1ms pulses, toggling at the total timer count for
// a square(ish) wave would make sense.
// To spin fast, we need rotations at least of 100rpm and maybe up to 400rpm... and 0 to change direction safely.
// 400 RPM => 6.667 RPS => 21333.3333 pulses per second => 42666.6667 alarms per second
// 80,000,000 (80MHz) / 42667 (aps) => 1875 alarm value (1874.9853) with no timer divider. 
// The timer diver of 20 used before would mean that 400RPM is an alarm of about 94; 
// 100RPM is an alarm value of 375. I guess this would work okay for the timer.
// 1R => 6400 alarms. 1RPM = 106.67 alarms/second, alarm value of about 37500.
// alarm_value(rpm) => APB_CLK_FREQ / LS_STEPPER_TIMER_DIVIDER / LS_STEPPER_STEPS_PER_ROTATION / 2 * 60 / rpm
// rpm(alarm_value) => alarm_value / (APB_CLK_FREQ / LS_STEPPER_TIMER_DIVIDER / LS_STEPPER_STEPS_PER_ROTATION / 2 * 60)
// The TMC2209 driver specifies a typical minimum low/high time for a step pulse of 100ns, [sec. 13, Step/Dir interface]
// or a frequency of 500MHz, so we couldn't make it too short given the 80MHz starting clock.
// However, there is also a frequency limit of half the clock (internal is 12MHz) at the maximum microstepping. 
// 6MHz pulses => 12MHz alarms... yeah, no way we need to worry about that.
// July 26 '24: Trying 10 instead of 20 to get better control of acceleration and deceleration

const uint64_t _alarms_1_rpm = APB_CLK_FREQ / LS_STEPPER_TIMER_DIVIDER / LS_STEPPER_STEPS_PER_ROTATION / 2ULL * 60ULL;

/**
 * Calculate the resulting RPM for a given timer alarm value
 */
BaseType_t _rpm_from_timer_alarm_value(uint64_t timer_alarm_value) {
    return (BaseType_t) (_alarms_1_rpm / timer_alarm_value);
}

/**
 * Calculate the timer alarm value for a desired RPM
 * Values == 0 => an alarm count equivalent to 0.5 RPM
 */
uint64_t _timer_alarm_count_from_rpm(BaseType_t rpm) {
    if (rpm == 0) {
        return _alarms_1_rpm * 2ULL;
    }

    return _alarms_1_rpm / (uint64_t) abs(rpm);
}


volatile BaseType_t IRAM_ATTR ls_stepper_mode_hop0_spin1; // 0/false for hops; 1/true for spins.
volatile BaseType_t IRAM_ATTR ls_stepper_steps_remaining; // hopping
volatile BaseType_t IRAM_ATTR ls_stepper_steps_taken; // hopping
uint64_t IRAM_ATTR ls_stepper_current_timer_alarm_count = _alarms_1_rpm * 2; // spinning
uint64_t IRAM_ATTR ls_stepper_target_timer_alarm_count = _alarms_1_rpm * 2; // spinning
uint64_t IRAM_ATTR ls_stepper_maximum_laser_enable_alarm_count; // set during ls_stepper_init() based on ls_settings_get_mode()
uint64_t ls_stepper_timer_alarm_count_stoppable;

volatile static BaseType_t IRAM_ATTR _ls_stepperstep_phase = 0;
enum ls_stepper_rotation_mode _current_stepper_rotation_mode;
enum ls_stepper_action _current_stepper_action;
enum ls_stepper_direction_t IRAM_ATTR ls_stepper_direction = LS_STEPPER_DIRECTION_FORWARD;
#ifdef LS_HAS_TAPE_SENSOR
static bool _ls_stepper_enable_skipping = false;
static int _ls_stepper_speed_when_skipping = LS_STEPPER_STEPS_PER_SECOND_MAX;
static int _ls_stepper_speed_not_skipping = LS_STEPPER_STEPS_PER_SECOND_DEFAULT;
#endif
static int _ls_stepper_steps_per_second_max = LS_STEPPER_STEPS_PER_SECOND_DEFAULT;

static int _ls_stepper_speed_current_hop_rate = LS_STEPPER_STEPS_PER_SECOND_MIN;

static uint8_t _ls_stepper_random_reverse_per255 = LS_STEPPER_RANDOM_HOP_REVERSE_PER255;

// how many steps it will take to decelerate from full speed
static int _ls_stepper_steps_to_decelerate(int current_rate)
{
    // used Excel to fit a quadratic curve (y=ax^2+bx+c) the the calculated deceleration steps
    // the constant term might want to change if the speed limit is changed
    return (current_rate * current_rate) / (2 * LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_SECOND) + // 'a' coefficient on x^2 is the reciprocal of 2x the steps delta
           (current_rate / 20) +                                                              // the 'b' coefficient is consistently 0.05... not sure exactly why
           10;                                                                                // 'c' constant term for steps left over after steps delta has been removed each time (3600 is not divisible by 800)
}

void _ls_stepper_enqueue_finished_move()
{
    ls_event event;
    event.type = LSEVT_STEPPER_FINISHED_MOVE;
    event.value = 0;
    xQueueSendToFrontFromISR(ls_event_queue, (void *)&event, NULL);
}

void _ls_stepper_enqueue_reached_speed()
{
    ls_event event;
    event.type = LSEVT_STEPPER_REACHED_SPEED;
    event.value = 0;
    xQueueSend(ls_event_queue, (void *)&event, 0);
}

ls_stepper_position_t ls_stepper_position_constrained(ls_stepper_position_t position)
{
    while(position < 0) {
        position += LS_STEPPER_STEPS_PER_ROTATION;
    }
    return position % LS_STEPPER_STEPS_PER_ROTATION;
}

void ls_stepper_set_maximum_steps_per_second(int steps_per_second)
{
#ifdef LSDEBUG_STEPPER
    bool changed = steps_per_second != _ls_stepper_steps_per_second_max;
#endif
    // set and constrain new current speed limit (except if doing the warning)
    if (steps_per_second != LS_STEPPER_STEPS_PER_SECOND_WARNING)
    {
        steps_per_second = _constrain(steps_per_second, LS_STEPPER_STEPS_PER_SECOND_MIN, LS_STEPPER_STEPS_PER_SECOND_MAX);
    }
    _ls_stepper_steps_per_second_max = steps_per_second;
#ifdef LSDEBUG_STEPPER
    // might be called before print mutex is set up, so no debug_printf
    if (changed)
    {
        printf("Stepper speed set to %d max steps/s (%d requested); %d steps to decelerate.\n",
               _ls_stepper_steps_per_second_max, steps_per_second, _ls_stepper_steps_to_decelerate(_ls_stepper_steps_per_second_max));
    }
#endif
}

static bool IRAM_ATTR ls_stepper_step_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    // turn the laser off if spinning and too slow
    if (1==ls_stepper_mode_hop0_spin1 && ls_stepper_current_timer_alarm_count > ls_stepper_maximum_laser_enable_alarm_count)
    {
        gpio_set_level(LSGPIO_LASERPOWERENABLE,0);
    }
    else if (ls_laser_mode_is_on())
    {
        gpio_set_level(LSGPIO_LASERPOWERENABLE,1);
    }
    
    bool stepping = 0==ls_stepper_mode_hop0_spin1 ? 
        ls_stepper_steps_remaining > 0 // if hopping
        :
        ls_stepper_current_timer_alarm_count < ls_stepper_timer_alarm_count_stoppable // if spinning
        ;


    if (stepping) {
        _ls_stepperstep_phase = 1 - _ls_stepperstep_phase;
        gpio_set_level(LSGPIO_STEPPERSTEP, _ls_stepperstep_phase);
        if ( _ls_stepperstep_phase)  { // beginning a step pulse (high)
            ls_stepper_position += ls_stepper_direction * 2 - 1;
            while (ls_stepper_position < 0)
            {
                ls_stepper_position += LS_STEPPER_STEPS_PER_ROTATION;
            }
            while (ls_stepper_position >= LS_STEPPER_STEPS_PER_ROTATION)
            {
                ls_stepper_position -= LS_STEPPER_STEPS_PER_ROTATION;
            }
#ifdef LS_HAS_TAPE_SENSOR    
            if (ls_laser_mode_is_mappped())
            {
                gpio_set_level(LSGPIO_LASERPOWERENABLE, ls_map_is_enabled_at(ls_stepper_position));
            }
#endif
        } // if beginning pulse
        else
        { // ending the step pulse (low)
            if (0 == ls_stepper_mode_hop0_spin1){
                ls_stepper_steps_remaining--;
                ls_stepper_steps_taken++;
                if (ls_stepper_steps_remaining == 0)
                {
                    _ls_stepper_enqueue_finished_move();
                }
            }
        } // else ending pulse
    } // if stepping

    /* See timer_group_example for how to use this: */
    //    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

void ls_stepper_init(void)
{
    if(ls_spinmode()==LS_SPINMODE_1M) {
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: Laser will be enabled when spinning at least %d RPM [1m].\n", LS_LASER_ENABLE_MINIMUM_RPM_1M);
#endif
        ls_stepper_maximum_laser_enable_alarm_count = _alarms_1_rpm / LS_LASER_ENABLE_MINIMUM_RPM_1M;
    } else if (ls_spinmode_is_spinning())
    {
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: Laser will be enabled when spinning at least %d RPM [100mm].\n", LS_LASER_ENABLE_MINIMUM_RPM_100MM);
#endif
        ls_stepper_maximum_laser_enable_alarm_count = _alarms_1_rpm / LS_LASER_ENABLE_MINIMUM_RPM_100MM;
    }
    gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_DISABLE); // don't step while we get ready
    gpio_set_level(LSGPIO_STEPPERDIRECTION, LS_STEPPER_DIRECTION_FORWARD); // reasonable default
    ls_stepper_position = 0;
    ls_stepper_direction = LS_STEPPER_DIRECTION_FORWARD; 
    bootloader_random_enable();
    ls_stepper_queue = xQueueCreate(8, sizeof(ls_stepper_action_message));
    ls_stepper_steps_remaining = 0;
    ls_stepper_steps_taken = 0;
    ls_stepper_mode_hop0_spin1 = 0; // just to ensure initialization; any move request must set
    ls_stepper_timer_alarm_count_stoppable = _timer_alarm_count_from_rpm(LS_SETTINGS_MINIMUM_RPM_MOVEMENT); // spinning
    ls_stepper_current_timer_alarm_count = ls_stepper_timer_alarm_count_stoppable;
    ls_stepper_set_random_hop_strategy(ls_stepper_random_strategy_default);
    ls_stepper_move.direction = LS_STEPPER_DIRECTION_FORWARD;
    ls_stepper_move.steps = 0;
    timer_config_t stepper_step_timer_config = {
        .divider = LS_STEPPER_TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &stepper_step_timer_config));
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0ULL));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ls_stepper_current_timer_alarm_count));
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, ls_stepper_step_isr_callback, NULL, 0));
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
}



static void _ls_stepper_set_hop_speed(void)
{
#ifdef LS_HAS_TAPE_SENSOR    
    // if doing random movements and outside a span, allow faster movement (skipping)
    if (_ls_stepper_enable_skipping)
    {
        ls_stepper_set_maximum_steps_per_second(ls_map_is_enabled_at(ls_stepper_position) ? _ls_stepper_speed_not_skipping : _ls_stepper_speed_when_skipping);
    }
#endif
    int steps_to_decelerate = _ls_stepper_steps_to_decelerate(_ls_stepper_speed_current_hop_rate);

    bool could_accelerate = (int)ls_stepper_steps_remaining > steps_to_decelerate && _ls_stepper_speed_current_hop_rate < _ls_stepper_steps_per_second_max;
    bool should_decelerate = (int)ls_stepper_steps_remaining < steps_to_decelerate || _ls_stepper_speed_current_hop_rate > _ls_stepper_steps_per_second_max;
    // are enough steps remaining to accelerate?
    if (could_accelerate)
    {
        _ls_stepper_speed_current_hop_rate += LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_TICK;
    }
    if (should_decelerate)
    {
        // decelerate
        _ls_stepper_speed_current_hop_rate -= LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_TICK;
    }
#ifdef LSDEBUG_ACCELERATION
    ls_debug_printf("ACCEL: Current rate/max: %d/%d; steps to decelerate: %d; steps remaining: %d; %c %c\n", _ls_stepper_speed_current_hop_rate, _ls_stepper_steps_per_second_max, steps_to_decelerate, ls_stepper_steps_remaining,
                    could_accelerate ? '+' : ' ', should_decelerate ? '-' : ' ');
#endif
    // but stay within bounds
    _ls_stepper_speed_current_hop_rate = _constrain(_ls_stepper_speed_current_hop_rate, LS_STEPPER_STEPS_PER_SECOND_MIN, _ls_stepper_steps_per_second_max);

    // OOPS: forgot to divide by two (because each alarm is only half the square wave pulse).

    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, APB_CLK_FREQ / LS_STEPPER_TIMER_DIVIDER / _ls_stepper_speed_current_hop_rate));
}

static void _ls_stepper_set_spin_speed(void)
{
    // too fast:
    if (ls_stepper_current_timer_alarm_count < ls_stepper_target_timer_alarm_count)
    {

            ls_stepper_current_timer_alarm_count += ((ls_stepper_current_timer_alarm_count/LS_STEPPER_SPINNING_ALARMS_CHANGE_DIVISOR) + LS_STEPPER_SPINNING_ALARMS_CHANGE_MINIMUM); 
            if (ls_stepper_current_timer_alarm_count > ls_stepper_target_timer_alarm_count) {
                ls_stepper_current_timer_alarm_count = ls_stepper_target_timer_alarm_count;
            }
#ifdef LSDEBUG_STEPPER
            // ls_debug_printf("-"); // greater alarm count is slower
#endif
    }
    // too slow:
    if (ls_stepper_current_timer_alarm_count > ls_stepper_target_timer_alarm_count) 
    {
            ls_stepper_current_timer_alarm_count -= ((ls_stepper_current_timer_alarm_count/LS_STEPPER_SPINNING_ALARMS_CHANGE_DIVISOR) + LS_STEPPER_SPINNING_ALARMS_CHANGE_MINIMUM);
            if (ls_stepper_current_timer_alarm_count < ls_stepper_target_timer_alarm_count) {
                ls_stepper_current_timer_alarm_count = ls_stepper_target_timer_alarm_count;
            }
#ifdef LSDEBUG_STEPPER
            // ls_debug_printf("+"); // lower alarm count is faster
#endif
    }
#ifdef LSDEBUG_STEPPER
            // ls_debug_printf("[^%d]", (int) ls_stepper_current_timer_alarm_count); // too much
#endif
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ls_stepper_current_timer_alarm_count));
}

void _ls_stepper_enqueue_idle_action(int32_t value)
{
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_IDLE;
    message.value = value;
    xQueueSendToFront(ls_stepper_queue, (void *)&message, 0);
}

void _ls_stepper_stop_next_action(void)
{
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_STOP;
    message.value = 0;
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: _ls_stepper_stop_next_action() called\n");
#endif
}

void _ls_stepper_repeat_action(ls_stepper_action_message *current_message)
{
    ls_stepper_action_message message;
    message.action = current_message->action;
    message.value = current_message->value;
    if (LS_STEPPER_ACTION_IDLE == message.action && message.value > 0) {
        message.value--;
    }
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
#ifdef LSDEBUG_STEPPER
if (LS_STEPPER_ACTION_IDLE != message.action || message.value ==0) {
    ls_debug_printf("STEPPER: _ls_stepper_repeat_action(action=%d, value=%d)\n", message.action, message.value);
}
#endif
}

enum ls_stepper_rotation_mode _do_state_hopping(enum ls_stepper_rotation_mode current_rotation_mode, ls_stepper_action_message *current_message)
{
    enum ls_stepper_action current_action = current_message->action;
    int32_t current_value = current_message->value;
    enum ls_stepper_rotation_mode successor_rotation_mode = current_rotation_mode;
    switch (current_action)
    {
    case LS_STEPPER_ACTION_HOP_FORWARD_STEPS: // LS_STEPPER_ROTATION_MODE_HOPPING|LS_STEPPER_ROTATION_MODE_RANDOM_HOP
        gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_ENABLE);
        if (ls_stepper_steps_remaining <= 0)
        {
#ifdef LSDEBUG_STEPPER
            ls_debug_printf("STEPPER: begin hop %d step(s)\n", current_value);
#endif
            ls_stepper_direction = LS_STEPPER_DIRECTION_FORWARD;
            gpio_set_level(LSGPIO_STEPPERDIRECTION, ls_stepper_direction);
            ls_stepper_steps_taken = 0;
            ls_stepper_steps_remaining = current_value;
            _ls_stepper_enqueue_idle_action(0);
        }
        else
        {
#ifdef LSDEBUG_STEPPER
            ls_debug_printf("STEPPER: extend hop by %d step(s)\n", current_value);
#endif
            ls_stepper_steps_remaining += current_value;
            _ls_stepper_enqueue_idle_action(0);
        }
        _ls_stepper_set_hop_speed();
        break;
    case LS_STEPPER_ACTION_HOP_REVERSE_STEPS: // LS_STEPPER_ROTATION_MODE_HOPPING|LS_STEPPER_ROTATION_MODE_RANDOM_HOP
        gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_ENABLE);
        if (ls_stepper_steps_remaining <= 0)
        {
#ifdef LSDEBUG_STEPPER
            ls_debug_printf("STEPPER: begin hop of -%d step(s)\n", current_value);
#endif
            ls_stepper_direction = LS_STEPPER_DIRECTION_REVERSE;
            gpio_set_level(LSGPIO_STEPPERDIRECTION, ls_stepper_direction);
            ls_stepper_steps_taken = 0;
            ls_stepper_steps_remaining = current_value;
            _ls_stepper_enqueue_idle_action(0);
        }
        else
        {
#ifdef LSDEBUG_STEPPER
            ls_debug_printf("STEPPER: extend hop by -%d step(s)\n", current_value);
#endif
            ls_stepper_steps_remaining += current_value;
            _ls_stepper_enqueue_idle_action(0);
        }
        _ls_stepper_set_hop_speed();
        break;
    case LS_STEPPER_ACTION_STOP: // LS_STEPPER_ROTATION_MODE_HOPPING|LS_STEPPER_ROTATION_MODE_RANDOM_HOP
#ifdef LSDEBUG_STEPPER
        ls_debug_printf("STEPPER: stopping\n");
#endif
        gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_ENABLE); // enabled because we need to control deceleration
        ls_stepper_steps_remaining = _constrain(ls_stepper_steps_remaining, 0, _ls_stepper_steps_to_decelerate(_ls_stepper_speed_current_hop_rate));
        _ls_stepper_set_hop_speed();
        if (ls_stepper_steps_remaining <= 0)
        {
            successor_rotation_mode = LS_STEPPER_ROTATION_MODE_STOPPED;
        }
        break;
    case LS_STEPPER_ACTION_IDLE: // LS_STEPPER_ROTATION_MODE_HOPPING|LS_STEPPER_ROTATION_MODE_RANDOM_HOP
        if(ls_stepper_mode_hop0_spin1 == 0) {
            _ls_stepper_set_hop_speed();
        } else {
            _ls_stepper_set_spin_speed();
        }
        break;
    case LS_STEPPER_ACTION_RANDOM_HOP: // LS_STEPPER_ROTATION_MODE_HOPPING|LS_STEPPER_ROTATION_MODE_RANDOM_HOP
        _ls_stepper_set_hop_speed();
        successor_rotation_mode = LS_STEPPER_ROTATION_MODE_RANDOM_HOP;
        break;
    case LS_STEPPER_ACTION_RANDOM_SPIN: // LS_STEPPER_ROTATION_MODE_HOPPING|LS_STEPPER_ROTATION_MODE_RANDOM_HOP
        _ls_stepper_set_spin_speed();
        successor_rotation_mode = LS_STEPPER_ROTATION_MODE_RANDOM_SPIN;
        break;
    case LS_STEPPER_ACTION_TARGET_RPM: // LS_STEPPER_ROTATION_MODE_HOPPING|LS_STEPPER_ROTATION_MODE_RANDOM_HOP
        _ls_stepper_set_spin_speed();
        successor_rotation_mode = LS_STEPPER_ROTATION_MODE_SPINNING;
        break;
    case LS_STEPPER_ACTION_SLEEP: // LS_STEPPER_ROTATION_MODE_HOPPING|LS_STEPPER_ROTATION_MODE_RANDOM_HOP
        _ls_stepper_stop_next_action();
        break;
    case LS_STEPPER_ACTION_SPIN:
        _ls_stepper_set_spin_speed();
        _ls_stepper_repeat_action(current_message);
    break;
    }
    return successor_rotation_mode;
}
void _ls_stepper_set_direction_and_timer_for_rpm(int rpm){
    enum ls_stepper_direction_t new_direction = (rpm >= 0) ? LS_STEPPER_DIRECTION_FORWARD : LS_STEPPER_DIRECTION_REVERSE;
    uint64_t new_count = _timer_alarm_count_from_rpm(rpm);
    if((new_direction != ls_stepper_direction) || (new_count != ls_stepper_target_timer_alarm_count))
    {
        gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_ENABLE);
        ls_stepper_direction = new_direction;
        ls_stepper_target_timer_alarm_count = new_count;
        gpio_set_level(LSGPIO_STEPPERDIRECTION, ls_stepper_direction);
    }
    _ls_stepper_set_spin_speed();
    if(ls_stepper_current_timer_alarm_count == ls_stepper_target_timer_alarm_count)
    {
        _ls_stepper_enqueue_reached_speed();
    }
}

enum ls_stepper_rotation_mode _do_state_spinning(enum ls_stepper_rotation_mode current_rotation_mode, ls_stepper_action_message *current_message)
{
    enum ls_stepper_rotation_mode successor_rotation_mode = current_rotation_mode;

    switch (current_message->action)
    {
    case LS_STEPPER_ACTION_IDLE:
        _ls_stepper_set_spin_speed();
        break;
    case LS_STEPPER_ACTION_HOP_FORWARD_STEPS:
    case LS_STEPPER_ACTION_HOP_REVERSE_STEPS:
    case LS_STEPPER_ACTION_RANDOM_HOP:
    case LS_STEPPER_ACTION_SLEEP:
        _ls_stepper_stop_next_action();
        _ls_stepper_repeat_action(current_message);
    break;
    case LS_STEPPER_ACTION_STOP:
        if (ls_stepper_current_timer_alarm_count < ls_stepper_timer_alarm_count_stoppable)
        {
            ls_stepper_target_timer_alarm_count = ls_stepper_timer_alarm_count_stoppable;
        }
        // do anything we need to do exactly once (send):
        if (ls_stepper_current_timer_alarm_count == ls_stepper_timer_alarm_count_stoppable) {
            _ls_stepper_enqueue_finished_move();
#ifdef LSDEBUG_STEPPER
            ls_debug_printf("STEPPER: _do_state_spinning: Slowed to stoppable speed\n");
#endif
            /// then go to a slightly slower speed
            ls_stepper_target_timer_alarm_count += 2 * LS_STEPPER_SPINNING_ALARMS_CHANGE_MINIMUM;
            ls_stepper_current_timer_alarm_count = ls_stepper_target_timer_alarm_count;
            successor_rotation_mode = LS_STEPPER_ROTATION_MODE_STOPPED;
        }
        if (ls_stepper_current_timer_alarm_count > ls_stepper_timer_alarm_count_stoppable) {
            // already did what we need to do, but just in case:
            successor_rotation_mode = LS_STEPPER_ROTATION_MODE_STOPPED;
        }
        _ls_stepper_set_spin_speed();
        break;
    case LS_STEPPER_ACTION_RANDOM_SPIN:
        if(_current_stepper_rotation_mode != LS_STEPPER_ROTATION_MODE_RANDOM_SPIN) {
            // _ls_stepper_enqueue_idle_action(1);
            successor_rotation_mode=LS_STEPPER_ROTATION_MODE_RANDOM_SPIN;
        } else {
            _ls_stepper_set_spin_speed();            
        }
        break;
    case LS_STEPPER_ACTION_SPIN: // will have been held until stopped if direction changing
        successor_rotation_mode = LS_STEPPER_ROTATION_MODE_SPINNING;
        _ls_stepper_set_direction_and_timer_for_rpm(current_message->value);
        break;
    case LS_STEPPER_ACTION_TARGET_RPM: // will have been held until stopped if direction changing
        _ls_stepper_set_direction_and_timer_for_rpm(current_message->value);
        break;
    }
    return successor_rotation_mode;
}



void _ls_enqueue_random_spin_target_rpm(void) {
    // pick random ls_stepper_target_timer_alarm_count
    uint32_t random = esp_random();
    uint8_t rand_rpm = random & 0xFF;
    uint8_t rand_dir = (random >> 8) & 0xFF;
#ifdef LSDEBUG_STEPPER_RANDOM
    ls_debug_printf("STEPPER_RANDOM: _ls_enqueue_random_spin_target_rpm: rand_rpm=%d; rand_dir = %d.\n", rand_rpm, rand_dir);
#endif
    /** @todo get max from setting, not necessarily range limit */
    BaseType_t rpm = _map(rand_rpm, 0, 255, ls_settings_get_minimum_rpm(), ls_settings_get_maximum_rpm());
    // if we're already spinning backwards, keep spinning backwards
    if(LS_STEPPER_DIRECTION_REVERSE == ls_stepper_direction) {
        rpm = -rpm;
    }
    // unless we (randomly) want to change direction
    if(rand_dir < LS_STEPPER_RANDOM_SPIN_REVERSE_PER255) {
        rpm = -rpm;
#ifdef LSDEBUG_STEPPER_RANDOM
    ls_debug_printf("STEPPER_RANDOM: Random spin changing direction\n");
#endif
    }
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_TARGET_RPM;
    message.value = rpm;
    if(pdTRUE == xQueueSend(ls_stepper_queue, (void *) &message, 0)) {
#ifdef LSDEBUG_STEPPER_RANDOM
    ls_debug_printf("STEPPER_RANDOM: Random spin targeting %d RPM.\n", message.value);
#endif
    ;//ok
    }
    else {
        fprintf(stderr, "STEPPER_RANDOM: Could not add RPM to ls_stepper_queue\n");
    };
}

void _ls_enqueue_random_spin_idle_ticks(void) {
    // pick random ticks to hold speed
    uint32_t random = esp_random();
    uint16_t rand_ticks = random & 0xFFFF;
    BaseType_t ticks = _map(rand_ticks, 0, 65535, pdMS_TO_TICKS(1000*LS_STEPPER_RANDOM_SPIN_MINIMUM_SECONDS), pdMS_TO_TICKS(1000*LS_STEPPER_RANDOM_SPIN_MAXIMUM_SECONDS));
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_IDLE;
    message.value = ticks;
    xQueueSend(ls_stepper_queue, (void *) &message, 0);
#ifdef LSDEBUG_STEPPER_RANDOM
    ls_debug_printf("STEPPER_RANDOM: Random spin reached target RPM; will hold for at least %d ticks.\n", message.value);
#endif
}

void ls_stepper_task(void *pvParameter)
{
    ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_DEFAULT);
    ls_stepper_action_message message;
    _current_stepper_action = message.action = LS_STEPPER_ACTION_SLEEP;
    _current_stepper_rotation_mode = LS_STEPPER_ROTATION_MODE_UNPOWERED;
    enum ls_stepper_rotation_mode successor_stepper_rotation_mode = _current_stepper_rotation_mode;

// temporary to check arithmetic 
// #ifdef LSDEBUG_STEPPER
//     printf("_alarms_1_rpm=%lld\n", _alarms_1_rpm);
//     for(int i=50; i <= 500; i+=50){
//         printf("%d RPM => %lld alarm counts\n", i, _timer_alarm_count_from_rpm(i));
//     }
//     vTaskDelay(pdMS_TO_TICKS(5000));
// #endif
 

    while (1)
    {
        if (uxQueueMessagesWaiting(ls_stepper_queue) > 0)
        {
            // peek at message queue because we may need to stop first
            if (xQueuePeek(ls_stepper_queue, &message, 0)) {
                bool hop_other_direction = (
                    (LS_STEPPER_ACTION_HOP_FORWARD_STEPS == message.action && LS_STEPPER_DIRECTION_REVERSE == ls_stepper_direction) ||
                    (LS_STEPPER_ACTION_HOP_REVERSE_STEPS == message.action && LS_STEPPER_DIRECTION_FORWARD == ls_stepper_direction)
                ) && ls_stepper_steps_remaining > 0;

                bool spin_other_direction = (
                    (LS_STEPPER_ACTION_SPIN == message.action && message.value > 0 && LS_STEPPER_DIRECTION_REVERSE == ls_stepper_direction) ||
                    (LS_STEPPER_ACTION_SPIN == message.action && message.value > 0 && LS_STEPPER_DIRECTION_REVERSE == ls_stepper_direction) ||
                    (LS_STEPPER_ACTION_TARGET_RPM == message.action && message.value > 0 && LS_STEPPER_DIRECTION_REVERSE == ls_stepper_direction) ||
                    (LS_STEPPER_ACTION_TARGET_RPM == message.action && message.value < 0 && LS_STEPPER_DIRECTION_FORWARD == ls_stepper_direction)
                ) && ls_stepper_current_timer_alarm_count < ls_stepper_timer_alarm_count_stoppable;

                bool change_stepper_mode = (
                    // hop actions incompatible with spinning:
                    (LS_STEPPER_ACTION_HOP_FORWARD_STEPS == message.action && LS_STEPPER_ROTATION_MODE_RANDOM_SPIN == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_HOP_FORWARD_STEPS == message.action && LS_STEPPER_ROTATION_MODE_SPINNING == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_HOP_REVERSE_STEPS == message.action && LS_STEPPER_ROTATION_MODE_RANDOM_SPIN == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_HOP_REVERSE_STEPS == message.action && LS_STEPPER_ROTATION_MODE_SPINNING == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_RANDOM_HOP == message.action && LS_STEPPER_ROTATION_MODE_RANDOM_SPIN == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_RANDOM_HOP == message.action && LS_STEPPER_ROTATION_MODE_SPINNING == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_SLEEP == message.action && LS_STEPPER_ROTATION_MODE_SPINNING == _current_stepper_rotation_mode) ||
                    // spinning actions incompatible with hopping
                    (LS_STEPPER_ACTION_RANDOM_SPIN == message.action && LS_STEPPER_ROTATION_MODE_RANDOM_HOP == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_RANDOM_SPIN == message.action && LS_STEPPER_ROTATION_MODE_HOPPING == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_SPIN == message.action && LS_STEPPER_ROTATION_MODE_RANDOM_HOP == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_SPIN == message.action && LS_STEPPER_ROTATION_MODE_HOPPING == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_TARGET_RPM == message.action && LS_STEPPER_ROTATION_MODE_RANDOM_HOP == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_TARGET_RPM == message.action && LS_STEPPER_ROTATION_MODE_HOPPING == _current_stepper_rotation_mode) ||
                    (LS_STEPPER_ACTION_SLEEP == message.action && LS_STEPPER_ROTATION_MODE_HOPPING == _current_stepper_rotation_mode) ||
                    false // if not incompatible
                );

                if  ( hop_other_direction || spin_other_direction || 
                    change_stepper_mode)
                {
    #ifdef LSDEBUG_STEPPER
    if(_current_stepper_action != LS_STEPPER_ACTION_STOP) {
                   ls_debug_printf("STEPPER: stopping before change in mode or direction (stepper mode = %d; stepper action=%d (next=%d);  direction=%d)\n", 
                       _current_stepper_rotation_mode, _current_stepper_action, message.action, ls_stepper_direction);
    }
    #endif
                    _current_stepper_action = message.action = LS_STEPPER_ACTION_STOP;
                } // if next message requires stopping before taking its action
            else // we don't have to stop before dequeueing the next message
            { // dequeue the message
                if (xQueueReceive(ls_stepper_queue, &message, 0))
                {
                    _current_stepper_action = message.action;
#ifdef LSDEBUG_STEPPER
                    ls_debug_printf("STEPPER: dequeued action %d (value = %d); current mode is %d\n",
                           message.action, message.value, _current_stepper_rotation_mode);
#endif
                } // if we succeeded in dequeuing the message
            } // else (dequeue the message; don't have to stop first)
            } // if we were able to peek at the next message (of course we were, because we also checked...)
        }     // if there was anything in the message queue
        // now process according to current state
        successor_stepper_rotation_mode = _current_stepper_rotation_mode;
        switch (_current_stepper_rotation_mode) {
            case LS_STEPPER_ROTATION_MODE_UNPOWERED: // 0
                gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_DISABLE);
                switch(_current_stepper_action) {
                    case LS_STEPPER_ACTION_SLEEP: // we're already there... nothing to do
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("STEPPER: already sleeping\n");
#endif
                        _current_stepper_action = LS_STEPPER_ACTION_IDLE;
                        break;
                    case LS_STEPPER_ACTION_HOP_FORWARD_STEPS: // LS_STEPPER_ROTATION_MODE_UNPOWERED
                    case LS_STEPPER_ACTION_HOP_REVERSE_STEPS: // LS_STEPPER_ROTATION_MODE_UNPOWERED
                    case LS_STEPPER_ACTION_RANDOM_HOP: // LS_STEPPER_ROTATION_MODE_UNPOWERED
                    case LS_STEPPER_ACTION_RANDOM_SPIN: // LS_STEPPER_ROTATION_MODE_UNPOWERED
                    case LS_STEPPER_ACTION_TARGET_RPM: // LS_STEPPER_ROTATION_MODE_UNPOWERED
                    case LS_STEPPER_ACTION_SPIN:
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("STEPPER: waking; requeueing message action=%d value=%d\n", message.action, message.value);
#endif
                        _ls_stepper_repeat_action(&message);
                        successor_stepper_rotation_mode = LS_STEPPER_ROTATION_MODE_STOPPED;
                    break;
                    case LS_STEPPER_ACTION_STOP: // already done before we could shut down

                    case LS_STEPPER_ACTION_IDLE:
                    break;
                } // switch action for LS_STEPPER_ROTATION_MODE_UNPOWERED
            break;
            case LS_STEPPER_ROTATION_MODE_STOPPED: // 1
                switch(message.action) {
                    case LS_STEPPER_ACTION_IDLE: // LS_STEPPER_ROTATION_MODE_STOPPED
                    break;
                    case LS_STEPPER_ACTION_HOP_FORWARD_STEPS: // LS_STEPPER_ROTATION_MODE_STOPPED
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("STEPPER: starting; NOT requeueing message action=%d value=%d\n", message.action, message.value);
#endif
                        // _ls_stepper_repeat_action(&message);
                    successor_stepper_rotation_mode = LS_STEPPER_ROTATION_MODE_HOPPING;
                    break;
                    case LS_STEPPER_ACTION_HOP_REVERSE_STEPS: // LS_STEPPER_ROTATION_MODE_STOPPED
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("STEPPER: starting; NOT requeueing message action=%d value=%d\n", message.action, message.value);
#endif
                        // _ls_stepper_repeat_action(&message);
                    successor_stepper_rotation_mode = LS_STEPPER_ROTATION_MODE_HOPPING;
                    break;
                    case LS_STEPPER_ACTION_RANDOM_HOP: // LS_STEPPER_ROTATION_MODE_STOPPED
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("STEPPER: starting; NOT requeueing message action=%d value=%d\n", message.action, message.value);
#endif
                        // _ls_stepper_repeat_action(&message);
                    successor_stepper_rotation_mode = LS_STEPPER_ROTATION_MODE_RANDOM_HOP;
                    break;
                    case LS_STEPPER_ACTION_RANDOM_SPIN: // LS_STEPPER_ROTATION_MODE_STOPPED
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("STEPPER: starting; NOT requeueing message action=%d value=%d\n", message.action, message.value);
#endif
                        // _ls_stepper_repeat_action(&message);
                    successor_stepper_rotation_mode = LS_STEPPER_ROTATION_MODE_RANDOM_SPIN;
                    break;
                    case LS_STEPPER_ACTION_SPIN:
                    case LS_STEPPER_ACTION_TARGET_RPM: // LS_STEPPER_ROTATION_MODE_STOPPED
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("STEPPER: starting; NOT requeueing message action=%d value=%d\n", message.action, message.value);
#endif
                        // _ls_stepper_repeat_action(&message);
                    successor_stepper_rotation_mode = LS_STEPPER_ROTATION_MODE_SPINNING;
                    break;
                    case LS_STEPPER_ACTION_SLEEP: // LS_STEPPER_ROTATION_MODE_STOPPED
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("STEPPER: starting; NOT requeueing message action=%d value=%d\n", message.action, message.value);
#endif
                        // _ls_stepper_repeat_action(&message);
                    successor_stepper_rotation_mode = LS_STEPPER_ROTATION_MODE_UNPOWERED;
                    break;
                    case LS_STEPPER_ACTION_STOP: // LS_STEPPER_ROTATION_MODE_STOPPED
#ifdef LSDEBUG_STEPPER
                ls_debug_printf("STEPPER: has stopped.\n");
#endif
                    break;
                } // switch current_action for LS_STEPPER_ROTATION_MODE_STOPPED
#ifdef LSDEBUG_STEPPER
                // ls_debug_printf("Stepper starting\n");
#endif
            break;
            case LS_STEPPER_ROTATION_MODE_RANDOM_HOP: // 2
                gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_ENABLE);
                ls_stepper_mode_hop0_spin1 = 0;
                if (ls_stepper_steps_remaining <= 0)
                {
#ifdef LSDEBUG_STEPPER_RANDOM
ls_debug_printf("Finished move; calling the _ls_stepper_random_strategy\n");
#endif                    
                    // invoke the current move strategy
                    (*_ls_stepper_random_strategy)(&ls_stepper_move);
                    ls_stepper_direction = ls_stepper_move.direction;
                    ls_stepper_steps_remaining = ls_stepper_move.steps;
                    ls_stepper_steps_taken = 0;
                    gpio_set_level(LSGPIO_STEPPERDIRECTION, ls_stepper_direction);
    #ifdef LSDEBUG_STEPPER
                    ls_debug_printf("STEPPER: from %d, moving %d steps %s\n", ls_stepper_position, ls_stepper_steps_remaining, ls_stepper_direction ? "-->" : "<--");
    #endif
                } // finished move
                successor_stepper_rotation_mode = _do_state_hopping(_current_stepper_rotation_mode, &message);
            break;
            case LS_STEPPER_ROTATION_MODE_RANDOM_SPIN: // 3
                gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_ENABLE);
                ls_stepper_mode_hop0_spin1 = 1;
                switch(_current_stepper_action) {
                    case LS_STEPPER_ACTION_RANDOM_SPIN: // LS_STEPPER_ROTATION_MODE_RANDOM_SPIN
                    _ls_enqueue_random_spin_target_rpm();
                    break;
                    case LS_STEPPER_ACTION_TARGET_RPM: // LS_STEPPER_ROTATION_MODE_RANDOM_SPIN
#ifdef LSDEBUG_STEPPER_RANDOM
    if(ls_stepper_target_timer_alarm_count!=_timer_alarm_count_from_rpm(message.value))
    {
        ls_debug_printf("STEPPER_RANDOM: LS_STEPPER_ROTATION_MODE_RANDOM_SPIN received LS_STEPPER_ACTION_TARGET_RPM; new target of %d RPM [%d clocks].\n", 
            message.value, (int) _timer_alarm_count_from_rpm(message.value));
    }
#endif   
                    ls_stepper_target_timer_alarm_count = _timer_alarm_count_from_rpm(message.value);
                    if (ls_stepper_current_timer_alarm_count == ls_stepper_target_timer_alarm_count){
                       _ls_enqueue_random_spin_idle_ticks();
                    }
                    break;
                    case LS_STEPPER_ACTION_IDLE: // LS_STEPPER_ROTATION_MODE_RANDOM_SPIN
                    if (0>=message.value) { // we've been idle enough ticks
                        _ls_stepper_enqueue_finished_move();
                        _ls_enqueue_random_spin_target_rpm();
                    } else {
                        message.value--;
#ifdef LSDEBUG_STEPPER_RANDOM
    if(0==message.value % 100)
    {
        ls_debug_printf("STEPPER_RANDOM: spin reached target RPM; still holding for %d ticks.\n", message.value);
    }
#endif
                    }
                    break;
                    default:  // LS_STEPPER_ROTATION_MODE_RANDOM_SPIN
                    ; 
                } // switch _current_stepper_action for LS_STEPPER_ROTATION_MODE_RANDOM_SPIN
                successor_stepper_rotation_mode = _do_state_spinning(_current_stepper_rotation_mode, &message);
#ifdef LSDEBUG_STEPPER
if(successor_stepper_rotation_mode != _current_stepper_rotation_mode)
{
    ls_debug_printf("_do_state_spinning changing mode from %d to %d\n", _current_stepper_rotation_mode, successor_stepper_rotation_mode);
}
#endif
            break;
            case LS_STEPPER_ROTATION_MODE_HOPPING: // 4
                gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_ENABLE);
                ls_stepper_mode_hop0_spin1 = 0;
                successor_stepper_rotation_mode = _do_state_hopping(_current_stepper_rotation_mode, &message);
            break;
            case LS_STEPPER_ROTATION_MODE_SPINNING: // 5
                gpio_set_level(LSGPIO_STEPPERENABLE, STEPPERENABLE_ENABLE);
                ls_stepper_mode_hop0_spin1 = 1;
                successor_stepper_rotation_mode = _do_state_spinning(_current_stepper_rotation_mode, &message);
            break;
        } // switch (current_stepper_state)
#ifdef LSDEBUG_STEPPER_RANDOM
    if(_current_stepper_rotation_mode != successor_stepper_rotation_mode)
    {
        ls_debug_printf("STEPPER: mode changing from %d to %d.\n", _current_stepper_rotation_mode, successor_stepper_rotation_mode);
    }
#endif
        _current_stepper_rotation_mode = successor_stepper_rotation_mode;
        vTaskDelay(1);
    } // while 1 forever...
} // stepper task


void ls_stepper_set_random_hop_strategy(StepperMoveStrategy strategy)
{
    _ls_stepper_random_strategy = strategy;
}

void ls_stepper_stop_hopping(void)
{
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: ls_stepper_stop_hopping() called\n");
#endif
#ifdef LS_HAS_TAPE_SENSOR
    _ls_stepper_enable_skipping = false;
#endif
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_STOP;
    message.value = 0;
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
}

void ls_stepper_stop_spin(void)
{
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: ls_stepper_stop_spin() called\n");
#endif
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_STOP;
    message.value = 0;
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
}

void ls_stepper_forward_hop(int32_t steps)
{
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: ls_stepper_forward_hop(%d) called\n", steps);
#endif
#ifdef LS_HAS_TAPE_SENSOR
    _ls_stepper_enable_skipping = false;
#endif
    if (steps > 0)
    {
        ls_stepper_action_message message;
        message.action = LS_STEPPER_ACTION_HOP_FORWARD_STEPS;
        message.value = steps;
        xQueueSend(ls_stepper_queue, (void *)&message, 0);
    }
    else
    {
        ls_stepper_stop_hopping();
    }
}

void ls_stepper_reverse_hop(int32_t steps)
{
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: ls_stepper_reverse_hop(%d) called\n", steps);
#endif
#ifdef LS_HAS_TAPE_SENSOR
    _ls_stepper_enable_skipping = false;
#endif
    if (steps > 0)
    {
        ls_stepper_action_message message;
        message.action = LS_STEPPER_ACTION_HOP_REVERSE_STEPS;
        message.value = steps;
        xQueueSend(ls_stepper_queue, (void *)&message, 0);
    }
    else
    {
        ls_stepper_stop_hopping();
    }
}

void ls_stepper_random_hop(void)
{
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: ls_stepper_random_hop() called\n");
#endif
#ifdef LS_HAS_TAPE_SENSOR
    _ls_stepper_enable_skipping = LS_MAP_STATUS_OK == ls_map_get_status();
    if (_ls_stepper_enable_skipping)
    {
        _ls_stepper_speed_not_skipping = _ls_stepper_steps_per_second_max;
    }
#endif
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_RANDOM_HOP;
    message.value = 0;
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
}

void ls_stepper_random_spin(void)
{
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: ls_stepper_random_spin() called\n");
#endif
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_RANDOM_SPIN;
    message.value = 0;
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
}

void ls_stepper_spin_at_rpm(int32_t rpm)
{
    // the current stepper event handler can't distinguish between 
    // LS_STEPPER_ACTION_TARGET_RPM sent here vs. one from
    // a new random spin speed, so we'll stop first if not already stopped/spinning
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: ls_stepper_spin_at_rpm(%d) called\n", rpm);
#endif
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_SPIN;
    message.value = rpm;
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
}

void ls_stepper_sleep(void)
{
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: ls_stepper_sleep() called\n");
#endif
    ls_stepper_action_message message;
    message.action = LS_STEPPER_ACTION_SLEEP;
    message.value = 0;
    xQueueSend(ls_stepper_queue, (void *)&message, 0);
}

ls_stepper_position_t IRAM_ATTR ls_stepper_get_position(void)
{
    return ls_stepper_position;
}


enum ls_stepper_direction_t IRAM_ATTR ls_stepper_get_direction()
{
    return ls_stepper_direction;
}
void IRAM_ATTR ls_stepper_set_home_position(void)
{
    ls_stepper_position = 0;
}

void IRAM_ATTR ls_stepper_set_home_offset(int offset)
{
    ls_stepper_position -= offset;
    while(ls_stepper_position < 0)
    {
        ls_stepper_position += LS_STEPPER_STEPS_PER_ROTATION;
    }
}

bool ls_stepper_is_stopped(void)
{
    bool stopped = 0 == ls_stepper_mode_hop0_spin1 ? 
        0 == ls_stepper_steps_remaining :
        ls_stepper_current_timer_alarm_count >= ls_stepper_timer_alarm_count_stoppable;
#ifdef LSDEBUG_STEPPER
ls_debug_printf("STEPPER: ls_stepper_is_stopped() => ");
ls_debug_printf(stopped ? "true\n" : "false\n");
#endif
    return stopped;
}

void ls_stepper_stop(void)
{
    if (0==ls_stepper_mode_hop0_spin1)
    {
        ls_stepper_stop_hopping();
    }
    else {
        ls_stepper_stop_spin();
    }
}

BaseType_t ls_stepper_get_steps_taken(void)
{
    return ls_stepper_steps_taken;
}

void ls_stepper_set_random_reverse_per255(uint8_t value)
{
    _ls_stepper_random_reverse_per255 = value;
}

#ifdef LSDEBUG_STEPPER
void ls_stepper_debug_task(void *pvParameter)
{
    while (1)
    {
        ls_debug_printf("STEPPER DEBUG: _current_stepper_rotation_mode = %d; _current_stepper_action = %d.\n",
             _current_stepper_rotation_mode, _current_stepper_action);
        if(0==ls_stepper_mode_hop0_spin1) {
        ls_debug_printf("   (hopping) position=%d; rate=%d/%d; remaining=%d; taken=%d; direction=%d, step_phase=%d\n",
                        ls_stepper_position, _ls_stepper_speed_current_hop_rate, _ls_stepper_steps_per_second_max, 
                        ls_stepper_steps_remaining, ls_stepper_steps_taken, 
                        ls_stepper_direction, _ls_stepperstep_phase);
        }
        if(1==ls_stepper_mode_hop0_spin1) {
        ls_debug_printf("   (spinning) current RPM=%d [%llu]; target RPM=%d [%llu]; direction=%d, step_phase=%d\n",
                        _rpm_from_timer_alarm_value(ls_stepper_current_timer_alarm_count), ls_stepper_current_timer_alarm_count,
                        _rpm_from_timer_alarm_value(ls_stepper_target_timer_alarm_count), ls_stepper_target_timer_alarm_count,
                         ls_stepper_direction, _ls_stepperstep_phase);
        }
        vTaskDelay(pdMS_TO_TICKS(LSDEBUG_STEPPER_STATUS_INTERVAL_MS));
    }
}
#endif

// default stepper move strategy
void ls_stepper_random_strategy_default(struct ls_stepper_move_t *move)
{
    uint32_t random = esp_random();
    move->direction = ((uint8_t)random & 0xFF) > _ls_stepper_random_reverse_per255 ? false : true;
    move->steps = LS_STEPPER_RANDOM_HOP_STEPS_MIN + ((random >> 16) * (ls_settings_get_stepper_random_max() - LS_STEPPER_RANDOM_HOP_STEPS_MIN) / 65536);
#ifdef LSDEBUG_STEPPER_RANDOM
    ls_debug_printf("Default strategy: At end of random move; moving randomly %s%d\n", 
    move->direction?"+":"-", move->steps);
#endif
}
