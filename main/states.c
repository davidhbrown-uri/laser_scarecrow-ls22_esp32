/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2023 David H. Brown

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
#include "debug.h"
#include "states.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "stepper.h"
#include "servo.h"
#include "buzzer.h"
#include "magnet.h"
#include "tapemode.h"
#include "tape.h"
#include "map.h"
#include "init.h"
#include "substate_home.h"
#include "laser.h"
#include "settings.h"
#include "selftest.h"
#include "util.h"
#include "controls.h"
#include "coverage.h"
#include "leds.h"

//static const char* TAG = "LS States"; // for ESP logging                                                                                                                       

extern SemaphoreHandle_t print_mux;
extern QueueHandle_t ls_event_queue;

static TaskHandle_t ls_coverage_task_handle;


#define LEDCYCLE_HOMING LEDCYCLE_RAINBOW
#define LEDCYCLE_HOMING_FAIL LEDCYCLE_RED_FLASH

TimerHandle_t _ls_state_rehome_timer;

#define _ls_state_everything_off() \
    {                              \
        ls_servo_off();            \
        ls_stepper_off();          \
        ls_laser_set_mode_off();   \
        ls_tape_sensor_disable();  \
    }

void _ls_state_rehome_timer_callback(TimerHandle_t xTimer)
{
    ls_event event;
    event.type = LSEVT_REHOME_REQUIRED;
    event.value = NULL;
    if (xQueueSendToBack(ls_event_queue, (void *)&event, pdMS_TO_TICKS(5000)) != pdPASS)
    {
#ifdef LSDEBUG_STATES
        ls_debug_printf("WARNING: Could not enqueue LSEVT_REHOME_REQUIRED; will try to try again later\n");
#endif
        xTimerReset(_ls_state_rehome_timer, pdMS_TO_TICKS(5000));
    }
}

void ls_state_init(void)
{
    _ls_state_rehome_timer = xTimerCreate("rehome_timer",                                 // pcTimerName
                                          pdMS_TO_TICKS(LS_STATE_REHOME_TIMER_PERIOD_MS), // xTimerPeriodInTicks
                                          pdFALSE,                                        // uxAutoReload
                                          0,                                              // pvTimerId not used; only timer for callback
                                          _ls_state_rehome_timer_callback                 // pxCallbackFunction
    );
}

/**
 * @brief Checks the event queue and dispatches to current state function
 *
 * @link https://controllerstech.com/freertos-tutorial-5-using-queue/
 *
 * @param pvParameter
 */
void event_handler_state_machine(void *pvParameter)
{
    ls_State previous_state;
    previous_state.func = NULL;
    ls_event event;
    ls_event state_entry_event, noop_event;
    state_entry_event.type = LSEVT_STATE_ENTRY;
    state_entry_event.value = NULL;
    noop_event.type = LSEVT_NOOP;
    noop_event.value = NULL;

    // if the ls_state_current is set prior to beginning this task, start it up with an entry event
    if (ls_state_current.func != NULL)
    {
        xQueueSendToFront(ls_event_queue, (void *)&state_entry_event, 0);
    }

    while (1)
    {
        if (xQueueReceive(ls_event_queue, &event, pdMS_TO_TICKS(LS_EVENT_NOOP_TIMEOUT_MS)) != pdTRUE)
        {
            xQueueSendToFront(ls_event_queue, (void *)&noop_event, 0);
        }
        else
        {
#ifdef LSDEBUG_STATES
            ls_debug_printf("Received event %d\n", event.type);
#endif
            if (ls_state_current.func != NULL) // we have a current state
            {
                ls_state_current = ls_state_current.func(event);
                // send state entry events if the state changed
                if (ls_state_current.func != previous_state.func)
                {
#ifdef LSDEBUG_STATES
                    ls_debug_printf("Switching states; sending entry event\n");
#endif
                    xQueueSendToFront(ls_event_queue, (void *)&state_entry_event, 0);
                }
                previous_state.func = ls_state_current.func; // save current state as previous to next event
            }
            else
            {
#ifdef LSDEBUG_STATES
                ls_debug_printf("Event lost; no state function to handle it\n");
#endif
                ; // do nothing; no state
            }
        }
    } // while 1 -- task must not exit
}

bool ls_state_home_to_magnet_status = false;
int ls_magnet_homing_tries = 50;

ls_State ls_state_poweron(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_POWERON handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_prelaserwarn;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_tapemode_init();
        switch (ls_tapemode())
        {
        case LS_TAPEMODE_IGNORE:
            successor.func = ls_state_prelaserwarn;
#ifdef LSDEBUG_STATES
            ls_debug_printf("State poweron ignoring tape => pre-laser warning\n");
#endif
            break;
        case LS_TAPEMODE_SELFTEST:
#ifdef LSDEBUG_STATES
            ls_debug_printf("State poweron => selftest\n");
#endif
            successor.func = ls_state_selftest;
            break;
        default:
#ifdef LSDEBUG_STATES
            ls_debug_printf("During poweron, ls_map_get_status()=> %d\n", ls_map_get_status());
#endif
            if (ls_map_get_status() == LS_MAP_STATUS_OK)
            {
#ifdef LSDEBUG_STATES
                ls_debug_printf("State poweron (LS_MAP_STATUS_OK)=> pre-laser warning\n");
#endif
                successor.func = ls_state_prelaserwarn;
            }
            else
            {
#ifdef LSDEBUG_STATES
                ls_debug_printf("State poweron => map_build_substate_home\n");
#endif
                ls_state_set_home_successor(ls_state_map_build);
                successor.func = ls_state_home; // ls_state_map_build_substate_home;
            }

        } // switch tapemode
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default: // switch event.type
        ;
    }
    return successor;
}

static bool _ls_state_prelaserwarn_buzzer_complete;
static bool _ls_state_prelaserwarn_movement_complete;
static int _ls_state_prelaserwarn_rotation_count;
static void *_ls_state_prelaserwarn_successor = NULL;
void ls_state_set_prelaserwarn_successor(void *successor)
{
    _ls_state_prelaserwarn_successor = successor;
}
ls_State ls_state_prelaserwarn(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_PRELASERWARN handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_prelaserwarn;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        _ls_state_prelaserwarn_buzzer_complete = false;
        _ls_state_prelaserwarn_movement_complete = false;
        _ls_state_prelaserwarn_rotation_count = 0;
        ls_leds_cycle(LEDCYCLE_WARNING);

        while (ls_buzzer_in_use() || ls_stepper_is_moving())
        {
            vTaskDelay(1);
        }
        ls_event_empty_queue();
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1sec quiet/still before warning
        ls_buzzer_effect(LS_BUZZER_PRE_LASER_WARNING);
        ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_WARNING);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION / 2);
        break;
    case LSEVT_BUZZER_WARNING_COMPLETE:
        _ls_state_prelaserwarn_buzzer_complete = true;
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        _ls_state_prelaserwarn_rotation_count++;
        if (3 > _ls_state_prelaserwarn_rotation_count)
        {
            ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION / 2);
        }
        if (3 == _ls_state_prelaserwarn_rotation_count)
        {
            ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 3 / 2);
        }
        if (3 < _ls_state_prelaserwarn_rotation_count)
        {
            _ls_state_prelaserwarn_movement_complete = true;
        }
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:; // does not handle other events
    }
    if (_ls_state_prelaserwarn_buzzer_complete && _ls_state_prelaserwarn_movement_complete)
    {
        ls_leds_off();
        if (NULL == _ls_state_prelaserwarn_successor)
        {
            _ls_state_prelaserwarn_successor = ls_state_active; // default
        }
        successor.func = _ls_state_prelaserwarn_successor;
        _ls_state_prelaserwarn_successor = NULL;
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1sec quiet/still after warning
    }
    return successor;
}

ls_State ls_state_active(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_ACTIVE handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_active;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Beginning active state\n");
#endif
        ls_stepper_set_maximum_steps_per_second(ls_settings_get_stepper_speed());
        ls_stepper_random();
        ls_servo_random();
        ls_leds_off();
        ls_coverage_task_handle = NULL;

        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_laser_set_mode_mapped();
            xTimerReset(_ls_state_rehome_timer, pdMS_TO_TICKS(5000));
            xTaskCreate(&ls_coverage_task, "coverage_task", configMINIMAL_STACK_SIZE * 3, NULL, 2, &ls_coverage_task_handle); // cannot do before map is ready
        }
        else
        {
            ls_laser_set_mode_on();
        }
        break;
    case LSEVT_MAGNET_ENTER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Magnet Enter @ %d %s\n", (int32_t)event.value, ls_stepper_get_direction() ? "-->" : "<--");
#endif
        ls_buzzer_effect(LS_BUZZER_CLICK);
        break;
    case LSEVT_MAGNET_LEAVE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Magnet Leave @ %d %s\n", (int32_t)event.value, ls_stepper_get_direction() ? "-->" : "<--");
#endif
        ls_buzzer_effect(LS_BUZZER_CLICK);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Stepper finished %s move @%d \n", ls_stepper_get_direction() ? "-->" : "<--", ls_stepper_get_position());
#endif
        break;
    case LSEVT_SERVO_SWEEP_TOP:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Servo finished move (reached top)\n");
#endif
        break;
    case LSEVT_SERVO_SWEEP_BOTTOM:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Servo finished move (reached bottom)\n");
#endif
        break;
    case LSEVT_REHOME_REQUIRED:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Rehoming from active state\n");
#endif
        successor.func = ls_state_home;
        break;
    case LSEVT_LIGHT_NIGHT:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering night/sleep state\n")
#endif
            successor.func = ls_state_sleep;
        break;
    case LSEVT_CONTROLS_UPPER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering settings (upper) control\n")
#endif
            successor.func = ls_state_settings_upper;
        break;
    case LSEVT_CONTROLS_LOWER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering settings (lower) control\n")
#endif
            successor.func = ls_state_settings_lower;
        break;
    case LSEVT_CONTROLS_BOTH:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering settings (both) control\n")
#endif
            successor.func = ls_state_settings_both;
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:;
#ifdef LSDEBUG_STATES
        ls_debug_printf("Unknown event %d\n", event.type);
#endif
    }
    // /exit behaviors:
    if (successor.func != ls_state_active)
    {
        if (NULL != ls_coverage_task_handle)
        {
            vTaskDelete(ls_coverage_task_handle);
            ls_coverage_task_handle = NULL; // probably not necessary now, but just in case
        }
        ls_stepper_stop();
        ls_servo_off();
        ls_laser_set_mode_off();
    }
    return successor;
}

static void *_ls_state_home_successor = NULL;
void ls_state_set_home_successor(void *successor)
{
    _ls_state_home_successor = successor;
}
ls_State ls_state_home(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_HOME handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_home;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_leds_cycle(LEDCYCLE_HOMING);
        ls_substate_home_init();
        ls_event_enqueue_noop();
        break;
    case LSEVT_HOME_COMPLETED:
        if (NULL == _ls_state_home_successor)
        {
            _ls_state_home_successor = ls_state_active;
        }
        successor.func = _ls_state_home_successor;
        _ls_state_home_successor = ls_state_active;
        ls_event_enqueue_noop();
        break;
    case LSEVT_HOME_FAILED:
        ls_buzzer_effect(LS_BUZZER_PLAY_HOME_FAIL);
        switch (ls_tapemode())
        {
        case LS_TAPEMODE_BLACK_SAFE:
        case LS_TAPEMODE_REFLECT_SAFE:
            successor.func = ls_state_error_home;
            break;
        default:
            if (NULL == _ls_state_home_successor)
            {
                _ls_state_home_successor = ls_state_active;
            }
            ls_state_set_prelaserwarn_successor(ls_state_active);
            successor.func = ls_state_prelaserwarn;
        }
        ls_event_enqueue_noop();
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:
        ls_substate_home_handle_event(event);
    }
    if(successor.func != ls_state_home)
    {
        ls_leds_off();
    }
    return successor;
}

ls_State ls_state_selftest(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_SELFTEST handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_selftest;
    selftest_event_handler(event);
    return successor;
}

ls_State ls_state_settings_upper(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_SETTINGS_UPPER (speed) handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_settings_upper;
    BaseType_t control_value;

    // somewhat awkward patch for #44 https://github.com/davidhbrown-uri/laser_scarecrow-ls22_esp32/issues/44
    if (LS_CONTROLS_STATUS_OFF == ls_controls_get_current_status())
    {
        event.type = LSEVT_CONTROLS_OFF;
    }

    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_stepper_set_maximum_steps_per_second(ls_settings_get_stepper_speed());
        ls_laser_set_mode((ls_map_get_status() == LS_MAP_STATUS_OK) ? LS_LASER_MAPPED : LS_LASER_ON);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        ls_servo_sweep();
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        ls_leds_cycle(LEDCYCLE_CONTROLS_UPPER);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        break;
    case LSEVT_SERVO_SWEEP_TOP:
        ls_buzzer_effect(LS_BUZZER_PLAY_OCTAVE);
        break;
    case LSEVT_SERVO_SWEEP_BOTTOM:
        ls_buzzer_effect(LS_BUZZER_PLAY_ROOT);
        break;
    case LSEVT_CONTROLS_SLIDER1: // stepper speed
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_stepper_speed(ls_settings_map_control_to_stepper_speed(control_value));
        ls_stepper_set_maximum_steps_per_second(ls_settings_get_stepper_speed());
        break;
    case LSEVT_CONTROLS_SLIDER2: // servo speed
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_servo_pulse_delta(ls_settings_map_control_to_servo_pulse_delta(control_value));
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Setting servo pulse delta to %d microseconds per tick.\n", ls_settings_map_control_to_servo_pulse_delta(control_value));
#endif
        break;
    case LSEVT_CONTROLS_OFF:
        ls_stepper_stop();
        successor.func = ls_state_active;
        break;
    case LSEVT_CONTROLS_LOWER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering lower from upper settings control\n")
#endif
            successor.func = ls_state_settings_lower;
        break;
    case LSEVT_CONTROLS_BOTH:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering both from upper settings control\n")
#endif
            successor.func = ls_state_settings_both;
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:;
    } // switch event type

    if (ls_state_settings_upper != successor.func)
    {
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE);
        ls_settings_save();
    }
    return successor;
}

static int _ls_state_settings_servo_hold_count = 0;
ls_State ls_state_settings_lower(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_SETTINGS_LOWER (servo top/range) handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_settings_lower;
    BaseType_t control_value;

    // somewhat awkward patch for #44 https://github.com/davidhbrown-uri/laser_scarecrow-ls22_esp32/issues/44
    if (LS_CONTROLS_STATUS_OFF == ls_controls_get_current_status())
    {
        event.type = LSEVT_CONTROLS_OFF;
    }

    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_stepper_set_maximum_steps_per_second(ls_settings_get_stepper_speed());
        ls_laser_set_mode((ls_map_get_status() == LS_MAP_STATUS_OK) ? LS_LASER_MAPPED : LS_LASER_ON);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        ls_servo_sweep();
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        ls_leds_cycle(LEDCYCLE_CONTROLS_LOWER);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        if (_ls_state_settings_servo_hold_count > 0)
        {
            _ls_state_settings_servo_hold_count--;
        }
        else if (_ls_state_settings_servo_hold_count == 0)
        {
            ls_servo_sweep();
            _ls_state_settings_servo_hold_count = -1;
        }
        break;
    case LSEVT_SERVO_SWEEP_TOP:
        ls_buzzer_effect(LS_BUZZER_PLAY_OCTAVE);
        break;
    case LSEVT_SERVO_SWEEP_BOTTOM:
        ls_buzzer_effect(LS_BUZZER_PLAY_ROOT);
        break;
    case LSEVT_CONTROLS_SLIDER1:
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_servo_top(ls_settings_map_control_to_servo_top(control_value));
        ls_servo_jumpto(ls_settings_get_servo_top());
        _ls_state_settings_servo_hold_count = 3;
        break;
    case LSEVT_CONTROLS_SLIDER2:
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_servo_bottom(ls_settings_map_control_to_servo_bottom(control_value));
        ls_servo_jumpto(ls_settings_get_servo_bottom());
        _ls_state_settings_servo_hold_count = 3;
        break;
    case LSEVT_CONTROLS_OFF:
        ls_stepper_stop();
        successor.func = ls_state_active;
        break;
    case LSEVT_CONTROLS_UPPER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering upper from lower settings control\n")
#endif
            successor.func = ls_state_settings_upper;
        break;
    case LSEVT_CONTROLS_BOTH:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering both from lower settings control\n")
#endif
            successor.func = ls_state_settings_both;
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:;
    } // switch event type

    if (ls_state_settings_lower != successor.func)
    {
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE);
        ls_settings_save();
    }
    return successor;
}

static enum ls_buzzer_scale _ls_state_settings_lightsense_scale[] = {
    // 0..4 => switch at lower light levels
    LS_BUZZER_SCALE_D, LS_BUZZER_SCALE_Dx, LS_BUZZER_SCALE_E, LS_BUZZER_SCALE_F, LS_BUZZER_SCALE_Fx,
    LS_BUZZER_SCALE_G, // 5th value; center of range
    // 6..10 => switch at higher levels
    LS_BUZZER_SCALE_Gx, LS_BUZZER_SCALE_A, LS_BUZZER_SCALE_Ax, LS_BUZZER_SCALE_B, LS_BUZZER_SCALE_CC};
ls_State ls_state_settings_both(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("CONTROLS_SETTINGS_BOTH handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_settings_both;
    BaseType_t control_value;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_laser_set_mode_off();
        ls_stepper_stop();
        ls_stepper_sleep(); // why? loses homing
        ls_servo_off();
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        ls_leds_cycle(LEDCYCLE_CONTROLS_BOTH);
        break;

    case LSEVT_CONTROLS_SLIDER1: // light threshold
        control_value = *((BaseType_t *)event.value);
        // map input to 0-10 setting range
        int index = _map(_constrain(control_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP),
                         LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 0, 10);
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Setting lightsense thresholds to index %d.\n", index);
#endif
        // calculate/set threshold values
        // ls_settings_set_light_thresholds_from_0to10(index);
        // if event queue is empty, play tune to indicate threshold value
        ls_buzzer_note(_ls_state_settings_lightsense_scale[5], 3);
        ls_buzzer_note(_ls_state_settings_lightsense_scale[index], 3);
        ls_settings_set_light_thresholds_from_0to10(index);
        break;
    case LSEVT_CONTROLS_SLIDER2:; // nothing assigned
        break;
    case LSEVT_CONTROLS_OFF:
        successor.func = ls_state_wakeup; // because we turned laser off, must do wakeup and pre-laser warning
        break;
    case LSEVT_CONTROLS_UPPER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering upper from both settings control\n")
#endif
            ls_state_set_prelaserwarn_successor(ls_state_settings_upper);
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_state_set_home_successor(ls_state_prelaserwarn);
            successor.func = ls_state_home;
        }
        else
        {
            successor.func = ls_state_prelaserwarn;
        }
        break;
    case LSEVT_CONTROLS_LOWER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering lower from both settings control\n")
#endif
            ls_state_set_prelaserwarn_successor(ls_state_settings_lower);
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_state_set_home_successor(ls_state_prelaserwarn);
            successor.func = ls_state_home;
        }
        else
        {
            successor.func = ls_state_prelaserwarn;
        }
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    case LSEVT_NOOP:
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
    default:; // do nothing for this event
    }         // switch event type
    if (ls_state_settings_both != successor.func)
    {
        ls_substate_home_require_rehome(); 
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE);
        ls_settings_save();
    }
    return successor;
}

ls_State ls_state_sleep(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_SLEEP handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_sleep;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_laser_set_mode_off();
        ls_servo_off();
        ls_stepper_forward(1);
        ls_leds_off();
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        // the magnet sensor includes an LED which could pointlessly drain power during sleep
        if (ls_magnet_is_detected())
        {
            ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION / 4);
        }
        else
        {
            ls_stepper_sleep();
            ls_event_enqueue_noop();
        }
        break;
    case LSEVT_LIGHT_DAY:
#ifdef LSDEBUG_STATES
        ls_debug_printf("SLEEP received wake-up event\n");
#endif
        successor.func = ls_state_wakeup;
        break;
    case LSEVT_NOOP: // snore (?)
        ls_leds_cycle(LEDCYCLE_SLEEP);

        for (int i = 0; i < 20; i++)
        {
            ls_buzzer_effect(LS_BUZZER_CLICK);
            vTaskDelay(5 - i / 5);
        }
        for (int i = 0; i < 40; i++)
        {
            ls_buzzer_effect(LS_BUZZER_CLICK);
            vTaskDelay(1 + i / 3);
        }
        ls_leds_off();
        for (int i = 0; i < 40; i++)
        {
            if (ls_event_queue_has_messages())
            {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        ls_event_enqueue_noop_if_queue_empty();
        break;
    case LSEVT_CONTROLS_UPPER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering settings upper from sleep\n");
#endif
        ls_state_set_prelaserwarn_successor(ls_state_settings_upper);
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_state_set_home_successor(ls_state_prelaserwarn);
            successor.func = ls_state_home;
        }
        else
        {
            successor.func = ls_state_prelaserwarn;
        }
        break;
    case LSEVT_CONTROLS_LOWER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering settings lower from sleep\n");
#endif
        ls_state_set_prelaserwarn_successor(ls_state_settings_lower);
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_state_set_home_successor(ls_state_prelaserwarn);
            successor.func = ls_state_home;
        }
        else
        {
            successor.func = ls_state_prelaserwarn;
        }
        break;
    case LSEVT_CONTROLS_BOTH:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering secondary settings control from sleep\n")
#endif
            successor.func = ls_state_settings_both;
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:;
    }
    return successor;
}

ls_State ls_state_wakeup(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_WAKEUP handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_wakeup;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_substate_home_require_rehome();
            ls_substate_home_init();
            ls_event_enqueue_noop();
        }
        else
        {
            successor.func = ls_state_prelaserwarn;
        }
        break;
    case LSEVT_HOME_COMPLETED:
        successor.func = ls_state_prelaserwarn;
        ls_event_enqueue_noop();
        break;
    case LSEVT_HOME_FAILED:
        successor.func = ls_state_error_home;
        ls_event_enqueue_noop();
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:
        ls_substate_home_handle_event(event);
    }
    return successor;
}

static int _ls_state_map_build_steps_remaining;
static int _ls_state_map_enable_count = 0, _ls_state_map_disable_count = 0, _ls_state_map_misread_count = 0;

ls_State ls_state_map_build(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_MAP_BUILD handling event %d with %d steps remaining\n", event.type, _ls_state_map_build_steps_remaining);
#endif
    ls_State successor;
    successor.func = ls_state_map_build;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        _ls_state_map_build_steps_remaining = LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION;
        ls_tape_sensor_enable();
        while (ls_buzzer_in_use() || ls_stepper_is_moving())
        {
            // if we start moving while the buzzer is still indicating successfull homing,
            // we get a bunch of pitches queued that play faster than the others
            vTaskDelay(1);
        }
        ls_event_empty_queue(); // in case of a trailing LSEVT_STEPPER_FINISHED_MOVE
        ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_MAPPING);
        ls_stepper_forward(LS_MAP_RESOLUTION);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("case LSEVT_STEPPER_FINISHED_MOVE...\n");
#endif
        if (_ls_state_map_build_steps_remaining > 0)
        {
            _ls_state_map_build_read_and_set_map(&_ls_state_map_enable_count, &_ls_state_map_disable_count, &_ls_state_map_misread_count);
#ifdef LSDEBUG_STATES
            ls_debug_printf("continuing to next position...\n");
#endif
            ls_stepper_forward(LS_MAP_RESOLUTION);
            _ls_state_map_build_steps_remaining--;
        }
        else
        { // we're done building the map
            bool badmap = false;
            int low_peak_bin, high_peak_bin, low_edge_bin, high_edge_bin;
            _ls_state_map_build_histogram(ls_map_min_adc(), ls_map_max_adc());
            _ls_state_map_build_histogram_get_peaks_edges(&low_peak_bin, &low_edge_bin, &high_peak_bin, &high_edge_bin);
            if (low_edge_bin >= high_edge_bin) // mapping failed; insufficient contrast (probably no tape)
            {
                badmap = true;
            }
            else // reset the map based on customized histogram
            {
                // rebuild the histogram just between the peaks
                uint16_t low_peak_adc = _map(low_peak_bin, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1, ls_map_min_adc(), ls_map_max_adc());
                uint16_t high_peak_adc = _map(high_peak_bin, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1, ls_map_min_adc(), ls_map_max_adc());
                _ls_state_map_build_histogram(low_peak_adc, high_peak_adc);
                _ls_state_map_build_histogram_get_peaks_edges(&low_peak_bin, &low_edge_bin, &high_peak_bin, &high_edge_bin);
                // remap to custom thresholds
                _ls_state_map_build_set_map(&_ls_state_map_enable_count, &_ls_state_map_disable_count, &_ls_state_map_misread_count,
                                            _map(low_edge_bin, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1, low_peak_adc, high_peak_adc), // low threshold
                                            _map(high_edge_bin, 0, LS_MAP_HISTOGRAM_BINCOUNT - 1, low_peak_adc, high_peak_adc) // high threshold
                );
            }
#ifdef LSDEBUG_MAP
            ls_debug_printf("\nMapping completed with %d enabled, %d disabled, and %d misreads\n", _ls_state_map_enable_count, _ls_state_map_disable_count, _ls_state_map_misread_count);
#endif
            successor.func = ls_state_prelaserwarn;
            if (0 == _ls_state_map_enable_count || 0 == _ls_state_map_disable_count)
            {
                badmap = true;
#ifdef LSDEBUG_MAP
                ls_debug_printf("\nBad map: must include at least one enabled and one disabled reading.\n");
#endif
            }
            if (ls_map_is_excessive_misreads(_ls_state_map_misread_count))
            {
                badmap = true;
#ifdef LSDEBUG_MAP
                ls_debug_printf("\nBad map: too many misreads\n");
#endif
            }
            if (badmap)
            {
                ls_map_set_status(LS_MAP_STATUS_FAILED);
                ls_buzzer_effect(LS_BUZZER_PLAY_NOTHING);
                ls_buzzer_effect(LS_BUZZER_PLAY_MAP_FAIL);
                switch (ls_tapemode())
                {
                case LS_TAPEMODE_BLACK_SAFE:
                case LS_TAPEMODE_REFLECT_SAFE:
                    successor.func = ls_state_error_map;
                    break;
                default:
                    if (0 == _ls_state_map_enable_count)
                    {
                        ls_map_set_status(LS_MAP_STATUS_IGNORE);
                    }
                    break;
                }
            } // handle bad map
            else
            {
#ifdef LSDEBUG_MAP
                int32_t total = ls_map_find_spans();
                ls_debug_printf("Set LS_MAP_STATUS_OK; random map span strategy has %d steps in total.\n", total);
#else
                ls_map_find_spans();
#endif
                ls_stepper_set_random_strategy(ls_stepper_random_strategy_map_spans);
                ls_map_set_status(LS_MAP_STATUS_OK);
            }
            ls_tape_sensor_disable();
        } // done building map
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:; // nothing to do for event of this type
    }         // switch  on event
    return successor;
}

ls_State ls_state_error_home(ls_event event)
{
    _ls_state_everything_off();
    ls_State successor;
    ls_leds_cycle(LEDCYCLE_HOMING_FAIL);
    successor.func = ls_state_error_home;
#ifdef LSDEBUG_HOMING
    xSemaphoreTake(print_mux, portMAX_DELAY);
    ls_debug_printf(">>>>HOMING FAILED<<<\n");
    xSemaphoreGive(print_mux);
#endif
    vTaskDelay(pdMS_TO_TICKS(30000)); // 30 seconds between alerts
    ls_buzzer_effect(LS_BUZZER_PLAY_HOME_FAIL);
    ls_event_enqueue_noop();
    return successor;
}

ls_State ls_state_error_map(ls_event event)
{
    _ls_state_everything_off();
    ls_State successor;
    successor.func = ls_state_error_map;
#ifdef LSDEBUG_MAP
    xSemaphoreTake(print_mux, portMAX_DELAY);
    ls_debug_printf(">>>>MAPPING FAILED<<<\n");
    xSemaphoreGive(print_mux);
#endif
    vTaskDelay(pdMS_TO_TICKS(30000)); // 30 seconds between alerts
    if (0 == _ls_state_map_enable_count)
    {
        ls_buzzer_effect(LS_BUZZER_PLAY_TAPE_DISABLE);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second between tones
    }
    if (0 == _ls_state_map_disable_count)
    {
        ls_buzzer_effect(LS_BUZZER_PLAY_TAPE_ENABLE);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second between tones
    }
    if (ls_map_is_excessive_misreads(_ls_state_map_misread_count))
    {
        ls_buzzer_effect(LS_BUZZER_PLAY_TAPE_MISREAD);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second between tones
    }
    ls_buzzer_effect(LS_BUZZER_PLAY_MAP_FAIL);
    ls_event_enqueue_noop();
    return successor;
}

ls_State ls_state_error_tilt(ls_event event)
{
#ifdef LSDEBUG_TILT
    xSemaphoreTake(print_mux, portMAX_DELAY);
    ls_debug_printf("ERROR_TILT\n");
    xSemaphoreGive(print_mux);
#endif
    ls_State successor;
    successor.func = ls_state_error_tilt;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        _ls_state_everything_off();
        ls_event_enqueue_noop();
        break;
    case LSEVT_TILT_OK:
        switch (ls_tapemode())
        {
        case LS_TAPEMODE_BLACK_SAFE:
        case LS_TAPEMODE_REFLECT_SAFE:
#ifdef LSDEBUG_TILT
            xSemaphoreTake(print_mux, portMAX_DELAY);
            ls_debug_printf("TILT_OK but safe mode requires power cycle to resume\n");
            xSemaphoreGive(print_mux);
#endif
            ls_buzzer_effect(LS_BUZZER_PLAY_TILT_FAIL);
            ls_buzzer_effect(LS_BUZZER_ALERT_1S);
            ls_event_enqueue_noop();
            break;
        default:
            successor.func = ls_state_poweron;
        } // switch on tapemode if TILT_OK
        break;
    default:
        ls_buzzer_effect(LS_BUZZER_PLAY_TILT_FAIL);
        // 30 seconds between alerts unless an event is pending
        if (pdFALSE == xQueuePeek(ls_event_queue, &event, pdMS_TO_TICKS(30000)))
        {
            ls_event_enqueue_noop();
        }
    }
    return successor;
}

/**
 * @brief If no accelerometer is found, play a warning tone then attempt to restart MCU
 *
 * @param event
 * @return ls_State
 */
ls_State ls_state_error_noaccel(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_error_noaccel;
    // we're never really going to return
    ls_buzzer_effect(LS_BUZZER_PLAY_TILT_FAIL);
    vTaskDelay(pdMS_TO_TICKS(500));
    // if this is the first attempt just try to restart
    if (esp_reset_reason() != ESP_RST_SW)
    {
        esp_restart(); // software reset of the chip; starts execution again
    }
    // otherwise play the warning tone, too, and wait to try again
    ls_buzzer_effect(LS_BUZZER_ALTERNATE_HIGH);
    vTaskDelay(pdMS_TO_TICKS(29000));
    esp_restart(); // software reset of the chip; starts execution again
    // will not be reached
    return successor;
}