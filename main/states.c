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
#ifdef LS_HAS_TAPE_SENSOR
#include "substate_home.h"
#endif
#include "laser.h"
#include "settings.h"
#include "selftest.h"
#include "util.h"
#include "controls.h"
#include "coverage.h"
#include "leds.h"
#include "oled.h"
#include "failsafe.h"

// static const char* TAG = "LS States"; // for ESP logging

extern SemaphoreHandle_t print_mux;
extern QueueHandle_t ls_event_queue;

static TaskHandle_t ls_coverage_task_handle;

TimerHandle_t _ls_state_magnet_timeout_timer;



#ifdef LS_HAS_TAPE_SENSOR
#define LEDCYCLE_HOMING LEDCYCLE_RAINBOW

TimerHandle_t _ls_state_rehome_timer;

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
#define _ls_state_everything_off() \
    {                              \
        ls_servo_off();            \
        ls_stepper_off();          \
        ls_laser_set_mode_off();   \
        ls_tape_sensor_disable();  \
        ls_leds_off();             \
        ls_oled_blank_screen();    \
    }
#else
#define _ls_state_everything_off() \
    {                              \
        ls_laser_set_mode_off();   \
        ls_servo_off();            \
        ls_stepper_off();          \
        ls_leds_off();             \
        ls_oled_blank_screen();    \
    }
#endif

void _ls_state_magnet_timeout_callback(TimerHandle_t xTimer)
{
    ls_laser_set_mode_off();
    ls_buzzer_effect(LS_BUZZER_GENERAL_WARNING);
    ls_event event;
    event.type = LSEVT_MAGNET_TIMEOUT;
    event.value = NULL;
    if (pdTRUE != xQueueSend(ls_event_queue, (void *)&event, pdMS_TO_TICKS(5000)))
    {
        _ls_state_everything_off();
#ifdef LSDEBUG_STATES
        ls_debug_printf("WARNING: Could not enqueue LSEVT_MAGNET_TIMEOUT; failing\n");
#endif
        event.type = LSEVT_MAGNET_FAILURE;
        while(pdTRUE !=  xQueueSendToFront(ls_event_queue, (void *)&event, pdMS_TO_TICKS(500)))
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void ls_state_init(void)
{
#ifdef LS_HAS_TAPE_SENSOR
    _ls_state_rehome_timer = xTimerCreate("rehome_timer",                                 // pcTimerName
                                          pdMS_TO_TICKS(LS_STATE_REHOME_TIMER_PERIOD_MS), // xTimerPeriodInTicks
                                          pdFALSE,                                        // uxAutoReload
                                          0,                                              // pvTimerId not used; only timer for callback
                                          _ls_state_rehome_timer_callback                 // pxCallbackFunction
    );
#endif
_ls_state_magnet_timeout_timer= xTimerCreate("magnet_timeout_timer",                                 // pcTimerName
                                          pdMS_TO_TICKS(LS_STATE_MAGNET_TIMEOUT_PERIOD_MS), // xTimerPeriodInTicks
                                          pdFALSE,                                        // uxAutoReload
                                          0,                                              // pvTimerId not used; only timer for callback
                                          _ls_state_magnet_timeout_callback                 // pxCallbackFunction
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
            ls_debug_printf("STATES: Received event %d\n", event.type);
#endif
            if (ls_state_current.func != NULL) // we have a current state
            {
                ls_state_current = ls_state_current.func(event);
                // send state entry events if the state changed
                if (ls_state_current.func != previous_state.func)
                {
#ifdef LSDEBUG_STATES
                    ls_debug_printf("STATES: Switching states; sending entry event\n");
#endif
                    xQueueSendToFront(ls_event_queue, (void *)&state_entry_event, 0);
                }
                previous_state.func = ls_state_current.func; // save current state as previous to next event
            }
            else
            {
#ifdef LSDEBUG_STATES
                ls_debug_printf("STATES: Event lost; no state function to handle it\n");
#endif
                ; // do nothing; no state
            }
        }
    } // while 1 -- task must not exit
}

#ifdef LS_HAS_TAPE_SENSOR
bool ls_state_home_to_magnet_status = false;
int ls_magnet_homing_tries = 50;
#endif

ls_State ls_state_poweron(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: STATE_POWERON handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_prelaserwarn;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_buzzer_effect(LS_BUZZER_POWERON);
        ls_tapemode_init();
        switch (ls_tapemode())
        {
        case LS_TAPEMODE_SELFTEST:
#ifdef LSDEBUG_STATES
            ls_debug_printf("STATES: State poweron => selftest\n");
#endif
            successor.func = ls_state_selftest;
            break;
#ifdef LS_HAS_TAPE_SENSOR
        case LS_TAPEMODE_DARK:
        case LS_TAPEMODE_DARK_SAFE:
        case LS_TAPEMODE_LIGHT:
        case LS_TAPEMODE_LIGHT_SAFE:
#ifdef LSDEBUG_STATES
            ls_debug_printf("STATES: During poweron, ls_map_get_status()=> %d\n", ls_map_get_status());
#endif
            if (ls_map_get_status() == LS_MAP_STATUS_OK)
            {
#ifdef LSDEBUG_STATES
                ls_debug_printf("STATES: State poweron (LS_MAP_STATUS_OK)=> pre-laser warning\n");
#endif
                successor.func = ls_state_prelaserwarn;
            }
            else
            {
#ifdef LSDEBUG_STATES
                ls_debug_printf("STATES: State poweron => map_build_substate_home\n");
#endif
                ls_state_set_home_successor(ls_state_map_build);
                successor.func = ls_state_home; // ls_state_map_build_substate_home;
            }
#endif                
        default:
            successor.func = ls_state_prelaserwarn;
#ifdef LSDEBUG_STATES
            ls_debug_printf("STATES: State poweron ignoring tape => pre-laser warning\n");
#endif
        } // switch tapemode
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    case LSEVT_FAILSAFE_HEARTBEAT_MISSED:
        successor.func = ls_state_error_heartbeat;
        break;
    default: // switch event.type
        ;
    }
    return successor;
}

static bool _ls_state_prelaserwarn_buzzer_complete;
static bool _ls_state_prelaserwarn_movement_complete;
static int _ls_state_prelaserwarn_rotation_count;
static bool _ls_state_prelaserwarn_magnet_enter, _ls_state_prelaserwarn_magnet_leave;
static void *_ls_state_prelaserwarn_successor = NULL;
void ls_state_set_prelaserwarn_successor(void *successor)
{
    _ls_state_prelaserwarn_successor = successor;
}
ls_State ls_state_prelaserwarn(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: STATE_PRELASERWARN handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_prelaserwarn;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        _ls_state_prelaserwarn_buzzer_complete = false;
        _ls_state_prelaserwarn_movement_complete = false;
        _ls_state_prelaserwarn_magnet_enter = false;
        _ls_state_prelaserwarn_magnet_leave = false;
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
        ls_stepper_forward_hop(LS_STEPPER_STEPS_PER_ROTATION * 3 / 4);
        break;
    case LSEVT_BUZZER_WARNING_COMPLETE:
        _ls_state_prelaserwarn_buzzer_complete = true;
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        _ls_state_prelaserwarn_rotation_count++;
        if (3 > _ls_state_prelaserwarn_rotation_count)
        {
            ls_stepper_forward_hop(LS_STEPPER_STEPS_PER_ROTATION * 3 / 4);
        }
        if (3 == _ls_state_prelaserwarn_rotation_count)
        {
            ls_stepper_forward_hop(LS_STEPPER_STEPS_PER_ROTATION * 2);
        }
        if (3 < _ls_state_prelaserwarn_rotation_count)
        {
            _ls_state_prelaserwarn_movement_complete = true;
        }
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    case LSEVT_MAGNET_ENTER:
        _ls_state_prelaserwarn_magnet_enter = true;
        break;
    case LSEVT_MAGNET_LEAVE:
        _ls_state_prelaserwarn_magnet_leave = true;
        break;
    case LSEVT_FAILSAFE_HEARTBEAT_MISSED:
        successor.func = ls_state_error_heartbeat;
        break;
    default:; // does not handle other events
    }
    if (_ls_state_prelaserwarn_buzzer_complete && _ls_state_prelaserwarn_movement_complete)
    {
        ls_leds_off();
        if (NULL == _ls_state_prelaserwarn_successor)
        {
            ls_state_set_prelaserwarn_successor(ls_state_active); // default
        }
        if (!(_ls_state_prelaserwarn_magnet_enter && _ls_state_prelaserwarn_magnet_leave))
        {
            ls_state_set_prelaserwarn_successor(ls_state_error_norotate);
            successor.func = ls_state_error_norotate;
        }
        successor.func = _ls_state_prelaserwarn_successor;
        _ls_state_prelaserwarn_successor = NULL;
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1sec quiet/still after warning
    }
#ifdef LSDEBUG_STATES
ls_debug_printf("STATES: _ls_state_prelaserwarn_buzzer_complete=%d; _ls_state_prelaserwarn_movement_complete=%d\n", _ls_state_prelaserwarn_buzzer_complete, _ls_state_prelaserwarn_movement_complete);
#endif
    return successor;
}

ls_State ls_state_active(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: STATE_ACTIVE handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_active;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Beginning active state\n");
#endif
        ls_stepper_set_maximum_steps_per_second(ls_settings_get_stepper_speed());
        ls_stepper_random_spin();
        ls_servo_random();
        ls_leds_off();
        ls_oled_blank_screen();
        ls_coverage_task_handle = NULL;
        if (pdFAIL == xTimerStart(_ls_state_magnet_timeout_timer, pdMS_TO_TICKS(500))) 
        {
            successor.func = ls_state_error_norotate;
        }

#ifdef LS_HAS_TAPE_SENSOR
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
#else
        ls_laser_set_mode_on();
#endif
        break;
    case LSEVT_MAGNET_ENTER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Magnet Enter @ %d %s\n", (int32_t)event.value, ls_stepper_get_direction() ? "-->" : "<--");
#endif
        xTimerReset(_ls_state_magnet_timeout_timer, 0);
        ls_buzzer_effect(LS_BUZZER_CLICK);
        break;
    case LSEVT_MAGNET_LEAVE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Magnet Leave @ %d %s\n", (int32_t)event.value, ls_stepper_get_direction() ? "-->" : "<--");
#endif
        xTimerReset(_ls_state_magnet_timeout_timer, 0);
        ls_buzzer_effect(LS_BUZZER_CLICK);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Stepper finished %s move @%d \n", ls_stepper_get_direction() ? "-->" : "<--", ls_stepper_get_position());
#endif
        break;
    case LSEVT_SERVO_SWEEP_TOP:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Servo finished move (reached top)\n");
#endif
        break;
    case LSEVT_SERVO_SWEEP_BOTTOM:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Servo finished move (reached bottom)\n");
#endif
        break;
#ifdef LS_HAS_TAPE_SENSOR
    case LSEVT_REHOME_REQUIRED:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Rehoming from active state\n");
#endif
        successor.func = ls_state_home;
        break;
#endif
    case LSEVT_LIGHT_NIGHT:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Entering night/sleep state\n")
#endif
            successor.func = ls_state_sleep;
        break;
    case LSEVT_CONTROLS_UPPER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Entering settings (upper) control\n")
#endif
            successor.func = ls_state_settings_upper;
        break;
    case LSEVT_CONTROLS_LOWER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Entering settings (lower) control\n")
#endif
            successor.func = ls_state_settings_lower;
        break;
    case LSEVT_CONTROLS_BOTH:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Entering settings (both) control\n")
#endif
            successor.func = ls_state_settings_both;
        break;
    case LSEVT_MAGNET_TIMEOUT:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Magnet timeout - slowing to check rotation\n");
#endif
        successor.func = ls_state_rotation_check;
        break;
    case LSEVT_MAGNET_FAILURE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Magnet failure detected while active?\n");
#endif
        successor.func = ls_state_error_norotate;
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    case LSEVT_FAILSAFE_HEARTBEAT_MISSED:
        successor.func = ls_state_error_heartbeat;
        break;
    default:;
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Active state received unknown event %d\n", event.type);
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

int32_t _state_rotation_check_rpm;
#define LS_STATE_ROTATION_CHECK_STARTING_RPM 30
#define LS_STATE_ROTATION_CHECK_MINIMUM_RPM 15
#define LS_STATE_ROTATION_CHECK_MS_PER_RPM 500

ls_State ls_state_rotation_check(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_rotation_check;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: ls_state_rotation_check received event %d (LSEVT_STATE_ENTRY)\n", event.type);
#endif
        ls_leds_cycle(LEDCYCLE_WARNING);
        _state_rotation_check_rpm = LS_STATE_ROTATION_CHECK_STARTING_RPM;
        ls_stepper_spin_at_rpm(_state_rotation_check_rpm);
    break;

    case LSEVT_MAGNET_ENTER:
    case LSEVT_MAGNET_LEAVE:
        ls_buzzer_effect(LS_BUZZER_PLAY_HOME_SUCCESS);
        successor.func = ls_state_active;
    break;

    // left over from active state transferring control
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: ls_state_rotation_check received event %d (LSEVT_STEPPER_FINISHED_MOVE)\n", event.type);
#endif
    break;

    case LSEVT_STEPPER_REACHED_SPEED:
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: ls_state_rotation_check received event %d (LSEVT_STEPPER_REACHED_SPEED [%d])\n", event.type, _state_rotation_check_rpm);
#endif
        _state_rotation_check_rpm--;
        if(_state_rotation_check_rpm < LS_STATE_ROTATION_CHECK_MINIMUM_RPM)
        {
            successor.func = ls_state_error_norotate; 
        } else {
            vTaskDelay(pdMS_TO_TICKS(LS_STATE_ROTATION_CHECK_MS_PER_RPM));
            ls_stepper_spin_at_rpm(_state_rotation_check_rpm);
        }
        break;
    case LSEVT_MAGNET_FAILURE:
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: ls_state_rotation_check received event %d (LSEVT_MAGNET_FAILURE)\n", event.type);
#endif
        successor.func = ls_state_error_norotate;
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:
        ;//nothing to do
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: ls_state_rotation_check received UNEXPECTED event %d\n", event.type);
#endif
    }
    // exit actions:
    if (successor.func != ls_state_rotation_check)
    {
        ls_leds_off();
    }
    return successor;
    }

#ifdef LS_HAS_TAPE_SENSOR
static void *_ls_state_home_successor = NULL;
void ls_state_set_home_successor(void *successor)
{
    _ls_state_home_successor = successor;
}
ls_State ls_state_home(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: STATE_HOME handling event\n");
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
        case LS_TAPEMODE_DARK_SAFE:
        case LS_TAPEMODE_LIGHT_SAFE:
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
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:
        ls_substate_home_handle_event(event);
    }
    if (successor.func != ls_state_home)
    {
        ls_leds_off();
    }
    return successor;
}
#endif

ls_State ls_state_selftest(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: STATE_SELFTEST handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_selftest;
    selftest_event_handler(event);
    return successor;
}

ls_State ls_state_sleep(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: STATE_SLEEP handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_sleep;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_laser_set_mode_off();
        ls_servo_off();
        ls_stepper_stop(); // before sleeping
        ls_leds_off();
        ls_oled_blank_screen();
        while(ls_stepper_is_moving())
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        // the magnet sensor includes an LED which could pointlessly drain power during sleep
        // yes, wind might blow the arm back to the magnet, but at least we tried!
        if (ls_magnet_is_detected())
        {
            ls_stepper_forward_hop(LS_STEPPER_STEPS_PER_ROTATION / 4);
        } 
        else {
            ls_stepper_sleep();
            ls_event_enqueue_noop();
        }
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        // check whether the initial move of a quarter turn worked; if not, give it another smaller nudge
        if (ls_magnet_is_detected())
        {
            ls_stepper_forward_hop(LS_STEPPER_STEPS_PER_ROTATION / 7);
        }
        else
        {
            ls_stepper_sleep();
            ls_event_enqueue_noop();
        }
        break;
    case LSEVT_LIGHT_DAY:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: SLEEP received wake-up event\n");
#endif
        successor.func = ls_state_wakeup;
        break;
    case LSEVT_NOOP:
        // rate determined by LS_EVENT_NOOP_TIMEOUT_MS
        ls_buzzer_snore();
        break;
    case LSEVT_CONTROLS_UPPER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Entering settings upper from sleep\n");
#endif
        ls_state_set_prelaserwarn_successor(ls_state_settings_upper);
            successor.func = ls_state_prelaserwarn;
#ifdef LS_HAS_TAPE_SENSOR
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_state_set_home_successor(ls_state_prelaserwarn);
            successor.func = ls_state_home;
        }
#endif
        break;
    case LSEVT_CONTROLS_LOWER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Entering settings lower from sleep\n");
#endif
        ls_state_set_prelaserwarn_successor(ls_state_settings_lower);
        successor.func = ls_state_prelaserwarn;
#ifdef LS_HAS_TAPE_SENSOR
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_state_set_home_successor(ls_state_prelaserwarn);
            successor.func = ls_state_home;
        }
#endif
    break;
    case LSEVT_CONTROLS_BOTH:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: Entering secondary settings control from sleep\n")
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
    ls_debug_printf("STATES: STATE_WAKEUP handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_wakeup;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
            successor.func = ls_state_prelaserwarn;
#ifdef LS_HAS_TAPE_SENSOR
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_substate_home_require_rehome();
            ls_substate_home_init();
            ls_event_enqueue_noop();
        }
#endif
        break;
#ifdef LS_HAS_TAPE_SENSOR
    case LSEVT_HOME_COMPLETED:
        successor.func = ls_state_prelaserwarn;
        ls_event_enqueue_noop();
        break;
    case LSEVT_HOME_FAILED:
        successor.func = ls_state_error_home;
        ls_event_enqueue_noop();
        break;
#endif
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:
    ; 
#ifdef LS_HAS_TAPE_SENSOR
// should this be looking for specific events rather than handling everything else by default?
        ls_substate_home_handle_event(event);
#endif
    }
    return successor;
}

#ifdef LS_HAS_TAPE_SENSOR
static int _ls_state_map_build_steps_remaining;
static int _ls_state_map_enable_count = 0, _ls_state_map_disable_count = 0, _ls_state_map_misread_count = 0;
static enum ls_buzzer_effects _ls_state_map_fail_reason_tune = LS_BUZZER_PLAY_NOTHING;
ls_State ls_state_map_build(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: STATE_MAP_BUILD handling event %d with %d steps remaining\n", event.type, _ls_state_map_build_steps_remaining);
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
        ls_stepper_forward_hop(LS_MAP_RESOLUTION);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("STATES: case LSEVT_STEPPER_FINISHED_MOVE...\n");
#endif
        if (_ls_state_map_build_steps_remaining > 0)
        {
            _ls_state_map_build_read_and_set_map(&_ls_state_map_enable_count, &_ls_state_map_disable_count, &_ls_state_map_misread_count);
#ifdef LSDEBUG_STATES
            ls_debug_printf("STATES: continuing to next position...\n");
#endif
            ls_stepper_forward_hop(LS_MAP_RESOLUTION);
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
                switch (ls_tapemode())
                {
                case LS_TAPEMODE_DARK:
                case LS_TAPEMODE_DARK_SAFE:
                    _ls_state_map_fail_reason_tune = (0 == _ls_state_map_enable_count) ? LS_BUZZER_PLAY_TAPE_DISABLE : LS_BUZZER_PLAY_TAPE_ENABLE;
                    break;
                case LS_TAPEMODE_LIGHT:
                case LS_TAPEMODE_LIGHT_SAFE:
                    _ls_state_map_fail_reason_tune = (0 == _ls_state_map_enable_count) ? LS_BUZZER_PLAY_TAPE_ENABLE : LS_BUZZER_PLAY_TAPE_DISABLE;
                    break;
                default:;
                }
            }
            if (ls_map_is_excessive_misreads(_ls_state_map_misread_count))
            {
                badmap = true;
                _ls_state_map_fail_reason_tune = LS_BUZZER_PLAY_TAPE_MISREAD;
#ifdef LSDEBUG_MAP
                ls_debug_printf("\nBad map: too many misreads\n");
#endif
            }
            if (badmap)
            {
                ls_buzzer_effect(LS_BUZZER_PLAY_NOTHING);
                switch (ls_tapemode())
                {
                case LS_TAPEMODE_DARK_SAFE:
                case LS_TAPEMODE_LIGHT_SAFE:
                    // fail if a Safe mode
                    ls_map_set_status(LS_MAP_STATUS_FAILED);
                    successor.func = ls_state_error_scanning;
                    break;
                default:
                    // just play the fail tone and ignore map
                    ls_buzzer_effect(_ls_state_map_fail_reason_tune);
                    vTaskDelay(pdMS_TO_TICKS(1000)); // rest a second
                    ls_buzzer_effect(LS_BUZZER_PLAY_MAP_FAIL);
                    ls_map_set_status(LS_MAP_STATUS_IGNORE);
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
                ls_stepper_set_random_hop_strategy(ls_stepper_random_strategy_map_spans);
                ls_map_set_status(LS_MAP_STATUS_OK);
            }
            ls_tape_sensor_disable();
        } // done building map
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:; // nothing to do for event of this type
    }         // switch  on event
    return successor;
}
#endif

#ifdef LS_HAS_TAPE_SENSOR
ls_State ls_state_error_home(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: >>>>HOMING FAILED: ls_state_error_home<<<\n");
#endif
#ifdef LSDEBUG_HOMING
    ls_debug_printf("HOMING: >>>>HOMING FAILED<<<\n");
#endif
    _ls_state_everything_off();
    ls_State successor;
    successor.func = ls_state_error_home;
    ls_leds_cycle(LEDCYCLE_FAIL_HOMING);
    ls_buzzer_effect(LS_BUZZER_PLAY_HOME_FAIL);
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_LEDS_OFF_AFTER));
    ls_leds_off();
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_REPEAT_INTERVAL));
    ls_event_enqueue_noop();
    return successor;
}
#endif

#ifdef LS_HAS_TAPE_SENSOR
/** "map" is/was how I think of it, but "scanning" is more understandable to growers, I think*/
ls_State ls_state_error_scanning(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: >>>>Scanning FAILED: ls_state_error_scanning<<<\n");
#endif
#ifdef LSDEBUG_MAP
    ls_debug_printf("MAP: >>>>Scanning FAILED<<<\n");
#endif
    _ls_state_everything_off();
    ls_State successor;
    successor.func = ls_state_error_scanning;
    ls_leds_cycle(LEDCYCLE_FAIL_SCANNING);
    ls_buzzer_effect(_ls_state_map_fail_reason_tune);
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second between tones
    ls_buzzer_effect(LS_BUZZER_PLAY_MAP_FAIL);
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_LEDS_OFF_AFTER - 1000));
    ls_leds_off();
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: LEDs off by ls_state_error_scanning<<<\n");
#endif
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_REPEAT_INTERVAL));
    ls_event_enqueue_noop();
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: LSEVT_NOOP enqueued by returning ls_state_error_scanning<<<\n");
#endif
    return successor;
}
#endif

ls_State ls_state_error_tilt(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: >>>>Tilt Error: ls_state_error_tilt<<<\n");
#endif
#ifdef LSDEBUG_TILT
    ls_debug_printf("TILT: >>>>Tilt Error<<<\n");
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
        case LS_TAPEMODE_DARK_SAFE:
        case LS_TAPEMODE_LIGHT_SAFE:
#ifdef LSDEBUG_TILT
            ls_debug_printf("TILT: TILT_OK but safe mode requires power cycle to resume\n");
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
        ls_leds_cycle(LEDCYCLE_FAIL_TILT);
        ls_buzzer_effect(LS_BUZZER_PLAY_TILT_FAIL);
        // ledcycle for 5 seconds between alerts unless an event is pending
        if (pdFALSE == xQueuePeek(ls_event_queue, &event, pdMS_TO_TICKS(LS_FAILURE_LEDS_OFF_AFTER)))
        {
            ls_leds_off();
        }
        if (pdFALSE == xQueuePeek(ls_event_queue, &event, pdMS_TO_TICKS(LS_FAILURE_REPEAT_INTERVAL)))
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
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: >>>>NO ACCELEROMETER: ls_state_error_noaccel<<<\n");
#endif
    _ls_state_everything_off(); // not that we had a chance to turn anything on.
    ls_State successor;
    successor.func = ls_state_error_noaccel;
    // we're never really going to return
    ls_buzzer_effect(LS_BUZZER_PLAY_TILT_FAIL);
    ls_leds_cycle(LEDCYCLE_FAIL_ACCELEROMETER);
    vTaskDelay(pdMS_TO_TICKS(500));
    // if this is the first attempt just try to restart
    if (esp_reset_reason() != ESP_RST_SW)
    {
        esp_restart(); // software reset of the chip; starts execution again
    }
    // otherwise play the warning tone, too, and wait to try again
    ls_buzzer_effect(LS_BUZZER_ALTERNATE_HIGH);
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_LEDS_OFF_AFTER));
    ls_leds_off();
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_REPEAT_INTERVAL));
    esp_restart(); // software reset of the chip; starts execution again
    // will not be reached, but for consistency/type-correctness:
    return successor;
}

ls_State ls_state_error_norotate(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: >>>>NOT ROTATING: ls_state_error_norotate<<<\n");
#endif
    _ls_state_everything_off();
    ls_State successor;
    successor.func = ls_state_error_norotate;
    ls_leds_cycle(LEDCYCLE_FAIL_ROTATE);
    ls_buzzer_effect(LS_BUZZER_PLAY_NOROTATE);
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_LEDS_OFF_AFTER));
    ls_leds_off();
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_REPEAT_INTERVAL));
    ls_event_enqueue_noop();
    return successor;
}

ls_State ls_state_error_heartbeat(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATES: >>>>FAILSAFE HEARTBEAT NOT DETECTED: ls_state_error_heartbeat [%llu edges]<<<\n", ls_failsafe_edge_count());
#endif
    _ls_state_everything_off();
    ls_State successor;
    successor.func = ls_state_error_heartbeat;
    ls_leds_cycle(LEDCYCLE_FAIL_HEARTBEAT);
    ls_buzzer_effect(LS_BUZZER_PLAY_HEARTBEAT);
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_LEDS_OFF_AFTER));
    ls_leds_off();
    vTaskDelay(pdMS_TO_TICKS(LS_FAILURE_REPEAT_INTERVAL));
    ls_event_enqueue_noop();
    return successor;
}