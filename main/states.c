#include "debug.h"
#include "states.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
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

extern SemaphoreHandle_t print_mux;

TimerHandle_t _ls_state_rehome_timer;
void _ls_state_rehome_timer_callback(TimerHandle_t xTimer)
{
    ls_event event;
    event.type = LSEVT_REHOME_REQUIRED;
    event.value = NULL;
    if (xQueueSendToBack(ls_event_queue, (void *)&event, pdMS_TO_TICKS(5000)) != pdPASS)
    {
#ifdef LSDEBUG_STATES
        printf("WARNING: Could not enqueue LSEVT_REHOME_REQUIRED; will try to try again later\n");
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
 * @brief
 * @todo switch to using a queue with the full ls_event structure.
 * @see https://controllerstech.com/freertos-tutorial-5-using-queue/
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
        if (xQueueReceive(ls_event_queue, &event, portMAX_DELAY) != pdTRUE)
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
                ls_debug_printf("Event lost; no state function to handle it");
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
    ls_debug_printf("POWERON state handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_prelaserwarn_active;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_tapemode_init();
        switch (ls_tapemode())
        {
        case LS_TAPEMODE_IGNORE:
            successor.func = ls_state_prelaserwarn_active;
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
                successor.func = ls_state_prelaserwarn_active;
            }
            else
            {
#ifdef LSDEBUG_STATES
                ls_debug_printf("State poweron => map_build_substate_home\n");
#endif
                successor.func = ls_state_map_build_substate_home;
            }

        } // switch tapemode
        break;
    default: // switch event.type
        ;
    }
    return successor;
}

static bool _ls_state_prelaserwarn_buzzer_complete;
static bool _ls_state_prelaserwarn_movement_complete;
static int _ls_state_prelaserwarn_rotation_count;

ls_State ls_state_prelaserwarn_active(ls_event event)
{
#ifdef LSDEBUG_STATES
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("PRELASERWARN_ACTIVE state handling event\n");
    xSemaphoreGive(print_mux);
#endif
    ls_State successor;
    successor.func = ls_state_prelaserwarn_active;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        _ls_state_prelaserwarn_buzzer_complete = false;
        _ls_state_prelaserwarn_movement_complete = false;
        _ls_state_prelaserwarn_rotation_count = 0;
        while (ls_buzzer_in_use() || ls_stepper_is_moving())
        {
            vTaskDelay(1);
        }
        ls_event_empty_queue();
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1sec quiet/still before warning
        ls_buzzer_play(LS_BUZZER_PRE_LASER_WARNING);
        ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_MAX);
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
    default:; // does not handle other events
    }
    if (_ls_state_prelaserwarn_buzzer_complete && _ls_state_prelaserwarn_movement_complete)
    {
        successor.func = ls_state_active;
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1sec quiet/still after warning
    }
    return successor;
}

ls_State ls_state_active(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("ACTIVE state handling event\n");
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

        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_laser_set_mode_mapped();
            xTimerReset(_ls_state_rehome_timer, pdMS_TO_TICKS(5000));
        }
        else
        {
            ls_laser_set_mode_on();
        }
        break;
    case LSEVT_MAGNET_ENTER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Magnet Enter @ %d %s\n", *(int32_t *)event.value, ls_stepper_direction ? "-->" : "<--");
#endif
        ls_buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_MAGNET_LEAVE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Magnet Leave @ %d %s\n", *(int32_t *)event.value, ls_stepper_direction ? "-->" : "<--");
#endif
        ls_buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Stepper finished %s move @%d \n", ls_stepper_direction ? "-->" : "<--", ls_stepper_get_position());
#endif
        break;
    case LSEVT_REHOME_REQUIRED:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Rehoming from active state\n");
#endif
        successor.func = ls_state_active_substate_home;
        break;
    case LSEVT_LIGHT_NIGHT:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering night/sleep state\n")
#endif
            successor.func = ls_state_sleep;
        break;
    case LSEVT_CONTROLS_CONNECTED:
    case LSEVT_CONTROLS_SPEED:
    case LSEVT_CONTROLS_TOPANGLE:
    case LSEVT_CONTROLS_BOTTOMANGLE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering manual control\n")
#endif
            successor.func = ls_state_manual;
        break;

    default:;
#ifdef LSDEBUG_STATES
        ls_debug_printf("Unknown event %d", event.type);
#endif
    }
    // /exit behaviors:
    if (successor.func != ls_state_active)
    {
        ls_stepper_stop();
        // servo stop
        ls_laser_set_mode_off();
    }
    return successor;
}

ls_State ls_state_active_substate_home(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("ACTIVE_SUBSTATE_HOME handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_active_substate_home;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_substate_home_init();
        ls_event_enqueue_noop();
        break;
    case LSEVT_HOME_COMPLETED:
        successor.func = ls_state_active;
        ls_event_enqueue_noop();
        break;
    case LSEVT_HOME_FAILED:
        successor.func = ls_state_error_home;
        ls_event_enqueue_noop();
        break;
    default:
        ls_substate_home_handle_event(event);
    }
    return successor;
}

ls_State ls_state_selftest(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("SELFTEST state handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_selftest;

    return successor;
}

int _ls_state_manual_servo_hold_count = 0;
ls_State ls_state_manual(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("MANUAL state handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_manual;
    BaseType_t control_value;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_laser_set_mode((ls_map_get_status() == LS_MAP_STATUS_OK) ? LS_LASER_MAPPED : LS_LASER_ON);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        // ls_servo_sweep();
        ls_buzzer_play(LS_BUZZER_PLAY_MANUAL_CONTROL_ENTER);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        if (_ls_state_manual_servo_hold_count > 0)
        {
            _ls_state_manual_servo_hold_count--;
        }
        else
        {
            // ls_servo_sweep();
        }
        break;
    case LSEVT_CONTROLS_SPEED:
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_stepper_speed(ls_settings_map_control_to_stepper_speed(control_value));
        ls_stepper_set_maximum_steps_per_second(ls_settings_get_stepper_speed());
        break;
    case LSEVT_CONTROLS_TOPANGLE:
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_servo_top(ls_settings_map_control_to_servo_top(control_value));
        // ls_servo_moveto(ls_settings_get_servo_top());
        _ls_state_manual_servo_hold_count = 3;
        break;
    case LSEVT_CONTROLS_BOTTOMANGLE:
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_servo_bottom(ls_settings_map_control_to_servo_bottom(control_value));
        // ls_servo_moveto(ls_settings_get_servo_bottom());
        _ls_state_manual_servo_hold_count = 3;
        break;
    case LSEVT_CONTROLS_DISCONNECTED:
        ls_stepper_stop();
        successor.func = ls_state_active;
    default:;
    } // switch event type

    if (ls_state_manual != successor.func)
    {
        ls_buzzer_play(LS_BUZZER_PLAY_MANUAL_CONTROL_LEAVE);
        ls_settings_save();
    }
    return successor;
}

ls_State ls_state_sleep(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("SLEEP handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_sleep;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_laser_set_mode_off();
        // ls_servo_off();
        ls_stepper_forward(1);
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
        for (int i = 0; i < 20; i++)
        {
            ls_buzzer_play(LS_BUZZER_CLICK);
            vTaskDelay(5 - i / 5);
        }
        for (int i = 0; i < 40; i++)
        {
            ls_buzzer_play(LS_BUZZER_CLICK);
            vTaskDelay(1 + i / 3);
        }
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
    case LSEVT_CONTROLS_CONNECTED:
    case LSEVT_CONTROLS_SPEED:
    case LSEVT_CONTROLS_TOPANGLE:
    case LSEVT_CONTROLS_BOTTOMANGLE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering manual control\n")
#endif
            successor.func = ls_state_manual;
        break;
    default:;
    }
    return successor;
}

ls_State ls_state_wakeup(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("WAKEUP handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_wakeup;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_substate_home_init();
            ls_event_enqueue_noop();
        }
        else
        {
            successor.func = ls_state_prelaserwarn_active;
        }
        break;
    case LSEVT_HOME_COMPLETED:
        successor.func = ls_state_prelaserwarn_active;
        ls_event_enqueue_noop();
        break;
    case LSEVT_HOME_FAILED:
        successor.func = ls_state_error_home;
        ls_event_enqueue_noop();
        break;
    default:
        ls_substate_home_handle_event(event);
    }
    return successor;
}

static int _ls_state_map_build_steps_remaining;
static int _ls_state_map_misread_count;
static int _ls_state_map_enable_count;
static int _ls_state_map_disable_count;
static enum _ls_state_map_reading {
    LS_STATE_MAP_READING_DISABLE,
    LS_STATE_MAP_READING_ENABLE,
    LS_STATE_MAP_READING_MISREAD,
    LS_STATE_MAP_READING_INIT
} _ls_state_map_previous_read = LS_STATE_MAP_READING_INIT;

static void _ls_state_map_build_read_and_set_map(void)
{
    enum _ls_state_map_reading reading = LS_STATE_MAP_READING_MISREAD;
    BaseType_t raw_adc = ls_tape_sensor_read();
    BaseType_t position = ls_stepper_get_position();
    switch (ls_tapemode())
    {
    case LS_TAPEMODE_BLACK:
    case LS_TAPEMODE_BLACK_SAFE:
        if (raw_adc <= LS_REFLECTANCE_ADC_MAX_WHITE_BUCKET)
        {
            reading = LS_STATE_MAP_READING_ENABLE;
        }
        if (raw_adc >= LS_REFLECTANCE_ADC_MIN_BLACK_TAPE)
        {
            reading = LS_STATE_MAP_READING_DISABLE;
        }
        break;
    case LS_TAPEMODE_REFLECT:
    case LS_TAPEMODE_REFLECT_SAFE:
        if (raw_adc >= LS_REFLECTANCE_ADC_MIN_BLACK_BUCKET)
        {
            reading = LS_STATE_MAP_READING_ENABLE;
        }
        if (raw_adc <= LS_REFLECTANCE_ADC_MAX_SILVER_TAPE)
        {
            reading = LS_STATE_MAP_READING_DISABLE;
        }
        break;
    default:
        // we are ignoring the map, so why are we building a map?
        reading = LS_STATE_MAP_READING_ENABLE;
    }
    switch (reading)
    {
    case LS_STATE_MAP_READING_ENABLE:
        _ls_state_map_enable_count++;
        ls_map_enable_at(position);
        if (reading != _ls_state_map_previous_read)
        {
            ls_buzzer_play(LS_BUZZER_PLAY_TAPE_ENABLE);
        }
        break;
    case LS_STATE_MAP_READING_DISABLE:
        _ls_state_map_disable_count++;
        ls_map_disable_at(position);
        if (reading != _ls_state_map_previous_read)
        {
            ls_buzzer_play(LS_BUZZER_PLAY_TAPE_DISABLE);
        }
        break;
    case LS_STATE_MAP_READING_MISREAD:
        _ls_state_map_misread_count++;
        ls_map_disable_at(position);
        ls_buzzer_play(LS_BUZZER_PLAY_TAPE_MISREAD);
        break;
    default:; // init case only for previous
    }
    _ls_state_map_previous_read = reading;
#ifdef LSDEBUG_MAP
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("map @%d: %d [%d]=>%c\n", position, raw_adc, reading, ls_map_is_enabled_at(position) ? 'O' : '.');
    xSemaphoreGive(print_mux);
#endif
}

ls_State ls_state_map_build(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("MAP_BUILD state handling event %d with %d steps remaining\n", event.type, _ls_state_map_build_steps_remaining);
#endif
    ls_State successor;
    successor.func = ls_state_map_build;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        _ls_state_map_build_steps_remaining = LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION;
        _ls_state_map_misread_count = 0;
        ls_tape_sensor_enable();
        ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_MAPPING);
        ls_stepper_forward(LS_MAP_RESOLUTION);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
        ls_debug_printf("case LSEVT_STEPPER_FINISHED_MOVE...\n");
#endif
        if (_ls_state_map_build_steps_remaining >= 0)
        {
            _ls_state_map_build_read_and_set_map();
#ifdef LSDEBUG_STATES
            ls_debug_printf("continuing to next position...\n");
#endif
            ls_stepper_forward(LS_MAP_RESOLUTION);
            _ls_state_map_build_steps_remaining--;
        }
        else
        { // we're done building the map
#ifdef LSDEBUG_MAP
            ls_debug_printf("\nMapping completed with %d enabled, %d disabled, and %d misreads\n", _ls_state_map_enable_count, _ls_state_map_disable_count, _ls_state_map_misread_count);
#endif
            bool badmap = false;
            successor.func = ls_state_prelaserwarn_active;
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
                ls_buzzer_play(LS_BUZZER_PLAY_MAP_FAIL);
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
                int max_span_enabled = 0;
                int current_span = 0;
                for (int i = 0; i < LS_STEPPER_STEPS_PER_ROTATION * 2; i++)
                {
                    bool enabled = (bool)ls_map_is_enabled_at(i % LS_STEPPER_STEPS_PER_ROTATION);
                    if (enabled)
                    {
                        current_span++;
                    }
                    else
                    {
                        max_span_enabled = (current_span > max_span_enabled) ? current_span : max_span_enabled;
                        current_span = 0;
                    }
                }
                ls_settings_set_stepper_random_max(max_span_enabled / 2);
                ls_map_set_status(LS_MAP_STATUS_OK);
#ifdef LSDEBUG_MAP
                ls_debug_printf("Set LS_MAP_STATUS_OK; longest enabled span is %d steps.\n", max_span_enabled);
#endif
            }
        }     // done building map
    default:; // nothing to do for event of this type
    }         // switch  on event
    return successor;
}

ls_State ls_state_map_build_substate_home(ls_event event)
{
#ifdef LSDEBUG_STATES
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("MAP_BUILD_SUBSTATE_HOME handling event\n");
    xSemaphoreGive(print_mux);
#endif
    ls_State successor;
    successor.func = ls_state_map_build_substate_home;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_substate_home_init();
        ls_event_enqueue_noop();
        break;
    case LSEVT_HOME_COMPLETED:
        successor.func = ls_state_map_build;
        break;
    case LSEVT_HOME_FAILED:
        successor.func = ls_state_error_home;
        break;
    default:
        ls_substate_home_handle_event(event);
    }
    return successor;
}

ls_State ls_state_error_home(ls_event event)
{
    ls_gpio_initialize(); // turn things off
    ls_State successor;
    successor.func = ls_state_error_home;
#ifdef LSDEBUG_HOMING
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf(">>>>HOMING FAILED<<<\n");
    xSemaphoreGive(print_mux);
#endif

    ls_buzzer_play(LS_BUZZER_PLAY_HOME_FAIL);
    vTaskDelay(pdMS_TO_TICKS(30000)); // 30 seconds between alerts
    ls_event_enqueue_noop();
    return successor;
}

ls_State ls_state_error_map(ls_event event)
{
    if (LSEVT_STATE_ENTRY == event.type)
    {
        ls_gpio_initialize(); // turn things off
    }
    ls_State successor;
    successor.func = ls_state_error_map;
#ifdef LSDEBUG_MAP
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf(">>>>MAPPING FAILED<<<\n");
    xSemaphoreGive(print_mux);
#endif
    vTaskDelay(pdMS_TO_TICKS(30000)); // 30 seconds between alerts
    if (0 == _ls_state_map_enable_count)
    {
        ls_buzzer_play(LS_BUZZER_PLAY_TAPE_DISABLE);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second between tones
    }
    if (0 == _ls_state_map_disable_count)
    {
        ls_buzzer_play(LS_BUZZER_PLAY_TAPE_ENABLE);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second between tones
    }
    if (ls_map_is_excessive_misreads(_ls_state_map_misread_count))
    {
        ls_buzzer_play(LS_BUZZER_PLAY_TAPE_MISREAD);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second between tones
    }
    ls_buzzer_play(LS_BUZZER_PLAY_MAP_FAIL);
    ls_event_enqueue_noop();
    return successor;
}

ls_State ls_state_error_tilt(ls_event event)
{
    ls_gpio_initialize(); // turn things off
    ls_State successor;
    successor.func = ls_state_error_tilt;
#ifdef LSDEBUG_TILT
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf(">>>>I'VE FALLEN AND CAN'T GET UP<<<\n");
    xSemaphoreGive(print_mux);
#endif
    return successor;
}