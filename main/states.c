#include "debug.h"
#include "states.h"
#include "freertos/semphr.h"
#include "stepper.h"
#include "servo.h"
#include "buzzer.h"
#include "magnet.h"
#include "tapemode.h"
#include "tape.h"
#include "map.h"
#include "init.h"
#include "substate_home.h"

extern SemaphoreHandle_t print_mux;

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
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("Received event %d\n", event.type);
            xSemaphoreGive(print_mux);
#endif
            if (ls_state_current.func != NULL) // we have a current state
            {
                ls_state_current = ls_state_current.func(event);
                // send state entry events if the state changed
                if (ls_state_current.func != previous_state.func)
                {
#ifdef LSDEBUG_STATES
                    xSemaphoreTake(print_mux, portMAX_DELAY);
                    printf("Switching states; sending entry event\n");
                    xSemaphoreGive(print_mux);
#endif
                    xQueueSendToFront(ls_event_queue, (void *)&state_entry_event, 0);
                }
                previous_state.func = ls_state_current.func; // save current state as previous to next event
            }
            else
            {
#ifdef LSDEBUG_STATES
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Event lost; no state function to handle it");
                xSemaphoreGive(print_mux);
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
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("POWERON state handling event\n");
    xSemaphoreGive(print_mux);
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
            ls_map_ignore();
            successor.func = ls_state_prelaserwarn_active;
#ifdef LSDEBUG_STATES
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("State poweron => active\n");
            xSemaphoreGive(print_mux);
#endif
            break;
        case LS_TAPEMODE_SELFTEST:
#ifdef LSDEBUG_STATES
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("State poweron => selftest\n");
            xSemaphoreGive(print_mux);
#endif
            ls_map_ignore();
            successor.func = ls_state_selftest;
            break;
        default:
#ifdef LSDEBUG_STATES
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("State poweron => build_substate_home\n");
            xSemaphoreGive(print_mux);
#endif
            successor.func = ls_state_map_build_substate_home;
        }
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
        _ls_state_prelaserwarn_buzzer_complete=false;
        _ls_state_prelaserwarn_movement_complete=false;
        _ls_state_prelaserwarn_rotation_count=0;
        ls_buzzer_play(LS_BUZZER_PRE_LASER_WARNING);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION / 2);
        break;
    case LSEVT_BUZZER_WARNING_COMPLETE:
        _ls_state_prelaserwarn_buzzer_complete = true;
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        _ls_state_prelaserwarn_rotation_count++;
        if(2 > _ls_state_prelaserwarn_rotation_count)
        {
            ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION / 2);
        }
        if(2 == _ls_state_prelaserwarn_rotation_count)
        {
            ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 3 / 2);
        }
        if(2 < _ls_state_prelaserwarn_rotation_count)
        {
            _ls_state_prelaserwarn_movement_complete = true;
        }
        break;
        default:; // does not handle other events
    }
    if(_ls_state_prelaserwarn_buzzer_complete && _ls_state_prelaserwarn_movement_complete)
    {
        successor.func = ls_state_active;
    }
    return successor;
}

ls_State ls_state_active(ls_event event)
{
#ifdef LSDEBUG_STATES
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("ACTIVE state handling event\n");
    xSemaphoreGive(print_mux);
#endif
    ls_State successor;
    successor.func = ls_state_active;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Beginning active state\n");
        xSemaphoreGive(print_mux);
#endif
        ls_stepper_random();
        ls_servo_random();
        break;
    case LSEVT_MAGNET_ENTER:
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Magnet Enter @ %d %s\n", *(int32_t *)event.value, ls_stepper_direction ? "-->" : "<--");
        xSemaphoreGive(print_mux);
#endif
        ls_buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_MAGNET_LEAVE:
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Magnet Leave @ %d %s\n", *(int32_t *)event.value, ls_stepper_direction ? "-->" : "<--");
        xSemaphoreGive(print_mux);
#endif
        ls_buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Stepper finished %s move @%d \n", ls_stepper_direction ? "-->" : "<--", ls_stepper_get_position());
        xSemaphoreGive(print_mux);
#endif
        break;
    default:;
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Unknown event %d", event.type);
        xSemaphoreGive(print_mux);
#endif
    }
    return successor;
}

ls_State ls_state_active_substate_home(ls_event event)
{
#ifdef LSDEBUG_STATES
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("ACTIVE_SUBSTATE_HOME handling event\n");
    xSemaphoreGive(print_mux);
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
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("SELFTEST state handling event\n");
    xSemaphoreGive(print_mux);
#endif
    ls_State successor;
    successor.func = ls_state_selftest;

    return successor;
}

ls_State ls_state_manual(ls_event event)
{
#ifdef LSDEBUG_STATES
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("MANUAL state handling event\n");
    xSemaphoreGive(print_mux);
#endif
    ls_State successor;
    successor.func = ls_state_manual;

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
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("MAP_BUILD state handling event %d with %d steps remaining\n", event.type, _ls_state_map_build_steps_remaining);
    xSemaphoreGive(print_mux);
#endif
    ls_State successor;
    successor.func = ls_state_map_build;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        _ls_state_map_build_steps_remaining = LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_RESOLUTION;
        _ls_state_map_misread_count = 0;
        ls_tape_sensor_enable();
        ls_stepper_forward(LS_MAP_RESOLUTION);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("case LSEVT_STEPPER_FINISHED_MOVE...\n");
        xSemaphoreGive(print_mux);
#endif
        if (_ls_state_map_build_steps_remaining >= 0)
        {
            _ls_state_map_build_read_and_set_map();
#ifdef LSDEBUG_STATES
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("continuing to next position...\n");
            xSemaphoreGive(print_mux);
#endif
            ls_stepper_forward(LS_MAP_RESOLUTION);
            _ls_state_map_build_steps_remaining--;
        }
        else
        { // we're done building the map
#ifdef LSDEBUG_MAP
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("\nMapping completed with %d enabled, %d disabled, and %d misreads\n", _ls_state_map_enable_count, _ls_state_map_disable_count, _ls_state_map_misread_count);
            xSemaphoreGive(print_mux);
#endif
            bool badmap = false;
            successor.func = ls_state_prelaserwarn_active;
            if (0 == _ls_state_map_enable_count || 0 == _ls_state_map_disable_count)
            {
                badmap = true;
#ifdef LSDEBUG_MAP
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("\nBad map: must include at least one enabled and one disabled reading.\n");
                xSemaphoreGive(print_mux);
#endif
            }
            if (ls_map_is_excessive_misreads(_ls_state_map_misread_count))
            {
                badmap = true;
#ifdef LSDEBUG_MAP
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("\nBad map: too many misreads\n");
                xSemaphoreGive(print_mux);
#endif
            }
            if (badmap)
            {
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
                        ls_map_ignore();
                    }
                    break;
                }
            } // handle bad map
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
