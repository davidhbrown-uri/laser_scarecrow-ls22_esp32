#include "debug.h"
#include "states.h"
#include "freertos/semphr.h"
#include "stepper.h"
#include "buzzer.h"
#include "magnet.h"
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
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("Received event %d\n", event.type);
            xSemaphoreGive(print_mux);
            if (ls_state_current.func != NULL) // we have a current state
            {
                ls_state_current = ls_state_current.func(event);
                // send state entry events if the state changed
                if (ls_state_current.func != previous_state.func)
                {
                    xSemaphoreTake(print_mux, portMAX_DELAY);
                    printf("Switching states; sending entry event\n");
                    xSemaphoreGive(print_mux);
                    xQueueSendToFront(ls_event_queue, (void *)&state_entry_event, 0);
                }
                previous_state.func = ls_state_current.func; // save current state as previous to next event
            }
            else
            {
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Event lost; no state function to handle it");
                xSemaphoreGive(print_mux);
                ; // do nothing; no state
            }
        }
    } // while 1 -- task must not exit
}

bool ls_state_home_to_magnet_status = false;
int ls_magnet_homing_tries = 50;

ls_State ls_state_error_home(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_error_home;
#ifdef LSDEBUG_HOMING
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf(">>>>HOMING FAILED<<<\n");
    xSemaphoreGive(print_mux);
#endif
    return successor;
}

ls_State ls_state_poweron(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_active;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Beginning poweron state\n");
        xSemaphoreGive(print_mux);
#endif
        break;
    }
}
ls_State ls_state_active(ls_event event)
{
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
        buzzer_play(LS_BUZZER_ALTERNATE_HIGH);
        ls_stepper_random();
        break;
    case LSEVT_MAGNET_ENTER:
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Magnet Enter @ %d %s\n", *(int32_t *)event.value, ls_stepper_direction ? "-->" : "<--");
        xSemaphoreGive(print_mux);
#endif
        buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_MAGNET_LEAVE:
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Magnet Leave @ %d %s\n", *(int32_t *)event.value, ls_stepper_direction ? "-->" : "<--");
        xSemaphoreGive(print_mux);
#endif
        buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
#ifdef LSDEBUG_STATES
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Stepper finished %s move @%d \n", ls_stepper_direction ? "-->" : "<--", ls_stepper_get_position());
        xSemaphoreGive(print_mux);
#endif
        break;
    default:
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
    ls_State successor;
    successor.func = ls_state_selftest;

    return successor;
}

ls_State ls_state_manual(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_manual;

    return successor;
}

ls_State ls_state_map_build(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_map_build;

    return successor;
}

ls_State ls_state_map_build_substate_home(ls_event event)
{
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