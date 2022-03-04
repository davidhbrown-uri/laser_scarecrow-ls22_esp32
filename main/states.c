#include "states.h"
#include "freertos/semphr.h"
#include "stepper.h"
#include "buzzer.h"
#include "magnet.h"

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
    ls_event event;
    ls_event state_entry_event;
    state_entry_event.type = LSEVT_STATE_ENTRY;
    state_entry_event.value = NULL;
    ls_State previous_state;
    previous_state.func = NULL;

    // if the ls_state_current is set prior to beginning this task, start it up with an entry event
    if (ls_state_current.func != NULL)
    {
        xQueueSendToFront(ls_event_queue, (void *)&state_entry_event, 0);
    }

    while (1)
    {
        if (xQueueReceive(ls_event_queue, &event, portMAX_DELAY) != pdTRUE)
        {
            printf("No events received after maximum delay... getting very bored.");
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
        // send the event
    } // while 1 -- task must not exit
}

bool ls_state_home_to_magnet_status = false;

ls_State ls_state_homing_error(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_homing_error;
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf(">>>>HOMING FAILED<<<\n");
    xSemaphoreGive(print_mux);
    return successor;
}

ls_State ls_state_home_to_magnet(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_home_to_magnet;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_state_home_to_magnet_status = false;
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Homing is looking for magnet...\n");
        xSemaphoreGive(print_mux);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 2);
        break;
    case LSEVT_MAGNET_ENTER:
        ls_state_home_to_magnet_status = true;
        ls_stepper_stop();
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("homing found magnet; waiting to stop stepper...\n");
        xSemaphoreGive(print_mux);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        if (ls_state_home_to_magnet_status)
        {
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("homing found magnet; stepper now stopped...\n");
            xSemaphoreGive(print_mux);
            successor.func = ls_state_home_to_magnet_2back_up;
        }
        else
        {
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("homing could not find magnet in two rotations\n");
            xSemaphoreGive(print_mux);
            successor.func = ls_state_homing_error;
        }
        break;
    default:; // do nothing; not the event we're looking for
    }
    return successor;
}

/**
 * @brief Back up out of the magnet area to move forward into it more slowly
 *
 * @param event
 * @return ls_state_funcptr
 */
ls_State ls_state_home_to_magnet_2back_up(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_home_to_magnet_2back_up;
    int tries = 50;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Homing is backing up...\n");
        xSemaphoreGive(print_mux);
        ls_state_home_to_magnet_status = ls_magnet_is_detected();
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("...magnet was %sdetected...\n", ls_state_home_to_magnet_status ? "" : "not ");
        xSemaphoreGive(print_mux);
        ls_stepper_reverse(LS_STEPPER_STEPS_PER_ROTATION / 100);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        if (--tries <= 0)
        {
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("Homing could not back out of magnet area...\n");
            xSemaphoreGive(print_mux);
            successor.func = ls_state_homing_error;
        }
        else
        {
            ls_stepper_reverse(LS_STEPPER_STEPS_PER_ROTATION / 100);
        }
        break;
    case LSEVT_MAGNET_LEAVE:
        if (ls_state_home_to_magnet_status)
        {
            ls_stepper_stop();
            successor.func = ls_state_home_to_magnet_3step_to_edge;
        }
        break;
    case LSEVT_MAGNET_ENTER:
        ls_state_home_to_magnet_status = true;
        break;
    default:;
    }
    return successor;
}

ls_State ls_state_home_to_magnet_3step_to_edge(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_home_to_magnet_3step_to_edge;
    int tries = 100;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Homing is slowly approaching magnet area...\n");
        xSemaphoreGive(print_mux);
        ls_stepper_forward(1);
        break;
    case LSEVT_MAGNET_ENTER:
        ls_stepper_set_home_position();
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Magnet home position found!\n");
        xSemaphoreGive(print_mux);
        successor.func = ls_state_active;
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        if (--tries == 0)
        {
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("Homing could not find magnet again...\n");
            xSemaphoreGive(print_mux);
            successor.func = ls_state_homing_error;
        }
        else
        {
            ls_stepper_forward(1);
        }
        break;
    default:;
    }
    return successor;
}

ls_State ls_state_active(ls_event event)
{
    ls_State successor;
    successor.func = ls_state_active;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Beginning active state\n");
        xSemaphoreGive(print_mux);
        buzzer_play(LS_BUZZER_ALTERNATE_HIGH);
        break;
    case LSEVT_MAGNET_ENTER:
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Magnet Enter @ %d %s\n", *(int32_t *)event.value, ls_stepper_direction ? "-->" : "<--");
        xSemaphoreGive(print_mux);
        buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_MAGNET_LEAVE:
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Magnet Leave @ %d %s\n", *(int32_t *)event.value, ls_stepper_direction ? "-->" : "<--");
        xSemaphoreGive(print_mux);
        buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        printf("Stepper finished %s move @%d \n", ls_stepper_direction ? "-->" : "<--", ls_stepper_get_position());
        break;
    default:
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Unknown event %d", event.type);
        xSemaphoreGive(print_mux);
    }
    return successor;
}