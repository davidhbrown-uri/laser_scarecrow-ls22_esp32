#include "states.h"
#include "stepper.h"
#include "buzzer.h"
#include "freertos/semphr.h"

extern SemaphoreHandle_t print_mux;

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
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Homing is looking for magnet...\n");
        xSemaphoreGive(print_mux);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 2);
        break;
    case LSEVT_MAGNET_ENTER:
        ls_stepper_stop();
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("homing found magnet...\n");
        xSemaphoreGive(print_mux);
        successor.func = ls_state_home_to_magnet_2back_up;
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("homing could not find magnet in two rotations\n");
        xSemaphoreGive(print_mux);
        successor.func = ls_state_homing_error;
        break;
    default:; // do nothing; not the event we're looking for
    }
    return successor;
}

/**
 * @brief Back up out of the magnet area to move forward into it more slowly
 *
 * @todo error if this is done too many times? Too long? too much distance?
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
        ls_stepper_reverse(LS_STEPPER_STEPS_PER_ROTATION / 200);
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
            ls_stepper_reverse(LS_STEPPER_STEPS_PER_ROTATION / 200);
        }
        break;
    case LSEVT_MAGNET_LEAVE:
        ls_stepper_stop();
        successor.func = ls_state_home_to_magnet_3step_to_edge;
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
        printf("Magnet Enter @ %d %s\n", *(int *)event.value, ls_stepper_direction ? "-->" : "<--");
        xSemaphoreGive(print_mux);
        buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_MAGNET_LEAVE:
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Magnet Leave @ %d %s\n", *(int *)event.value, ls_stepper_direction ? "-->" : "<--");
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