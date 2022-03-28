#include "debug.h"
#include <stdio.h>
#include <stdlib.h>
#include "substate_home.h"
#include "stepper.h"

extern QueueHandle_t ls_event_queue;

enum
{
    LS_HOME_SUBSTATE_COMPLETE,
    LS_HOME_SUBSTATE_FAILED,
    LS_HOME_SUBSTATE_ROTATE_TO_MAGNET,
    LS_HOME_SUBSTATE_BACKUP_PAST_MAGNET,
    LS_HOME_SUBSTATE_SLOW_SEEK_TO_MAGNET,
} ls_substate_home_substate;

static int ls_substate_home_tries = 0;
static bool ls_substate_home_status = false;

static void ls_substate_home_failed(void)
{
    ls_substate_home_substate = LS_HOME_SUBSTATE_FAILED;
    ls_event event;
    event.type = LSEVT_HOME_FAILED;
    event.value = NULL;
    xQueueSendToFront(ls_event_queue, (void *) &event, 0);
}
static void ls_substate_home_completed(void)
{
    ls_substate_home_substate = LS_HOME_SUBSTATE_COMPLETE;
    ls_event event;
    event.type = LSEVT_HOME_COMPLETED;
    event.value = NULL;
    xQueueSendToFront(ls_event_queue, (void *) &event, 0);
}

void ls_substate_home_init(void)
{
    // enter substate ROTATE_TO_MAGNET
    ls_substate_home_substate = LS_HOME_SUBSTATE_ROTATE_TO_MAGNET;
    ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 2);
    ls_substate_home_status = false; // have not yet found magnet
#ifdef LSDEBUG_HOMING
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("Homing is looking for magnet...\n");
    xSemaphoreGive(print_mux);
#endif
}

void ls_substate_home_handle_event(ls_event event)
{
    switch (ls_substate_home_substate)
    {
    case LS_HOME_SUBSTATE_ROTATE_TO_MAGNET:
        switch (event.type)
        {
        case LSEVT_MAGNET_ENTER:
            ls_substate_home_status = true; // found magnet
            ls_stepper_stop();
#ifdef LSDEBUG_HOMING
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("homing found magnet; waiting to stop stepper...\n");
            xSemaphoreGive(print_mux);
#endif
            break;
        case LSEVT_STEPPER_FINISHED_MOVE:
            if (ls_substate_home_status)
            {
                // have found magnet
#ifdef LSDEBUG_HOMING
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("homing found magnet; stepper now stopped...\n");
                xSemaphoreGive(print_mux);
#endif
                // enter substate BACKUP_PAST_MAGNET
                ls_substate_home_substate = LS_HOME_SUBSTATE_BACKUP_PAST_MAGNET;
                ls_substate_home_tries = 25;
                ls_substate_home_status = false;
                ls_event_enqueue_noop();
            }
            else
            {
                // did not find magnet
                ls_substate_home_substate = LS_HOME_SUBSTATE_FAILED;
                ls_event_enqueue_noop();
#ifdef LSDEBUG_HOMING
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("homing could not find magnet in two rotations\n");
                xSemaphoreGive(print_mux);
#endif
            }
            default:
            ;
        }

        break;
    case LS_HOME_SUBSTATE_BACKUP_PAST_MAGNET:
        switch (event.type)
        {
        case LSEVT_MAGNET_LEAVE:
            ls_substate_home_status = true;
            ls_stepper_stop();
#ifdef LSDEBUG_HOMING
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("Homing backed out of magnet area...\n");
            xSemaphoreGive(print_mux);
#endif
            break;
        case LSEVT_STEPPER_FINISHED_MOVE: // falls through to default
        default:
            if (ls_substate_home_status)
            {
                // finished stopping; enter substate SLOW_SEEK_TO_MAGNET
                ls_substate_home_substate = LS_HOME_SUBSTATE_SLOW_SEEK_TO_MAGNET;
                ls_substate_home_tries = 50;
                ls_substate_home_status = false;
                ls_event_enqueue_noop();
            }
            else if (--ls_substate_home_tries >= 0)
            {
#ifdef LSDEBUG_HOMING
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Homing is backing up (%d)...\n", ls_home_substate_tries);
                xSemaphoreGive(print_mux);
#endif
                ls_stepper_reverse(LS_STEPPER_STEPS_PER_ROTATION / 50);
            }
            else
            {
#ifdef LSDEBUG_HOMING
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Homing could not back out of magnet area...\n");
                xSemaphoreGive(print_mux);
#endif
                ls_substate_home_failed();
            }
        }

        break;
    case LS_HOME_SUBSTATE_SLOW_SEEK_TO_MAGNET:
        switch (event.type)
        {
        case LSEVT_MAGNET_ENTER:
            ls_substate_home_status = true;
            ls_stepper_set_home_position();
            ls_substate_home_completed();
#ifdef LSDEBUG_HOMING
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("Magnet home position found!\n");
            xSemaphoreGive(print_mux);
#endif
            break;
        case LSEVT_STEPPER_FINISHED_MOVE:
            if (ls_substate_home_status)
            {
                ls_stepper_set_home_position();
                ls_substate_home_completed();
            }
            else if (--ls_substate_home_tries >= 0)
            {
#ifdef LSDEBUG_HOMING
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Homing is slowly approaching magnet (%d)...\n", ls_home_substate_tries);
                xSemaphoreGive(print_mux);
#endif
                ls_stepper_forward(1);
            }
            else
            {
#ifdef LSDEBUG_HOMING
                xSemaphoreTake(print_mux, portMAX_DELAY);
                printf("Homing could not slow step to magnet area...\n");
                xSemaphoreGive(print_mux);
#endif
                ls_substate_home_failed();
            }
            break;
            default:
            ; // do not handle this event type
        }
        break;
    case LS_HOME_SUBSTATE_FAILED:
        ls_substate_home_failed();
        break;
    case LS_HOME_SUBSTATE_COMPLETE:
        ls_substate_home_completed();
        break;
    }
}



