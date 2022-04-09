#include "debug.h"
#include <stdio.h>
#include <stdlib.h>
#include "substate_home.h"
#include "stepper.h"

extern QueueHandle_t ls_event_queue;
extern SemaphoreHandle_t print_mux;

enum
{
    LS_HOME_SUBSTATE_COMPLETE,            // 0
    LS_HOME_SUBSTATE_FAILED,              // 1
    LS_HOME_SUBSTATE_ROTATE_TO_MAGNET,    // 2
    LS_HOME_SUBSTATE_BACKUP_PAST_MAGNET,  // 3
    LS_HOME_SUBSTATE_SLOW_SEEK_TO_MAGNET, // 4
} ls_substate_home_substate;

static int ls_substate_home_tries = 0;
static bool ls_substate_home_status = false;

static void ls_substate_home_failed(void)
{
    ls_substate_home_substate = LS_HOME_SUBSTATE_FAILED;
    ls_event event;
    event.type = LSEVT_HOME_FAILED;
    event.value = NULL;
    xQueueSendToFront(ls_event_queue, (void *)&event, 0);
}
static void ls_substate_home_completed(void)
{
    ls_substate_home_substate = LS_HOME_SUBSTATE_COMPLETE;
    ls_event event;
    event.type = LSEVT_HOME_COMPLETED;
    event.value = NULL;
    xQueueSendToFront(ls_event_queue, (void *)&event, 0);
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

static void _ls_substate_home_handle_rotate_to_magnet(ls_event event)
{

#ifdef LSDEBUG_HOMING
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("...phase rotate to magnet\n");
        xSemaphoreGive(print_mux);
#endif
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
    default:;
    }
}

static void _ls_substate_home_handle_backup_past_magnet(ls_event event)
{
#ifdef LSDEBUG_HOMING
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("...phase backup past magnet\n");
        xSemaphoreGive(print_mux);
#endif
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
            printf("Homing is backing up (%d)...\n", ls_substate_home_tries);
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
}

static void _ls_substate_home_handle_slow_seek_to_magnet(ls_event event)
{
#ifdef LSDEBUG_HOMING
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("...phase slow seek to magnet\n");
        xSemaphoreGive(print_mux);
#endif
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
            printf("Homing is slowly approaching magnet (%d)...\n", ls_substate_home_tries);
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
    default:; // do not handle this event type
    }
}

void ls_substate_home_handle_event(ls_event event)
{
#ifdef LSDEBUG_STATES
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("SUBSTATE_HOME received forwarded event during phase %d\n", ls_substate_home_substate);
    xSemaphoreGive(print_mux);
#endif
    switch (ls_substate_home_substate)
    {
    case LS_HOME_SUBSTATE_ROTATE_TO_MAGNET:
        _ls_substate_home_handle_rotate_to_magnet(event);
        break;
    case LS_HOME_SUBSTATE_BACKUP_PAST_MAGNET:
        _ls_substate_home_handle_backup_past_magnet(event);
        break;
    case LS_HOME_SUBSTATE_SLOW_SEEK_TO_MAGNET:
        _ls_substate_home_handle_slow_seek_to_magnet(event);
        break;
    case LS_HOME_SUBSTATE_FAILED:
        ls_substate_home_failed();
        break;
    case LS_HOME_SUBSTATE_COMPLETE:
        ls_substate_home_completed();
        break;
    }
}
