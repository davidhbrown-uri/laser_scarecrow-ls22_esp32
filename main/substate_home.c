#include "debug.h"
#include <stdio.h>
#include <stdlib.h>
#include "substate_home.h"
#include "stepper.h"
#include "buzzer.h"

extern QueueHandle_t ls_event_queue;
extern SemaphoreHandle_t print_mux;

#define LS_HOME_SUBSTATE_ROTATIONS_ALLOWED 3

enum
{
    LS_HOME_SUBSTATE_COMPLETE,            // 0
    LS_HOME_SUBSTATE_FAILED,              // 1
    LS_HOME_SUBSTATE_ROTATE_TO_MAGNET,    // 2
    LS_HOME_SUBSTATE_BACKUP_PAST_MAGNET,  // 3
    LS_HOME_SUBSTATE_SLOW_SEEK_TO_MAGNET, // 4
    LS_HOME_SUBSTATE_WAIT_FOR_STOP,       // 5
} _ls_substate_home_substate_phase;

static int _ls_substate_home_tries_remaining = 0;
static bool _ls_substate_home_phase_successful = false;

static void ls_substate_home_failed(void)
{
    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_FAILED;
    ls_event event;
    event.type = LSEVT_HOME_FAILED;
    event.value = NULL;
    xQueueSendToFront(ls_event_queue, (void *)&event, 0);
}
static void ls_substate_home_completed(void)
{
    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_COMPLETE;
    ls_event event;
    event.type = LSEVT_HOME_COMPLETED;
    event.value = NULL;
    xQueueSendToFront(ls_event_queue, (void *)&event, 0);
    ls_buzzer_play(LS_BUZZER_PLAY_HOME_SUCCESS);
}

void ls_substate_home_init(void)
{
    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_WAIT_FOR_STOP;
    ls_event_enqueue_noop();
}

static void _ls_substate_home_begin_rotate_to_magnet(void)
{
    // enter substate ROTATE_TO_MAGNET
    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_ROTATE_TO_MAGNET;
    ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_HOMING);
    ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * LS_HOME_SUBSTATE_ROTATIONS_ALLOWED);
    _ls_substate_home_phase_successful = false; // have not yet found magnet
#ifdef LSDEBUG_HOMING
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("Homing is looking for magnet...\n");
    xSemaphoreGive(print_mux);
#endif
    ls_event_enqueue_noop();
}

static void _ls_substate_home_handle_rotate_to_magnet(ls_event event)
{
#ifdef LSDEBUG_HOMING
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("...phase rotate to magnet handling event %d\n", event.type);
    xSemaphoreGive(print_mux);
#endif
    switch (event.type)
    {
    case LSEVT_MAGNET_ENTER:
        _ls_substate_home_phase_successful = true; // found magnet
        ls_stepper_stop();
#ifdef LSDEBUG_HOMING
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("homing found magnet; stepper stop requested...\n");
        if (ls_stepper_get_steps_taken() > LS_STEPPER_STEPS_PER_ROTATION)
        {
            printf("WARNING: took > 1 rotation to find magnet!\n");
        }
        xSemaphoreGive(print_mux);
#endif
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        if (_ls_substate_home_phase_successful)
        {
            // have found magnet
#ifdef LSDEBUG_HOMING
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("stepper stopped after homing found magnet; on to next phase...\n");
            xSemaphoreGive(print_mux);
#endif
            // enter substate BACKUP_PAST_MAGNET
            _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_BACKUP_PAST_MAGNET;
            _ls_substate_home_tries_remaining = 25;
            _ls_substate_home_phase_successful = false; // reset for next phase
            ls_event_enqueue_noop();
        }
        else
        {
            // did not find magnet
            _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_FAILED;
            ls_event_enqueue_noop();
#ifdef LSDEBUG_HOMING
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("homing could not find magnet in %d rotations\n", LS_HOME_SUBSTATE_ROTATIONS_ALLOWED);
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
    printf("...phase backup past magnet handling event %d\n", event.type);
    xSemaphoreGive(print_mux);
#endif
    switch (event.type)
    {
    case LSEVT_MAGNET_LEAVE:
        _ls_substate_home_phase_successful = true;
        ls_stepper_stop();
#ifdef LSDEBUG_HOMING
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Homing backed out of magnet area...\n");
        xSemaphoreGive(print_mux);
#endif
        break;
    case LSEVT_STEPPER_FINISHED_MOVE: // falls through to default
    case LSEVT_NOOP:                  // was queued by previous phase
    default:
        if (_ls_substate_home_phase_successful)
        {
            // finished stopping; enter substate SLOW_SEEK_TO_MAGNET
            _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_SLOW_SEEK_TO_MAGNET;
            _ls_substate_home_tries_remaining = LS_STEPPER_STEPS_PER_ROTATION / 8;
            _ls_substate_home_phase_successful = false;
            ls_stepper_forward(1); // get things started for slow stepping
        }
        else if (--_ls_substate_home_tries_remaining >= 0)
        {
#ifdef LSDEBUG_HOMING
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("Homing is backing up (%d)...\n", _ls_substate_home_tries_remaining);
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
    printf("...phase slow-seek to magnet handling event %d with %d tries remaining\n", event.type, _ls_substate_home_tries_remaining);
    xSemaphoreGive(print_mux);
#endif
    switch (event.type)
    {
    case LSEVT_MAGNET_ENTER:
        _ls_substate_home_phase_successful = true;
        ls_stepper_set_home_position();
        ls_substate_home_completed();
#ifdef LSDEBUG_HOMING
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Magnet home position found!\n");
        xSemaphoreGive(print_mux);
#endif
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        if (_ls_substate_home_phase_successful)
        {
            ls_stepper_set_home_position();
            ls_substate_home_completed();
        }
        else if (--_ls_substate_home_tries_remaining >= 0)
        {
#ifdef LSDEBUG_HOMING
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("Sending step request (%d tries remain)...\n", _ls_substate_home_tries_remaining);
            xSemaphoreGive(print_mux);
#endif
            ls_stepper_forward(1);
        }
        else
        {
#ifdef LSDEBUG_HOMING
            xSemaphoreTake(print_mux, portMAX_DELAY);
            printf("Homing could not slow-step to magnet area...\n");
            xSemaphoreGive(print_mux);
#endif
            ls_substate_home_failed();
        }
        break;
    default:; // do not handle this event type
    }
}

static void _ls_substate_home_wait_for_stop(void)
{

    while (ls_stepper_is_moving())
    {
        vTaskDelay(1);
    }
    // a pending LSEVT_STEPPER_FINISHED_MOVE would confuse homing
    ls_event_empty_queue();
    _ls_substate_home_begin_rotate_to_magnet();
    ls_event_enqueue_noop();
}

void ls_substate_home_handle_event(ls_event event)
{
#ifdef LSDEBUG_STATES
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf("SUBSTATE_HOME received forwarded event during phase %d\n", _ls_substate_home_substate_phase);
    xSemaphoreGive(print_mux);
#endif
    switch (_ls_substate_home_substate_phase)
    {
    case LS_HOME_SUBSTATE_WAIT_FOR_STOP:
        _ls_substate_home_wait_for_stop();
        break;
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
