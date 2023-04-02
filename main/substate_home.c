/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022  David H. Brown

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
#include <stdio.h>
#include <stdlib.h>
#include "substate_home.h"
#include "stepper.h"
#include "buzzer.h"
#include "magnet.h"
#include "map.h"

extern QueueHandle_t ls_event_queue;
extern SemaphoreHandle_t print_mux;

enum
{
    LS_HOME_SUBSTATE_COMPLETE,         // 0
    LS_HOME_SUBSTATE_FAILED,           // 1
    LS_HOME_SUBSTATE_ROTATE_TO_MAGNET, // 2
    LS_HOME_SUBSTATE_WAIT_FOR_NO_STEPS, // 3 -- use only if there may not be a end move event, e.g., on init which could be poweron/wake
    LS_HOME_SUBSTATE_WAIT_FOR_END_MOVE, // 4 -- use this preferentially when we know we have asked the stepper to move
} _ls_substate_home_substate_phase;

static bool _ls_substate_home_phase_successful = false;

static int _ls_substate_home_initial_offsets[LS_HOME_INITIAL_HOMES_TO_AVERAGE];
static int _ls_substate_home_initial_count = 0;

static bool _ls_substate_home_find_average = false;

#ifdef LSDEBUG_HOMING
static int _ls_substate_debug_cumulative_error = 0;
#endif

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
    ls_buzzer_effect(LS_BUZZER_PLAY_HOME_SUCCESS);
}

void ls_substate_home_init(void)
{
    if (ls_map_get_status() == LS_MAP_STATUS_NOT_BUILT)
    {
        _ls_substate_home_find_average = true;
        _ls_substate_home_initial_count = 0;
    }
    else
    {
        _ls_substate_home_find_average = false;
    }
    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_WAIT_FOR_NO_STEPS;
    ls_event_enqueue_noop();
}

static void _ls_substate_home_begin_rotate_to_magnet(void)
{
    // enter substate ROTATE_TO_MAGNET
    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_ROTATE_TO_MAGNET;
    ls_magnet_find_home();
    ls_stepper_set_maximum_steps_per_second(
        _ls_substate_home_find_average
            ? LS_STEPPER_STEPS_PER_SECOND_HOMING_INITIAL
            : LS_STEPPER_STEPS_PER_SECOND_HOMING);
    ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * LS_HOME_ROTATIONS_ALLOWED);
    _ls_substate_home_phase_successful = false; // have not yet found magnet
#ifdef LSDEBUG_HOMING
    ls_debug_printf("Homing is looking for magnet...\n");
#endif
}

/**
 * Use this substate after our own moves (not init)
*/
static void _ls_substate_home_handle_backup(ls_event event)
{
    switch (event.type)
    {
        case LSEVT_STEPPER_FINISHED_MOVE:
            _ls_substate_home_begin_rotate_to_magnet();
        break;
        default:
        ; // do nothing
    }
}


static void _ls_substate_home_handle_rotate_to_magnet(ls_event event)
{
    switch (event.type)
    {
    case LSEVT_MAGNET_HOMED:
        _ls_substate_home_phase_successful = true; // found magnet
        ls_stepper_stop();

#ifdef LSDEBUG_HOMING
        _ls_substate_debug_cumulative_error += (int)event.value;
        ls_debug_printf("homing found magnet with offset of %d steps ; cumulative = %d...\n", (int)event.value, _ls_substate_debug_cumulative_error);
        if (ls_stepper_get_steps_taken() > ((5 * LS_STEPPER_STEPS_PER_ROTATION) / 4))
        {
            ls_debug_printf("WARNING: took > %d rotation to find magnet!\n", ls_stepper_get_steps_taken() / LS_STEPPER_STEPS_PER_ROTATION);
        }
#endif
        if (_ls_substate_home_find_average)
        {
            // save the offset in _ls_substate_home_initial_offsets except the first [0] is always 0
            _ls_substate_home_initial_offsets[_ls_substate_home_initial_count] =
                _ls_substate_home_initial_count == 0 ? 0 : _ls_substate_home_initial_offsets[_ls_substate_home_initial_count - 1] + (int)event.value;
#ifdef LSDEBUG_HOMING
            ls_debug_printf("Cumulative offset after home #%d is %d\n", _ls_substate_home_initial_count, _ls_substate_home_initial_offsets[_ls_substate_home_initial_count]);
#endif
            _ls_substate_home_initial_count++;
        }
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        if (_ls_substate_home_phase_successful)
        {
            // have found magnet
#ifdef LSDEBUG_HOMING
            ls_debug_printf("stepper stopped after homing found magnet; homing complete\n");
#endif
            if (_ls_substate_home_find_average && _ls_substate_home_initial_count < LS_HOME_INITIAL_HOMES_TO_AVERAGE)
            {
                // step an increasing fraction around the circle, then home again
#ifdef LSDEBUG_HOMING
            ls_debug_printf("Moving backwards #%d steps at maximum speed\n", LS_STEPPER_STEPS_PER_ROTATION * _ls_substate_home_initial_count / LS_HOME_INITIAL_HOMES_TO_AVERAGE);
#endif
                ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_MAX);
                ls_stepper_reverse(LS_STEPPER_STEPS_PER_ROTATION * _ls_substate_home_initial_count / (LS_HOME_INITIAL_HOMES_TO_AVERAGE));
                _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_WAIT_FOR_END_MOVE;
            } // we aren't done finding the average offset from the magnet
            else
            {
                if (_ls_substate_home_find_average)
                {
                    int average_offset = 0;
                    for (int i = 0; i < LS_HOME_INITIAL_HOMES_TO_AVERAGE; i++)
                    {
                        average_offset += _ls_substate_home_initial_offsets[i];
                    }
                    average_offset /= LS_HOME_INITIAL_HOMES_TO_AVERAGE;
                    ls_stepper_set_home_offset(average_offset);
#ifdef LSDEBUG_HOMING
                    ls_debug_printf("Average offset was %d\n", average_offset);
                    _ls_substate_debug_cumulative_error = 0;
#endif
                } // we can compute the average offset from the magnet

                ls_substate_home_completed();

                ls_event_enqueue_noop();
            } // we are done finding home, average or not
        }     // phase was not successful
        else
        {
            // did not find magnet
            _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_FAILED;
            ls_event_enqueue_noop();
#ifdef LSDEBUG_HOMING
            ls_debug_printf("homing could not find magnet in %d rotations\n", LS_HOME_ROTATIONS_ALLOWED);
#endif
        }
    default:;
    } // switch on event type
}


/**
 * This is a rather awkward method that can cause problems. 
 * Use only on first entry to homing in case the stepper is already stopped 
 * and there is not going to be any LSEVT_STEPPER_FINISHED_MOVE
 * (e.g., power-on or wake)
*/
static void _ls_substate_home_wait_for_stop(void)
{
#ifdef LSDEBUG_HOMING
        ls_debug_printf(ls_stepper_is_moving() ? "Waiting to stop...\n" : "Already stopped...\n");
#endif
    while (ls_stepper_is_moving())
    {
        vTaskDelay(1);
    }
    // a pending LSEVT_STEPPER_FINISHED_MOVE would confuse homing
    ls_event_empty_queue();
    _ls_substate_home_begin_rotate_to_magnet();
}

void ls_substate_home_handle_event(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("SUBSTATE_HOME handling forwarded event during phase %d\n", _ls_substate_home_substate_phase);
#endif
#ifndef LSDEBUG_STATES
#ifdef LSDEBUG_HOMING
    ls_debug_printf("SUBSTATE_HOME handling forwarded event %d during phase %d\n", event.type, _ls_substate_home_substate_phase);
#endif
#endif

    switch (_ls_substate_home_substate_phase)
    {
    case LS_HOME_SUBSTATE_WAIT_FOR_NO_STEPS:
        _ls_substate_home_wait_for_stop();
        break;
    case LS_HOME_SUBSTATE_ROTATE_TO_MAGNET:
        _ls_substate_home_handle_rotate_to_magnet(event);
        break;
    case LS_HOME_SUBSTATE_FAILED:
        ls_substate_home_failed();
        break;
    case LS_HOME_SUBSTATE_COMPLETE:
        ls_substate_home_completed();
        break;
    case LS_HOME_SUBSTATE_WAIT_FOR_END_MOVE:
        _ls_substate_home_handle_backup(event);
    }
#ifdef LSDEBUG_HOMING
    ls_debug_printf("(finished event %d during phase %d)\n", event.type, _ls_substate_home_substate_phase);
#endif
}
