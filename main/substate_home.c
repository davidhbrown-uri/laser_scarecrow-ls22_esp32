/*
    Control software for URI Laser Scarecrow, 2022-2023 Models
    Copyright (C) 2022-2023  David H. Brown

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

/**
 * Basic plan: rotate quickly to find the magnet.
 * Back up slowly to clear the magnet.
 * Move slowly to find the magnet...
 *   a) if never homed or if position offset > threshold, repeat and set to average
 *   b) if position offset <= threshold, do nothing... stepper is generally more accurate than magnet
 */

enum
{
    LS_HOME_SUBSTATE_COMPLETE,          // 0
    LS_HOME_SUBSTATE_FAILED,            // 1
    LS_HOME_SUBSTATE_ROTATE_TO_MAGNET,  // 2
    LS_HOME_SUBSTATE_BACKUP,            // 3
    LS_HOME_SUBSTATE_SLOW_TO_MAGNET,    // 4
    LS_HOME_SUBSTATE_WAIT_FOR_NO_STEPS, // 5 -- use only if there may not be a end move event, e.g., on init which could be poweron/wake
} _ls_substate_home_substate_phase;

static int _ls_home_attempts = 0;
static bool _ls_home_found_magnet = false;

static int _ls_home_homing_offsets[LS_HOME_HOMINGS_TO_AVERAGE];
static int _ls_home_homing_index = 0;

static bool _ls_home_homing_required = true; // true at power-on
static ls_stepper_position_t _ls_substate_magnet_entry_offset = 0;

/**
 * May be called by previous state... active does not require homing; power-on, wake-up do (when tape map in use).
 */
void ls_substate_home_require_rehome()
{
    _ls_home_homing_required = true;
}
void ls_substate_home_init()
{
    _ls_home_attempts = 0;
    _ls_home_homing_index = 0;
    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_WAIT_FOR_NO_STEPS;
    ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
}

/**
 * Use this substate after our own moves (not init)
 */
static void _ls_substate_home_backup(ls_event event)
{
    switch (event.type)
    {
    case LSEVT_SUBSTATE_ENTRY:
    {
        ls_stepper_position_t magnet_position = _ls_substate_magnet_entry_offset < 0 ? _ls_substate_magnet_entry_offset + LS_STEPPER_STEPS_PER_ROTATION : _ls_substate_magnet_entry_offset;
        // calculate how far to back up
        ls_stepper_position_t steps = (ls_stepper_get_position() - magnet_position);
        while (steps < 0)
        {
            steps += LS_STEPPER_STEPS_PER_ROTATION;
        }
#ifdef LSDEBUG_HOMING
        ls_debug_printf("Need to back up at least %d steps from stopped position %d to magnet entry at %d.\n",
                        steps, ls_stepper_get_position(), magnet_position);
#endif
        ls_stepper_set_maximum_steps_per_second(LS_HOME_INITIAL_STEPPER_STEPS_PER_SECOND);
        ls_stepper_reverse(steps + LS_HOME_BACKUP_ADDITIONAL_STEPS);
    }
    break;

    case LSEVT_STEPPER_FINISHED_MOVE:
        _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_SLOW_TO_MAGNET;
        ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
        break;
    default:; // do nothing
    }
}

static void _ls_substate_home_slow_to_magnet(ls_event event)
{
    switch (event.type)
    {
    case LSEVT_SUBSTATE_ENTRY:
        ls_stepper_set_maximum_steps_per_second(LS_HOME_STEPPER_STEPS_PER_SECOND);
        ls_stepper_forward(LS_HOME_FORWARD_STEPS);
        break;
    case LSEVT_MAGNET_ENTER:
#ifdef LSDEBUG_HOMING
        ls_debug_printf("Slow step found magnet at offset %d.\n", (int)event.value);
#endif
        ls_stepper_stop();
        /* if offset exceeds threshold, we will need to reset home position (might already be doig that; it's okay to set true twice!)*/
        if ((int)event.value > LS_HOME_OFFSET_THRESHOLD_TO_REHOME || -((int)event.value) > LS_HOME_OFFSET_THRESHOLD_TO_REHOME)
        {
            _ls_home_homing_required = true;
        }
        _ls_home_found_magnet = true;
        _ls_substate_magnet_entry_offset = (int) event.value;
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        if (_ls_home_found_magnet)
        {
            if (_ls_home_homing_required)
            {
                /** if we are setting home position, store offset in array */
                _ls_home_homing_offsets[_ls_home_homing_index] = _ls_substate_magnet_entry_offset;
#ifdef LSDEBUG_HOMING
                    ls_debug_printf(">>>> offset[%d]=%d;\n", _ls_home_homing_index, _ls_home_homing_offsets[_ls_home_homing_index] );
#endif
                /** if done collecting offsets, calculate/set new home; succeed */
                _ls_home_homing_index++;
                if (_ls_home_homing_index >= LS_HOME_HOMINGS_TO_AVERAGE)
                {
                    int average_offset = 0;
                    for (int i = 0; i < LS_HOME_HOMINGS_TO_AVERAGE; i++)
                    {
                        average_offset += _ls_home_homing_offsets[i];
                    }
                    average_offset /= LS_HOME_HOMINGS_TO_AVERAGE;
                    ls_stepper_set_home_offset(average_offset);
#ifdef LSDEBUG_HOMING
                    ls_debug_printf("Average offset was %d.\n", average_offset);
#endif
                    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_COMPLETE;
                    ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
                }    // if that was the last offset needed
                else /** backup for the next offset*/
                {
                    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_BACKUP;
                    ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
                }
            } // if (full) homing required
            else
            { // full rehoming not required
                _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_COMPLETE;
                ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
            }
        }    // if magnet found
        else // magnet not found
        {
#ifdef LSDEBUG_HOMING
            ls_debug_printf("Slow step to magnet failed. Move ended at position %d.\n", (int)ls_stepper_get_position());
#endif
            _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_FAILED;
            ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
        }
        break;
    default:; // do nothing
    }
}

static void _ls_substate_home_rotate_to_magnet(ls_event event)
{
    switch (event.type)
    {
    case LSEVT_SUBSTATE_ENTRY:
        ls_stepper_set_maximum_steps_per_second(LS_HOME_INITIAL_STEPPER_STEPS_PER_SECOND);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * LS_HOME_INITIAL_ROTATIONS);
        _ls_home_found_magnet = false; // have not yet found magnet
#ifdef LSDEBUG_HOMING
        ls_debug_printf("Homing is looking for magnet...\n");
#endif
        break;
    case LSEVT_MAGNET_ENTER:
        _ls_home_found_magnet = true; // found magnet
        ls_stepper_stop();
        _ls_substate_magnet_entry_offset = (ls_stepper_position_t)event.value;
#ifdef LSDEBUG_HOMING
        ls_debug_printf("Found magnet in initial rotation(s).\n");
        if (ls_stepper_get_steps_taken() > ((5 * LS_STEPPER_STEPS_PER_ROTATION) / 4))
        {
            ls_debug_printf("WARNING: took > %d rotation to find magnet!\n", ls_stepper_get_steps_taken() / LS_STEPPER_STEPS_PER_ROTATION);
        }
#endif
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        if (_ls_home_found_magnet)
        {
            _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_BACKUP;
            ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
        } // if found magnet
        else
        {
            // did not find magnet
#ifdef LSDEBUG_HOMING
            ls_debug_printf("FAILED: homing could not find magnet in %d rotations\n", LS_HOME_INITIAL_ROTATIONS);
#endif
            _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_FAILED;
            ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
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
    _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_ROTATE_TO_MAGNET;
    ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
}

static void _ls_substate_home_failed(ls_event event)
{
    _ls_home_attempts++;
    if (_ls_home_attempts >= LS_HOME_ATTEMPTS_ALLOWED)
    {
        _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_FAILED;
        ls_event_enqueue_front(LSEVT_HOME_FAILED);
    }
    else // try again from the top
    {
        _ls_substate_home_substate_phase = LS_HOME_SUBSTATE_WAIT_FOR_NO_STEPS;
        ls_event_enqueue(LSEVT_SUBSTATE_ENTRY);
    }
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
        _ls_substate_home_rotate_to_magnet(event);
        break;
    case LS_HOME_SUBSTATE_BACKUP:
        _ls_substate_home_backup(event);
        break;
    case LS_HOME_SUBSTATE_SLOW_TO_MAGNET:
        _ls_substate_home_slow_to_magnet(event);
        break;
    case LS_HOME_SUBSTATE_FAILED:
        _ls_substate_home_failed(event);
        break;
    case LS_HOME_SUBSTATE_COMPLETE:
        _ls_home_homing_required = false;
        ls_buzzer_effect(LS_BUZZER_PLAY_HOME_SUCCESS);
        ls_event_enqueue_front(LSEVT_HOME_COMPLETED);
        break;
    }
}
