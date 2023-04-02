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
#include "state_test_movement.h"
#include "stepper.h"
#include "buzzer.h"

extern QueueHandle_t ls_event_queue;
extern SemaphoreHandle_t print_mux;

static int _state_test_movement_phase = 0;

void ls_state_test_movement_handle_event(ls_event event)
{
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_stepper_set_maximum_steps_per_second(LSDEBUG_TEST_MOVEMENT_STEPS_PER_SECOND);
        ls_event_enqueue_noop();
        break;
    case LSEVT_NOOP:
        switch (_state_test_movement_phase)
        {
        case 0:
            ls_buzzer_note(LS_BUZZER_SCALE_D, 1);
            ls_stepper_forward(LSDEBUG_TEST_MOVEMENT_STEPS);
            break;
        case 1:
            ls_buzzer_note(LS_BUZZER_SCALE_E, 1);
            ls_stepper_reverse(LSDEBUG_TEST_MOVEMENT_STEPS / 8);
            break;
        case 2:
            ls_buzzer_note(LS_BUZZER_SCALE_F, 1);
            ls_stepper_forward(LSDEBUG_TEST_MOVEMENT_STEPS / 8);
            break;
        case 3:
            ls_buzzer_note(LS_BUZZER_SCALE_G, 1);
            ls_stepper_forward(LSDEBUG_TEST_MOVEMENT_STEPS / 4);
            break;
        case 4:
            ls_buzzer_note(LS_BUZZER_SCALE_A, 1);
            ls_stepper_reverse(LSDEBUG_TEST_MOVEMENT_STEPS / 4);
            break;
        default:
            _state_test_movement_phase = -1; //because we're about to increment it
            ls_event_enqueue_noop();
        }
        _state_test_movement_phase++;
        break;
    case LSEVT_MAGNET_ENTER:
#ifdef LSDEBUG_TEST_MOVEMENT
        ls_debug_printf("Magnet enter at position %d\n", (int)event.value);
#endif
        break;
    case LSEVT_MAGNET_LEAVE:
#ifdef LSDEBUG_TEST_MOVEMENT
        ls_debug_printf("Magnet leave at position %d\n", (int)event.value);
#endif
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        ls_buzzer_note(LS_BUZZER_SCALE_C, 2);
#ifdef LSDEBUG_TEST_MOVEMENT
        ls_debug_printf("Finished move at %d\n", ls_stepper_get_position());
#endif
        vTaskDelay(pdMS_TO_TICKS(1000));
        ls_event_enqueue_noop();
        break;
    default:
#ifdef LSDEBUG_TEST_MOVEMENT
        ls_debug_printf("Ignoring event type %d.\n", (int)event.type);
#endif
    }
}