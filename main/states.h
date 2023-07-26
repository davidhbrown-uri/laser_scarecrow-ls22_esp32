/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2023 David H. Brown

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
#pragma once
#include "events.h"

// see http://c-faq.com/decl/recurfuncp.html
// "Q: How can I declare a function that can return a pointer to a function of the same type? I'm building a state machine with one function for each state.... "
typedef int (*ls_state_funcptr)(ls_event);	  /* generic function pointer */
typedef ls_state_funcptr (*_ls_state_funcptr)(ls_event);  /* ptr to ls_state fcn taking event and returning g.f.p. */

void event_handler_state_machine(void *pvParameter);

/**
 * @brief Initialize timers 
 * 
 */
void ls_state_init(void);

typedef struct ls_State {
    struct ls_State (*func)(ls_event);
}ls_State;

ls_State ls_state_current, ls_state_previous;

ls_State ls_state_poweron(ls_event);
ls_State ls_state_selftest(ls_event);
ls_State ls_state_settings(ls_event);
ls_State ls_state_secondary_settings(ls_event);
ls_State ls_state_sleep(ls_event);
ls_State ls_state_wakeup(ls_event);

ls_State ls_state_prelaserwarn(ls_event);

/**
 * @brief Default if not called is ls_state_active. 
 * 
 * typical use:
 *         ls_state_set_prelaserwarn_successor(&ls_state_settings);
 *         successor.func = ls_state_prelaserwarn;
 * 
 */
void ls_state_set_prelaserwarn_successor(void*);
ls_State ls_state_active(ls_event);
/**
 * @brief Default if not called is ls_state_active. 
 * 
 * typical use:
 *         ls_state_set_home_successor(&ls_state_map_build);
 *         successor.func = ls_state_home;
 * 
 */
void ls_state_set_home_successor(void*);
ls_State ls_state_home(ls_event);

ls_State ls_state_map_build(ls_event);
ls_State ls_state_map_build_substate_home(ls_event);

ls_State ls_state_error_home(ls_event);
ls_State ls_state_error_map(ls_event);
ls_State ls_state_error_tilt(ls_event);
ls_State ls_state_error_noaccel(ls_event);
