#pragma once
#include "events.h"

// see http://c-faq.com/decl/recurfuncp.html
// "Q: How can I declare a function that can return a pointer to a function of the same type? I'm building a state machine with one function for each state.... "
typedef int (*ls_state_funcptr)(ls_event);	  /* generic function pointer */
typedef ls_state_funcptr (*ptr_ls_state_funptr)(ls_event);  /* ptr to ls_state fcn taking event and returning g.f.p. */
/*
ptr_ls_state_funptr ls_state_current;

ls_state_funcptr ls_state_homing_start();
ls_state_funcptr ls_state_homing_1find_magnet(ls_event);
ls_state_funcptr ls_state_homing_2back_up(ls_event);
ls_state_funcptr ls_state_homing_3step_to_edge(ls_event);
ls_state_funcptr ls_state_homing_error(ls_event);

ls_state_funcptr ls_state_active(ls_event);

*/

typedef struct ls_State {
    struct ls_State (*func)(ls_event);
}ls_State;

ls_State ls_state_current;

ls_State ls_state_active(ls_event);

ls_State ls_state_home_to_magnet(ls_event);
ls_State ls_state_home_to_magnet_2back_up(ls_event);
ls_State ls_state_home_to_magnet_3step_to_edge(ls_event);
ls_State ls_state_home_to_magnet_error(ls_event);
