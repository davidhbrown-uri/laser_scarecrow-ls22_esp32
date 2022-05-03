#pragma once
#include "events.h"

// see http://c-faq.com/decl/recurfuncp.html
// "Q: How can I declare a function that can return a pointer to a function of the same type? I'm building a state machine with one function for each state.... "
typedef int (*ls_state_funcptr)(ls_event);	  /* generic function pointer */
typedef ls_state_funcptr (*_ls_state_funcptr)(ls_event);  /* ptr to ls_state fcn taking event and returning g.f.p. */
/*
ptr_ls_state_funptr ls_state_current;

ls_state_funcptr ls_state_homing_start();
ls_state_funcptr ls_state_homing_1find_magnet(ls_event);
ls_state_funcptr ls_state_homing_2back_up(ls_event);
ls_state_funcptr ls_state_homing_3step_to_edge(ls_event);
ls_state_funcptr ls_state_homing_error(ls_event);

ls_state_funcptr ls_state_active(ls_event);

*/
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
ls_State ls_state_manual(ls_event);
ls_State ls_state_sleep(ls_event);
ls_State ls_state_wakeup(ls_event);

ls_State ls_state_prelaserwarn(ls_event);

/**
 * @brief Default if not called is ls_state_active. 
 * 
 * typical use:
 *         ls_state_set_prelaserwarn_successor(&ls_state_manual);
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
