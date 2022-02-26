#pragma once
#include "events.h"

// see http://c-faq.com/decl/recurfuncp.html

typedef int (*ls_state_funcptr)(ls_event);	  /* generic function pointer */
typedef ls_state_funcptr (*ptr_ls_state_funptr)(ls_event);  /* ptr to ls_state fcn taking event and returning g.f.p. */

ls_state_funcptr ls_state_homing(ls_event);
ls_state_funcptr ls_state_active(ls_event);

