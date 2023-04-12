#include "events.h"

// see http://c-faq.com/decl/recurfuncp.html
// "Q: How can I declare a function that can return a pointer to a function of the same type? I'm building a state machine with one function for each state.... "
typedef int (*ls_state_funcptr)(ls_event);	  /* generic function pointer */
typedef ls_state_funcptr (*_ls_state_funcptr)(ls_event);  /* ptr to ls_state fcn taking event and returning g.f.p. */

typedef struct ls_State {
    struct ls_State (*func)(ls_event);
}ls_State;
