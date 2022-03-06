#include "events.h"


void ls_event_queue_init(void)
{
    ls_event_queue = xQueueCreate(32, sizeof(ls_event));
}