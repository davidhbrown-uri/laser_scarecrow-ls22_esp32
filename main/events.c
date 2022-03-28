#include "events.h"

void ls_event_queue_init(void)
{
    ls_event_queue = xQueueCreate(32, sizeof(ls_event));
}

void ls_event_enqueue_noop(void)
{
    ls_event event;
    event.type = LSEVT_NOOP;
    xQueueSendToFront(ls_event_queue, (void *)&event, 0);
}
