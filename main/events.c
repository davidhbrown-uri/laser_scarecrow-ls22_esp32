#include "events.h"

void ls_event_queue_init(void)
{
    ls_event_queue = xQueueCreate(32, sizeof(ls_event));
}

void ls_event_enqueue_noop(void)
{
    ls_event event;
    event.type = LSEVT_NOOP;
    xQueueSendToBack(ls_event_queue, (void *)&event, 0);
}

bool ls_event_queue_has_messages(void)
{
    return uxQueueMessagesWaiting(ls_event_queue) > 0;
}
void ls_event_enqueue_noop_if_queue_empty(void)
{
    if (! ls_event_queue_has_messages()) 
    {
        ls_event_enqueue_noop();
    }
}