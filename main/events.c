#include "events.h"

void ls_event_queue_init(void)
{
    ls_event_queue = xQueueCreate(32, sizeof(ls_event));
}

static void ls_event_enqueue_noop(void)
{
    ls_event event;
    event.type = LSEVT_NOOP;
    xQueueCRSendToFront(ls_event_queue, (void *)&event, 0);
}
