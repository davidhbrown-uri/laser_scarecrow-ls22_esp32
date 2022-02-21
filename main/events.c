#include "events.h"

extern QueueHandle_t ls_event_queue;

void ls_event_queue_init(void)
{
    ls_event_queue = xQueueCreate(32, sizeof(ls_event));
}