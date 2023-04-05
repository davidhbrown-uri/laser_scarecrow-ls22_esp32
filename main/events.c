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
#include "events.h"

void ls_event_queue_init(void)
{
    ls_event_queue = xQueueCreate(32, sizeof(ls_event));
}

void ls_event_enqueue_noop(void)
{
    ls_event_enqueue(LSEVT_NOOP);
}

/**
 * Add an event of specified type at the back of the queue; value is NULL
*/
void ls_event_enqueue(enum ls_event_t type)
{
    ls_event event;
    event.type = type;
    event.value = NULL;
    xQueueSendToBack(ls_event_queue, (void *)&event, 0);
}

/**
 * Insert an event of the specified type at the front of the queue; value is NULL
*/
void ls_event_enqueue_front(enum ls_event_t type)
{
    ls_event event;
    event.type = type;
    event.value = NULL;
    xQueueSendToFront(ls_event_queue, (void *)&event, 0);
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

void ls_event_empty_queue(void)
{
    xQueueReset(ls_event_queue);
}