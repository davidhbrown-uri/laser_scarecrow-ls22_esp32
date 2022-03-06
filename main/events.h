#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_event_types
{
    LSEVT_NOOP, // 0: nothing happened, but we need an event
    LSEVT_STATE_ENTRY, // 1: allow successor state to initialize
    // fire when the magnet on the rotating arm enter/leave the detection area of the Hall-effect sensor
    LSEVT_MAGNET_ENTER, // 2: 
    LSEVT_MAGNET_LEAVE, // 3: <! no value for magnet events
    LSEVT_STEPPER_FINISHED_MOVE, // 4: 
} ls_event_types;

typedef struct ls_event
{
    enum ls_event_types type;
    void *value;
} ls_event;

QueueHandle_t ls_event_queue;

void ls_event_queue_init(void);
