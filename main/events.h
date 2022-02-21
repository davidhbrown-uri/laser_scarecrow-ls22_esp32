#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_event_types {
    // fire when the magnet on the rotating arm enter/leave the detection area of the Hall-effect sensor
    LSEVT_MAGNET_ENTER, LSEVT_MAGNET_LEAVE // <! no value for magnet events
};
typedef struct ls_event {
    enum ls_event_types type;
    void *value;    
}ls_event;

extern QueueHandle_t ls_event_queue;

void ls_event_queue_init(void);

