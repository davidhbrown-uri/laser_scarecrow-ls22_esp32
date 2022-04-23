#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

enum ls_event_types
{
    LSEVT_NOOP=0, // 0: nothing happened, but we need an event
    LSEVT_STATE_ENTRY=1, // 1: allow successor state to initialize
    // fire when the magnet on the rotating arm enter/leave the detection area of the Hall-effect sensor
    LSEVT_MAGNET_ENTER = 20, 
    LSEVT_MAGNET_LEAVE, 

    LSEVT_STEPPER_FINISHED_MOVE = 30, 

    LSEVT_SERVO_FINISHED_MOVE = 40,

    LSEVT_LIGHT_DAY = 50,
    LSEVT_LIGHT_NIGHT,

    LSEVT_CONTROLS_CONNECTED = 60,
    LSEVT_CONTROLS_DISCONNECTED, 
    LSEVT_CONTROLS_SPEED,
    LSEVT_CONTROLS_TOPANGLE,
    LSEVT_CONTROLS_BOTTOMANGLE,

    LSEVT_BUZZER_WARNING_COMPLETE = 80,

    LSEVT_HOME_COMPLETED = 100,
    LSEVT_HOME_FAILED,
    LSEVT_REHOME_REQUIRED,

    LSEVT_MAP_COMPLETED = 110,
    LSEVT_MAP_FAILED 
} ls_event_types;

typedef struct ls_event
{
    enum ls_event_types type;
    void *value;
} ls_event;

QueueHandle_t ls_event_queue;

void ls_event_queue_init(void);

/**
 * @brief Enqueue a meaningless event to ensure the event handler is called
 * 
 */
void ls_event_enqueue_noop(void);
bool ls_event_queue_has_messages(void);
void ls_event_enqueue_noop_if_queue_empty(void);