#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

enum ls_event_t
{
    LSEVT_NOOP=0, // nothing happened, but we need an event to keep processing internal behavior
    LSEVT_STATE_ENTRY=1, // inserted by state machine on state change to allow new state to initialize
    
    LSEVT_MAGNET_ENTER = 20, // rotating arm's magnet enters detection area
    LSEVT_MAGNET_LEAVE, // rotating arm's magnet leaves detection area

    LSEVT_STEPPER_FINISHED_MOVE = 30, // rotating arm finishes movement

    LSEVT_SERVO_SWEEP_TOP = 40, // servo tilt reaches top of sweep range
    LSEVT_SERVO_SWEEP_BOTTOM = 41, // servo tilt reaches bottom of sweep range

    LSEVT_LIGHT_DAY = 50, // ambient light level moves above daytime light level threshold 
    LSEVT_LIGHT_NIGHT, // ambient light level moves below nighttime light level threshold 

    LSEVT_CONTROLS_CONNECTED = 60, // external controls switched "on"
    LSEVT_CONTROLS_DISCONNECTED, // extenral controls switched "off"
    LSEVT_CONTROLS_SPEED, // external control slider 1 moved (ls_event.value is pointer to ADC value)
    LSEVT_CONTROLS_TOPANGLE, // external control slider 2 moved (ls_event.value is pointer to ADC value)
    LSEVT_CONTROLS_BOTTOMANGLE, // external control slider 3 moved (ls_event.value is pointer to ADC value)

    LSEVT_BUZZER_WARNING_COMPLETE = 80, // long pre-laser warning sequence of tones has finished

    LSEVT_HOME_COMPLETED = 100, // the homing routine has moved the arm to the reference location
    LSEVT_HOME_FAILED, // the homing routing could not locate the reference location
    LSEVT_REHOME_REQUIRED, // homing must be done again to ensure stability of position over time

    LSEVT_MAP_COMPLETED = 110, // the locations where the laser can't be on has successfully been read
    LSEVT_MAP_FAILED, // the locations where the laser can't be on could not be read

    LSEVT_TILT_OK = 120, // the tilt sensor indicates the device orientation is in bounds
    LSEVT_TILT_DETECTED, // the tilt sensor indicates the device orientation is out of bounds
}; 

typedef struct ls_event
{
    enum ls_event_t type;
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
void ls_event_empty_queue(void);