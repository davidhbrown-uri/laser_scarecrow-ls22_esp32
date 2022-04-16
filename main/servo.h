#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_servo_event_types {
    LS_SERVO_ON,             // Data: unused
    LS_SERVO_OFF,            // Data: unused
    LS_SERVO_SET_PULSE_WIDTH // Data: pulse_width
};

struct ls_servo_event {
    enum ls_servo_event_types event_type;
    uint16_t data;
};

QueueHandle_t ls_servo_queue;

void ls_servo_on(void);

void ls_servo_off(void);

void ls_servo_init(void);

void ls_servo_task(void* pvParameter);