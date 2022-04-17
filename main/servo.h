#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_servo_event_types {
    LS_SERVO_ON,            // Data: unused
    LS_SERVO_OFF,           // Data: unused
    LS_SERVO_JUMP_TO,       // Data: pulse_width
    LS_SERVO_MOVE_TO,       // Data: pulse_width
    LS_SERVO_MOVE_RANDOMLY, // Data: unused
    LS_SERVO_SWEEP          // Data: unused
};

enum _ls_servo_motion_modes {
    LS_SERVO_MODE_FIXED,  // Moves to a fixed position
    LS_SERVO_MODE_RANDOM, // Continuously moves to a random position
    LS_SERVO_MODE_SWEEP   // Continuously sweeps between the min and max positions
};

struct ls_servo_event {
    enum ls_servo_event_types event_type;
    uint16_t data;
};

QueueHandle_t ls_servo_queue;

void ls_servo_init(void);

void ls_servo_on(void);

void ls_servo_off(void);

void ls_servo_sweep(void);

void ls_servo_random(void);

void ls_servo_moveto(uint32_t pulsewidth_us);

void ls_servo_jumpto(uint32_t pulsewidth_us);

void ls_servo_task(void* pvParameter);