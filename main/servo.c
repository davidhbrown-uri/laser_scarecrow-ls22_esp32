#include "servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/mcpwm.h"
#include "config.h"
#include "debug.h"

#ifdef LSDEBUG_SERVO
extern SemaphoreHandle_t print_mux; // in ls2022_esp32.c
static void _ls_servo_emit_message(const char *msg)
{
    xSemaphoreTake(print_mux, portMAX_DELAY);
    printf(msg);
    xSemaphoreGive(print_mux);
}
#endif
// Jumps to the specified pulse_width
static void _ls_servo_jump_to_pw(uint32_t pulse_width)
{
    mcpwm_set_duty_in_us(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER, LS_SERVO_MCPWM_GENERATOR, pulse_width);
}
// Turns on the servo
void ls_servo_on()
{
    gpio_set_level(LSGPIO_SERVOPOWERENABLE, 1);
    mcpwm_start(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER);
}

// Turns off the servo
void ls_servo_off()
{
    gpio_set_level(LSGPIO_SERVOPOWERENABLE, 0);
    mcpwm_stop(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER);
}

// Initializes the servo
void ls_servo_init()
{
    // Initialize the task queue
    ls_servo_queue = xQueueCreate(32, sizeof(struct ls_servo_event));

    // Set PWM0A to LSGPIO_SERVOPULSE (from the example code)
    mcpwm_gpio_init(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_IO_SIGNALS, LSGPIO_SERVOPULSE);
    mcpwm_config_t pwm_config = {
        .frequency = 50, // Frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // Duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0};
    mcpwm_init(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER, &pwm_config); // Configure PWM0A & PWM0B with above settings
}

void ls_servo_task(void *pvParameter)
{
    // Initialize and turn on the servo
    // TODO Does it make more sense for the servo to start off or on?
    ls_servo_init();
    ls_servo_on();
    bool servo_is_on = true;

    const int MS_BETWEEN_STEPS = 20; // TODO Chosen based on pwm_config config of 50hz... not sure if this is at all related. Consult David

    const uint16_t PW_MAX_DELTA = 25;
    uint16_t current_pulse_width = LS_SERVO_US_MID;
    uint16_t target_pulse_width = LS_SERVO_US_MID;

    struct ls_servo_event received;

    while (true)
    {
        // Adjust the delay based on whether or not the servo is currently away from its target pulse width
        TickType_t delay = current_pulse_width != target_pulse_width ? pdMS_TO_TICKS(MS_BETWEEN_STEPS) : portMAX_DELAY;

        if (xQueueReceive(ls_servo_queue, &received, delay) == pdTRUE)
        {
            switch (received.event_type)
            {
            case LS_SERVO_SET_PULSE_WIDTH:
                // If the servo is currently off, still update the target pulse width but emit a warning
                if (!servo_is_on)
                {
                    ; // do nothing
#ifdef LSDEBUG_SERVO
                    _ls_servo_emit_message("Warning: Set the servo pulse width while the servo was not on!\n");
#endif
                }

                // TODO I am not confident that this is correct
                // UPDATE: I am confident that this is not correct
                // target_pulse_width = *((uint16_t*) received.data);
                // TODO Temporary workaround just to demonstrate that the event is received
                // target_pulse_width = target_pulse_width == LS_SERVO_US_MAX ? LS_SERVO_US_MIN : LS_SERVO_US_MAX;
                // TODO will this work?
                target_pulse_width = received.data;
                break;

            case LS_SERVO_ON:
                ls_servo_on();
                servo_is_on = true;
                break;

            case LS_SERVO_OFF:
                ls_servo_off();
                servo_is_on = false;
                break;

            default:
#ifdef LSDEBUG_SERVO
                _ls_servo_emit_message("Warning: Received unknown ls_servo_event_type " + received.event_type);
#endif
            }
        }

        // If no event was received within the delay, move the servo before looping again.
        else
        {
            // Calculate the updated current pulse width
            int new_current_pulse_width = target_pulse_width;
            if (new_current_pulse_width < current_pulse_width - PW_MAX_DELTA)
            {
                new_current_pulse_width = current_pulse_width - PW_MAX_DELTA;
            }
            else if (new_current_pulse_width > current_pulse_width + PW_MAX_DELTA)
            {
                new_current_pulse_width = current_pulse_width + PW_MAX_DELTA;
            }
            current_pulse_width = new_current_pulse_width;

            // Set the servo pulse width
            _ls_servo_jump_to_pw(current_pulse_width);
        }
    }
}