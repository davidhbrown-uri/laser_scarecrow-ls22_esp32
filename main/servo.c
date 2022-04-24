#include "servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/mcpwm.h"
#include "esp_random.h"
#include "bootloader_random.h"
#include "config.h"
#include "debug.h"
#include "settings.h"

// Immediately jumps to the specified pulse_width
static void _ls_servo_jump_to_pw(uint32_t pulse_width)
{
    mcpwm_set_duty_in_us(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER, LS_SERVO_MCPWM_GENERATOR, pulse_width);
}

// Turns on the servo
static void _ls_servo_on()
{
    gpio_set_level(LSGPIO_SERVOPOWERENABLE, 1);
    mcpwm_start(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER);
}

// Turns off the servo
static void _ls_servo_off()
{
    gpio_set_level(LSGPIO_SERVOPOWERENABLE, 0);
    mcpwm_stop(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER);
}

// Helper to clamp an int to a fixed range
static int _ls_clamp(int n, int min, int max) {
    return n < min ? min : n > max ? max : n;
}

// Initializes the servo and servo task queue
void ls_servo_init()
{
    // Initialize the task queue
    ls_servo_queue = xQueueCreate(32, sizeof(struct ls_servo_event));

    // Enable bootloader random (for esp_random())
    bootloader_random_enable();

    // Set PWM0A to LSGPIO_SERVOPULSE (from the example code)
    mcpwm_gpio_init(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_IO_SIGNALS, LSGPIO_SERVOPULSE);
    mcpwm_config_t pwm_config = {
        .frequency = 50, // Frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // Duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0};
    mcpwm_init(LS_SERVO_MCPWM_UNIT, LS_SERVO_MCPWM_TIMER, &pwm_config); // Configure PWM0A & PWM0B with above settings
}

// Creates and sends a message to the servo task to turn on the servo
void ls_servo_on()
{
    struct ls_servo_event event;
    event.event_type = LS_SERVO_ON;
    event.data = 0;
    xQueueSend(ls_servo_queue, (void *) &event, 0);
}

// Creates and sends a message to the servo task to turn off the servo
void ls_servo_off()
{
    struct ls_servo_event event;
    event.event_type = LS_SERVO_OFF;
    event.data = 0;
    xQueueSend(ls_servo_queue, (void *) &event, 0);
}

// Creates and sends a message to the servo task to put the servo in sweep mode
void ls_servo_sweep()
{
    struct ls_servo_event event;
    event.event_type = LS_SERVO_SWEEP;
    event.data = 0;
    xQueueSend(ls_servo_queue, (void *) &event, 0);
}

// Creates and sends a message to the servo task to put the servo in random motion mode
void ls_servo_random()
{
    struct ls_servo_event event;
    event.event_type = LS_SERVO_MOVE_RANDOMLY;
    event.data = 0;
    xQueueSend(ls_servo_queue, (void *) &event, 0);
}

// Creates and sends a message to the servo task to move the servo smoothly to a specified position
void ls_servo_moveto(uint32_t pulsewidth_us)
{
    struct ls_servo_event event;
    event.event_type = LS_SERVO_MOVE_TO;
    event.data = pulsewidth_us;
    xQueueSend(ls_servo_queue, (void *) &event, 0);
}

// Creates and sends a message to the servo task to immediately jump the servo to a specified position
void ls_servo_jumpto(uint32_t pulsewidth_us)
{
    struct ls_servo_event event;
    event.event_type = LS_SERVO_JUMP_TO;
    event.data = pulsewidth_us;
    xQueueSend(ls_servo_queue, (void *) &event, 0);
}

void ls_servo_task(void *pvParameter)
{
    #ifdef LSDEBUG_SERVO
    ls_debug_printf("Initializing servo task\n");
    #endif

    // Turn on the servo
    _ls_servo_on();
    bool servo_is_on = true;

    const uint16_t PW_MAX_DELTA = 1;
    uint16_t current_pulse_width = LS_SERVO_US_MID;
    uint16_t target_pulse_width = LS_SERVO_US_MID;
    
    enum _ls_servo_motion_modes mode = LS_SERVO_MODE_FIXED;

    // Variable to hold the received event
    struct ls_servo_event received;

    while (true)
    {
        // Adjust the delay based on whether or not the servo should be moving
        bool servo_should_move = current_pulse_width != target_pulse_width || mode != LS_SERVO_MODE_FIXED;
        TickType_t delay = servo_should_move ? 1 : portMAX_DELAY;

        if (xQueueReceive(ls_servo_queue, &received, delay) == pdTRUE)
        {
            switch (received.event_type)
            {
            case LS_SERVO_ON:
                servo_is_on = true;

                break;

            case LS_SERVO_OFF:
                #ifdef LSDEBUG_SERVO
                ls_debug_printf("Servo task received off event, feeling... sleepy.....\n");
                #endif

                _ls_servo_off();
                servo_is_on = false;

                break;

            case LS_SERVO_JUMP_TO:
                #ifdef LSDEBUG_SERVO
                ls_debug_printf("Servo task received jump_to event with pulse width %d\n", received.data);
                #endif

                // If the servo is currently off, turn it on
                if (!servo_is_on)
                {
                    _ls_servo_on();
                    servo_is_on = true;
                }

                // TODO Confirm that using `received.data` works as expected
                target_pulse_width = received.data;
                _ls_servo_jump_to_pw(target_pulse_width);
                current_pulse_width = target_pulse_width;

                break;
            
            case LS_SERVO_MOVE_TO:
                #ifdef LSDEBUG_SERVO
                ls_debug_printf("Servo task received move_to event with pulse width %d\n", received.data);
                #endif

                // If the servo is currently off, turn it on
                if (!servo_is_on)
                {
                    _ls_servo_on();
                    servo_is_on = true;
                }

                // TODO Confirm that using `received.data` works as expected
                target_pulse_width = received.data;

                break;
            
            case LS_SERVO_MOVE_RANDOMLY:
                #ifdef LSDEBUG_SERVO
                ls_debug_printf("Servo task received move_randomly event, entering random motion mode\n");
                #endif

                mode = LS_SERVO_MODE_RANDOM;
                target_pulse_width = current_pulse_width;

                break;
            
            case LS_SERVO_SWEEP:
                #ifdef LSDEBUG_SERVO
                ls_debug_printf("Servo task received sweep event, entering sweep mode\n");
                #endif

                mode = LS_SERVO_MODE_SWEEP;
                target_pulse_width = current_pulse_width;
                
                break;

            default:
                #ifdef LSDEBUG_SERVO
                ls_debug_printf("Warning: Received unknown ls_servo_event_type\n");
                #endif
            }
        }

        // If no event was received within the delay, move the servo before looping again.
        else
        {
            // Check if we're already at the target pulse width,
            // and (possibly) change it depending on the mode we're in
            if(mode == LS_SERVO_MODE_RANDOM && current_pulse_width == target_pulse_width)
            {
                #ifdef LSDEBUG_SERVO
                ls_debug_printf("Servo reached target, choosing new random target\n");
                #endif

                uint16_t min = (uint16_t) ls_settings_get_servo_top();
                uint16_t max = (uint16_t) ls_settings_get_servo_bottom();
                target_pulse_width = esp_random() % (max - min + 1) + min;

                // Delay for 500ms
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            else if(mode == LS_SERVO_MODE_SWEEP && current_pulse_width == target_pulse_width)
            {
                // Update the target pulse width, depending on its current position
                // Most of the time, it will be at either end - but possibly not if we just entered sweep mode from another
                // In that case, move towards whichever end is currently further away
                uint16_t min = (uint16_t) ls_settings_get_servo_top();
                uint16_t max = (uint16_t) ls_settings_get_servo_bottom();
                uint16_t mid = (min + max) / 2;
                target_pulse_width = current_pulse_width < mid ? max : min;
            }

            // Calculate the updated current pulse width
            int min_new_pulse_width = current_pulse_width - PW_MAX_DELTA;
            int max_new_pulse_width = current_pulse_width + PW_MAX_DELTA;
            current_pulse_width = _ls_clamp(target_pulse_width, min_new_pulse_width, max_new_pulse_width);

            // Set the servo pulse width
            _ls_servo_jump_to_pw(current_pulse_width);
        }
    }
}