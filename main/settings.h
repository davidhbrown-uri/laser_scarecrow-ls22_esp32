#pragma once
#include "freertos/FreeRTOS.h"

/**
 * @brief Load firmware defaults at power-up; will be overridden saved settings
 * 
 */
void ls_settings_set_defaults(void);
/**
 * @brief Overwrite any saved settings with firmware defaults
 * 
 */
void ls_settings_reset_defaults(void);
void ls_settings_read(void);
void ls_settings_save(void);

BaseType_t ls_settings_map_control_to_stepper_speed(BaseType_t adc);
void ls_settings_set_stepper_speed(BaseType_t steps_per_second);
BaseType_t ls_settings_get_stepper_speed(void);

BaseType_t ls_settings_map_control_to_servo_top(BaseType_t adc);
void ls_settings_set_servo_top(BaseType_t microseconds);
BaseType_t ls_settings_get_servo_top(void);

void ls_settings_set_servo_random_pause_ms(BaseType_t);
BaseType_t ls_settings_get_servo_random_pause_ms(void);

void ls_settings_set_servo_sweep_pause_ms(BaseType_t);
BaseType_t ls_settings_get_servo_sweep_pause_ms(void);

BaseType_t ls_settings_map_control_to_servo_bottom(BaseType_t adc);
void ls_settings_set_servo_bottom(BaseType_t microseconds);
BaseType_t ls_settings_get_servo_bottom(void);

BaseType_t ls_settings_map_control_to_pulse_delta(BaseType_t adc);
void ls_settings_set_servo_pulse_delta(BaseType_t);
BaseType_t ls_settings_get_servo_pulse_delta(void);

void ls_settings_set_stepper_random_max(BaseType_t steps);
BaseType_t ls_settings_get_stepper_random_max(void);

void ls_settings_set_light_threshold_on(BaseType_t adc);
BaseType_t ls_settings_get_light_threshold_on(void);

void ls_settings_set_light_threshold_off(BaseType_t adc);
BaseType_t ls_settings_get_light_threshold_off(void);

void ls_settings_set_tilt_threshold_mg_detected(BaseType_t milli_gs);
BaseType_t ls_settings_get_tilt_threshold_mg_detected(void);

void ls_settings_set_tilt_threshold_mg_ok(BaseType_t milli_gs);
BaseType_t ls_settings_get_tilt_threshold_mg_ok(void);