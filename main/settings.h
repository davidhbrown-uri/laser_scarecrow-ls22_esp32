#pragma once
#include "freertos/FreeRTOS.h"

void ls_settings_set_defaults(void);
void ls_settings_read(void);
void ls_settings_save(void);

BaseType_t ls_settings_map_control_to_stepper_speed(BaseType_t adc);
void ls_settings_set_stepper_speed(BaseType_t steps_per_second);
BaseType_t ls_settings_get_stepper_speed(void);

BaseType_t ls_settings_map_control_to_servo_top(BaseType_t adc);
void ls_settings_set_servo_top(BaseType_t microseconds);
BaseType_t ls_settings_get_servo_top(void);

BaseType_t ls_settings_map_control_to_servo_bottom(BaseType_t adc);
void ls_settings_set_servo_bottom(BaseType_t microseconds);
BaseType_t ls_settings_get_servo_bottom(void);

void ls_settings_set_stepper_random_max(BaseType_t steps);
BaseType_t ls_settings_get_stepper_random_max(void);

void ls_settings_set_light_threshold_on(BaseType_t adc);
BaseType_t ls_settings_get_light_threshold_on(void);

void ls_settings_set_light_threshold_off(BaseType_t adc);
BaseType_t ls_settings_get_light_threshold_off(void);