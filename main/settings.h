/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022  David H. Brown

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
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

BaseType_t ls_settings_map_control_to_servo_pulse_delta(BaseType_t adc);
void ls_settings_set_servo_pulse_delta(BaseType_t);
BaseType_t ls_settings_get_servo_pulse_delta(void);

void ls_settings_set_stepper_random_max(BaseType_t steps);
BaseType_t ls_settings_get_stepper_random_max(void);

void ls_settings_set_light_threshold_on(BaseType_t adc);
BaseType_t ls_settings_get_light_threshold_on(void);

void ls_settings_set_light_threshold_off(BaseType_t adc);
BaseType_t ls_settings_get_light_threshold_off(void);

/**
 * @brief Set thresholds to identifiable values with a logrithmic response
 * 
 * @param index 
 */
void ls_settings_set_light_thresholds_from_0to10(int index);

void ls_settings_set_tilt_threshold_mg_detected(BaseType_t milli_gs);
BaseType_t ls_settings_get_tilt_threshold_mg_detected(void);

void ls_settings_set_tilt_threshold_mg_ok(BaseType_t milli_gs);
BaseType_t ls_settings_get_tilt_threshold_mg_ok(void);