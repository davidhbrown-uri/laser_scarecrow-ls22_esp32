#pragma once
#include "freertos/FreeRTOS.h"

void ls_settings_set_defaults(void);
void ls_settings_read_from_flash(void);
void ls_settings_save_to_flash(void);

void ls_settings_set_speed(BaseType_t);
BaseType_t ls_settings_get_speed(void);

void ls_settings_set_top_angle(BaseType_t);
BaseType_t ls_settings_get_top_angle(void);

void ls_settings_set_bottom_angle(BaseType_t);
BaseType_t ls_settings_get_bottom_angle(void);
