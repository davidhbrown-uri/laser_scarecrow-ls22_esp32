/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2023 David H. Brown

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
#include "freertos/queue.h"

enum ls_buzzer_scale
{
    LS_BUZZER_SCALE_bb = 967,  // b
    LS_BUZZER_SCALE_C = 1024,  // C
    LS_BUZZER_SCALE_Cx = 1085, // C#
    LS_BUZZER_SCALE_D = 1149,  // D
    LS_BUZZER_SCALE_Dx = 1218, // D#
    LS_BUZZER_SCALE_E = 1289,  // E
    LS_BUZZER_SCALE_F = 1367,  // F
    LS_BUZZER_SCALE_Fx = 1448, // F#
    LS_BUZZER_SCALE_G = 1534,  // G
    LS_BUZZER_SCALE_Gx = 1625, // G#
    LS_BUZZER_SCALE_A = 1722,  // A
    LS_BUZZER_SCALE_Ax = 1825, // A#
    LS_BUZZER_SCALE_B = 1933,  // B
    LS_BUZZER_SCALE_CC = 2048, // C'
    LS_BUZZER_SCALE_DD = 2300  // D'
} ls_buzzer_scale;

enum ls_buzzer_effects
{
    LS_BUZZER_CLICK,
    LS_BUZZER_CLICK_HIGH,
    LS_BUZZER_ALERT_1S,
    LS_BUZZER_ALTERNATE_HIGH,
    LS_BUZZER_PRE_LASER_WARNING,
    LS_BUZZER_PLAY_TAPE_ENABLE,            // enable tape read
    LS_BUZZER_PLAY_TAPE_DISABLE,           // disable tape read
    LS_BUZZER_PLAY_TAPE_MISREAD,           // bad tape read
    LS_BUZZER_PLAY_HOME_SUCCESS,           // home success fanfare
    LS_BUZZER_PLAY_HOME_FAIL,              // L home failure
    LS_BUZZER_PLAY_MAP_FAIL,               // descending minor cord
    LS_BUZZER_PLAY_TILT_FAIL,              // long descending scale
    LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER, // short ascending scale
    LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE, // short descending scale
    LS_BUZZER_PLAY_ROOT,                   // C, 200ms
    LS_BUZZER_PLAY_OCTAVE,                 // C', 200ms
    LS_BUZZER_PLAY_WAKE,                   // ascending scale alternating with root
    LS_BUZZER_PLAY_SLEEP,                  // descending scale alternating with octave
    LS_BUZZER_PLAY_TONE,                   // specify frequency using ls_buzzer_tone() instead of ls_buzzer_effect()
    LS_BUZZER_PLAY_NOTHING                 // 1 tick silence
} ls_buzzer_effects;

QueueHandle_t ls_buzzer_queue;

void ls_buzzer_init(void);

void ls_buzzer_effect(enum ls_buzzer_effects effect);

void ls_buzzer_note(enum ls_buzzer_scale note, TickType_t ticks);

void ls_buzzer_tone(BaseType_t frequency_hz);

void ls_buzzer_handler_task(void *pvParameter);

bool ls_buzzer_in_use(void);