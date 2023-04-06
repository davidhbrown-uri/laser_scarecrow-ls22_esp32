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
#include "selftest.h"
#include "settings.h"
#include "servo.h"
#include "stepper.h"
#include "buzzer.h"
#include "util.h"
#include "laser.h"
#include "tape.h"
#include "tapemode.h"
#include "leds.h"
#include "oled.h"

static bool _selftest_switches_off = false;
static bool _selftest_switches_upper = false;
static bool _selftest_switches_lower = false;
static bool _selftest_switches_both = false;
static bool _selftest_slider1_min = false;
static bool _selftest_slider1_max = false;
static bool _selftest_slider2_min = false;
static bool _selftest_slider2_max = false;

static bool _selftest_light_low = false;
static bool _selftest_light_high = false;
static bool _selftest_magnet_enter = false;
static bool _selftest_magnet_leave = false;
static bool _selftest_tape_light = false;
static bool _selftest_tape_dark = false;
static bool _selftest_tilt_detect = false;
static bool _selftest_tilt_ok = false;
static bool _selftest_tapemode_darksafe = false;
static bool _selftest_tapemode_dark = false;
static bool _selftest_tapemode_ignore = false;
static bool _selftest_tapemode_light = false;
static bool _selftest_tapemode_lightsafe = false;

static bool _selftest_oled_update_needed = true;

static int _selftest_stepper_behavior_sequence = 0;

void _selftest_detected_event(bool * selftest)
{
    if(*selftest == false)
    {
        _selftest_oled_update_needed = true;
    }
    *selftest = true;
}

void _selftest_stepper_behavior(void)
{
    switch (_selftest_stepper_behavior_sequence)
    {
    case 0:
    case 1:
        ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_MIN);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION / 8);
        break;
    case 2:
        ls_stepper_reverse(LS_STEPPER_STEPS_PER_ROTATION / 8);
        break;
    case 3:
    case 4:
        ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_MAX);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION / 2);
        break;
    case 5:
        ls_stepper_reverse(LS_STEPPER_STEPS_PER_ROTATION / 2);
        break;
    case 6:
        ls_stepper_set_maximum_steps_per_second(LS_HOME_STEPPER_STEPS_PER_SECOND);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 2);
        break;
    case 7:
        ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_MAX);
        ls_stepper_random();
        break;
    case 20:
        // restart sequence
        _selftest_stepper_behavior_sequence = -1;
        break;
    default:;
    }
    _selftest_stepper_behavior_sequence++;
}

static void _selftest_update_leds(void)
{
    if (_selftest_switches_off && _selftest_switches_upper && _selftest_switches_lower && _selftest_switches_both && _selftest_slider1_min && _selftest_slider1_max && _selftest_slider2_min && _selftest_slider2_max)
    {
        if (_selftest_light_low && _selftest_light_high && _selftest_magnet_enter && _selftest_magnet_leave && _selftest_tape_light && _selftest_tape_dark && _selftest_tilt_detect && _selftest_tilt_ok && _selftest_tapemode_darksafe && _selftest_tapemode_dark && _selftest_tapemode_ignore && _selftest_tapemode_light && _selftest_tapemode_lightsafe)
        {
            ls_leds_rgb(0,128,0); // darker green
        }
        else
        {
            ls_leds_rgb(128,96,0); // darker yellow
        }
    }
}

static void _selftest_update_oled(void)
{
    if (!_selftest_oled_update_needed) {
        return;
    }
    _selftest_oled_update_needed = false;
    ls_oled_goto_line(0);
    if (_selftest_light_high && _selftest_light_low)
    {
        ls_oled_println("> Light OK");
    }
    else
    {
        ls_oled_println("Light: %c %c", _selftest_light_high ? ' ' : 0x01, _selftest_light_low ? ' ' : 0x02);
    }
    if (_selftest_tilt_detect && _selftest_tilt_ok)
    {
        ls_oled_println("> Tilt OK");
    }
    else
    {
        ls_oled_println("Tilte: %c %c", _selftest_tilt_detect ? ' ' : '/', _selftest_tilt_ok ? ' ' : '|');
    }

    ls_oled_goto_line(2);
    if (_selftest_magnet_enter && _selftest_magnet_leave)
    {
        ls_oled_println("> Magnet OK");
    }
    else
    {
        ls_oled_println("Magnet: %c %c", _selftest_magnet_enter ? ' ' : '+', _selftest_magnet_leave ? ' ' : '-');
    }
    if (_selftest_tape_dark && _selftest_magnet_leave)
    {
        ls_oled_println("> Tape OK");
    }
    else
    {
        ls_oled_println("Tape: %s %s", _selftest_tape_light ? "  " : "Lt", _selftest_tape_dark ? "  " : "Dk");
    }
    if (_selftest_tapemode_darksafe && _selftest_tapemode_dark && _selftest_tapemode_ignore && _selftest_tapemode_light && _selftest_tapemode_lightsafe)
    {
        ls_oled_println("> Mode Jmprs OK");
    }
    else
    {
        ls_oled_println("Mode: %c %c %c %c %c",
                        _selftest_tapemode_darksafe ? ' ' : 'D',
                        _selftest_tapemode_dark ? ' ' : 'd',
                        _selftest_tapemode_ignore ? ' ' : '-',
                        _selftest_tapemode_light ? ' ' : 'l',
                        _selftest_tapemode_lightsafe ? ' ' : 'L');
    }

    ls_oled_goto_line(6);
    if (_selftest_switches_off && _selftest_switches_upper && _selftest_switches_lower && _selftest_switches_both)
    {
        ls_oled_println("> Buttons OK");
    }
    else
    {
        ls_oled_println("Buttons: %c%c%c%c",
                        _selftest_switches_off ? ' ' : 'O',
                        _selftest_switches_upper ? ' ' : 'U',
                        _selftest_switches_lower ? ' ' : 'L',
                        _selftest_switches_both ? ' ' : 'B');
    }
    if (_selftest_slider1_min && _selftest_slider1_max && _selftest_slider2_min && _selftest_slider2_max)
    {
        ls_oled_println("> Sliders OK");
    }
    else
    {
        ls_oled_println("Slider 1%c%c; 2%c%c",
                        _selftest_slider1_min ? ' ' : 0x02,
                        _selftest_slider1_max ? ' ' : 0x01,
                        _selftest_slider2_min ? ' ' : 0x02,
                        _selftest_slider2_max ? ' ' : 0x01);
    }
}

void selftest_event_handler(ls_event event)
{
    BaseType_t *event_value = event.value;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_servo_jumpto(LS_SERVO_US_MID);
        ls_leds_cycle(LEDCYCLE_RAINBOW);
        vTaskDelay(pdMS_TO_TICKS(LS_SERVO_SELFTEST_HOLD_MS));
        ls_buzzer_effect(LS_BUZZER_PRE_LASER_WARNING);
        while (ls_buzzer_in_use())
        {
            vTaskDelay(1);
        }
        ls_settings_reset_defaults();
        xTaskCreate(&ls_tape_sensor_selftest_task, "tapesense_selftest", configMINIMAL_STACK_SIZE * 2, NULL, 10, NULL);
        xTaskCreate(&ls_tapemode_selftest_task, "tapemode_selftest", configMINIMAL_STACK_SIZE * 2, NULL, 10, NULL);
        ls_laser_pulse_init();
        ls_settings_set_servo_bottom(LS_SERVO_US_MAX);
        ls_settings_set_servo_top(LS_SERVO_US_MIN);
        ls_settings_set_servo_sweep_pause_ms(500);
        ls_servo_sweep();
        _selftest_stepper_behavior();
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        _selftest_stepper_behavior();
        break;
    case LSEVT_MAGNET_ENTER:
        ls_buzzer_effect(LS_BUZZER_CLICK_HIGH);
        _selftest_detected_event(&_selftest_magnet_enter);
        break;
    case LSEVT_MAGNET_LEAVE:
        ls_buzzer_effect(LS_BUZZER_CLICK);
        _selftest_detected_event(&_selftest_magnet_leave);
        break;
    case LSEVT_LIGHT_DAY:
        ls_buzzer_effect(LS_BUZZER_PLAY_WAKE);
        _selftest_detected_event(&_selftest_light_high);
        break;
    case LSEVT_LIGHT_NIGHT:
        ls_buzzer_effect(LS_BUZZER_PLAY_SLEEP);
        _selftest_detected_event(&_selftest_light_low);
        break;
    case LSEVT_CONTROLS_UPPER:
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        _selftest_detected_event(&_selftest_switches_upper);
        break;
    case LSEVT_CONTROLS_LOWER:
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        _selftest_detected_event(&_selftest_switches_lower);
        break;
    case LSEVT_CONTROLS_BOTH:
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        _selftest_detected_event(&_selftest_switches_both);
        break;
    case LSEVT_CONTROLS_OFF:
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE);
        ls_leds_off();
        _selftest_detected_event(&_selftest_switches_off);
        break;
    case LSEVT_CONTROLS_SLIDER1:
        ls_leds_rgb(_constrain(_map(*event_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 255, 0), 0, 255), _constrain(_map(*event_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 0, 255), 0, 255), 0);
        if (!ls_buzzer_in_use())
        {
            ls_buzzer_tone(_map(*event_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 1000, 2000));
        }
        if (*event_value >= LS_CONTROLS_READING_TOP)
        {
            _selftest_detected_event(&_selftest_slider1_max);
        }
        if (*event_value <= LS_CONTROLS_READING_BOTTOM)
        {
            _selftest_detected_event(&_selftest_slider1_min);
        }
        break;
    case LSEVT_CONTROLS_SLIDER2:
        ls_leds_rgb(0, _constrain(_map(*event_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 255, 0), 0, 255), _constrain(_map(*event_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 0, 255), 0, 255));
        if (!ls_buzzer_in_use())
        {
            ls_buzzer_tone(_map(*event_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 2000, 4000));
        }
        if (*event_value >= LS_CONTROLS_READING_TOP)
        {
            _selftest_detected_event(&_selftest_slider2_max);
        }
        if (*event_value <= LS_CONTROLS_READING_BOTTOM)
        {
            _selftest_detected_event(&_selftest_slider2_min);
        }
        break;
    case LSEVT_TILT_DETECTED:
        ls_buzzer_effect(LS_BUZZER_PLAY_TILT_FAIL);
        _selftest_detected_event(&_selftest_tilt_detect);
        break;
    case LSEVT_TILT_OK:
        ls_buzzer_effect(LS_BUZZER_ALERT_1S);
        _selftest_detected_event(&_selftest_tilt_ok);
        break;
    case LSEVT_SELFTEST_TAPE_DARK:
        _selftest_detected_event(&_selftest_tape_dark);
        break;
    case LSEVT_SELFTEST_TAPE_LIGHT:
        _selftest_detected_event(&_selftest_tape_light);
        break;
    case LSEVT_SELFTEST_MODE_DARKSAFE:
        _selftest_detected_event(&_selftest_tapemode_darksafe);
        break;
    case LSEVT_SELFTEST_MODE_DARK:
        _selftest_detected_event(&_selftest_tapemode_dark);
        break;
    case LSEVT_SELFTEST_MODE_IGNORE:
        _selftest_detected_event(&_selftest_tapemode_ignore);
        break;
    case LSEVT_SELFTEST_MODE_LIGHT:
        _selftest_detected_event(&_selftest_tapemode_light);
        break;
    case LSEVT_SELFTEST_MODE_LIGHTSAFE:
        _selftest_detected_event(&_selftest_tapemode_lightsafe);
        break;
    default:;
    }
    _selftest_update_leds();
    _selftest_update_oled();
}