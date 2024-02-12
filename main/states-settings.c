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
#include "states-settings.h"
#include "settings.h"
#include "controls.h"
#include "events.h"
#include "buzzer.h"
#include "servo.h"
#include "leds.h"
#include "stepper.h"
#include "laser.h"
#include "map.h"
#include "lightsense.h"
#include "util.h"

ls_State ls_state_settings_upper(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_SETTINGS_UPPER (speed) handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_settings_upper;
    BaseType_t control_value;

    // somewhat awkward patch for #44 https://github.com/davidhbrown-uri/laser_scarecrow-ls22_esp32/issues/44
    if (LS_CONTROLS_STATUS_OFF == ls_controls_get_current_status())
    {
        event.type = LSEVT_CONTROLS_OFF;
    }

    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_stepper_set_maximum_steps_per_second(ls_settings_get_stepper_speed());
        ls_laser_set_mode((ls_map_get_status() == LS_MAP_STATUS_OK) ? LS_LASER_MAPPED : LS_LASER_ON);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        ls_servo_sweep();
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        ls_leds_cycle(LEDCYCLE_CONTROLS_UPPER);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        break;
    case LSEVT_SERVO_SWEEP_TOP:
        ls_buzzer_effect(LS_BUZZER_PLAY_OCTAVE);
        break;
    case LSEVT_SERVO_SWEEP_BOTTOM:
        ls_buzzer_effect(LS_BUZZER_PLAY_ROOT);
        break;
    case LSEVT_CONTROLS_SLIDER1: // stepper speed
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_stepper_speed(ls_settings_map_control_to_stepper_speed(control_value));
        ls_stepper_set_maximum_steps_per_second(ls_settings_get_stepper_speed());
        break;
    case LSEVT_CONTROLS_SLIDER2: // servo speed
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_servo_pulse_delta(ls_settings_map_control_to_servo_pulse_delta(control_value));
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Setting servo pulse delta to %d microseconds per tick.\n", ls_settings_map_control_to_servo_pulse_delta(control_value));
#endif
        break;
    case LSEVT_CONTROLS_OFF:
        ls_stepper_stop();
        successor.func = ls_state_active;
        break;
    case LSEVT_CONTROLS_LOWER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering lower from upper settings control\n")
#endif
            successor.func = ls_state_settings_lower;
        break;
    case LSEVT_CONTROLS_BOTH:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering both from upper settings control\n")
#endif
            successor.func = ls_state_settings_both;
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:;
    } // switch event type

    if (ls_state_settings_upper != successor.func)
    {
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE);
        ls_settings_save();
    }
    return successor;
}

static int _ls_state_settings_servo_hold_count = 0;
ls_State ls_state_settings_lower(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("STATE_SETTINGS_LOWER (servo top/range) handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_settings_lower;
    BaseType_t control_value;

    // somewhat awkward patch for #44 https://github.com/davidhbrown-uri/laser_scarecrow-ls22_esp32/issues/44
    if (LS_CONTROLS_STATUS_OFF == ls_controls_get_current_status())
    {
        event.type = LSEVT_CONTROLS_OFF;
    }

    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_stepper_set_maximum_steps_per_second(ls_settings_get_stepper_speed());
        ls_laser_set_mode((ls_map_get_status() == LS_MAP_STATUS_OK) ? LS_LASER_MAPPED : LS_LASER_ON);
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        ls_servo_sweep();
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        ls_leds_cycle(LEDCYCLE_CONTROLS_LOWER);
        break;
    case LSEVT_STEPPER_FINISHED_MOVE:
        ls_stepper_forward(LS_STEPPER_STEPS_PER_ROTATION * 5 / 4);
        if (_ls_state_settings_servo_hold_count > 0)
        {
            _ls_state_settings_servo_hold_count--;
        }
        else if (_ls_state_settings_servo_hold_count == 0)
        {
            ls_servo_sweep();
            _ls_state_settings_servo_hold_count = -1;
        }
        break;
    case LSEVT_SERVO_SWEEP_TOP:
        ls_buzzer_effect(LS_BUZZER_PLAY_OCTAVE);
        break;
    case LSEVT_SERVO_SWEEP_BOTTOM:
        ls_buzzer_effect(LS_BUZZER_PLAY_ROOT);
        break;
    case LSEVT_CONTROLS_SLIDER1:
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_servo_top(ls_settings_map_control_to_servo_top(control_value));
        ls_servo_jumpto(ls_servo_get_top_pulse_ms());
        _ls_state_settings_servo_hold_count = 3;
        break;
    case LSEVT_CONTROLS_SLIDER2:
        control_value = *((BaseType_t *)event.value);
        ls_settings_set_servo_bottom(ls_settings_map_control_to_servo_bottom(control_value));
        ls_servo_jumpto(ls_servo_get_bottom_pulse_ms());
        _ls_state_settings_servo_hold_count = 3;
        break;
    case LSEVT_CONTROLS_OFF:
        ls_stepper_stop();
        successor.func = ls_state_active;
        break;
    case LSEVT_CONTROLS_UPPER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering upper from lower settings control\n")
#endif
            successor.func = ls_state_settings_upper;
        break;
    case LSEVT_CONTROLS_BOTH:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering both from lower settings control\n")
#endif
            successor.func = ls_state_settings_both;
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    default:;
    } // switch event type

    if (ls_state_settings_lower != successor.func)
    {
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE);
        ls_settings_save();
    }
    return successor;
}

static enum ls_buzzer_scale _ls_state_settings_lightsense_scale[] = {
    // 0..3 => switch at lower light levels
    LS_BUZZER_SCALE_Dx, LS_BUZZER_SCALE_E, LS_BUZZER_SCALE_F, LS_BUZZER_SCALE_Fx,
    LS_BUZZER_SCALE_G, // 4 (5th value) => center of range
    // 5..8 => switch at higher levels
    LS_BUZZER_SCALE_Gx, LS_BUZZER_SCALE_A, LS_BUZZER_SCALE_Ax, LS_BUZZER_SCALE_B};
int _ls_state_settings_lightsense_index = -1; // an invalid value
int _ls_state_settings_lightsense_trills = 0;

void _ls_state_settings_both_status_leds(int index)
{
    if (LS_LIGHTSENSE_THRESHOLD_DEFAULT == index) // steady purply pink
    {
        ls_leds_rgb(128, 32, 64);
    }
    if (LS_LIGHTSENSE_THRESHOLD_DEFAULT > index) // blue pulses
    {
        ls_leds_pulses_2sec(LS_LIGHTSENSE_THRESHOLD_DEFAULT - index, GRB_BLUE);
    }
    if (LS_LIGHTSENSE_THRESHOLD_DEFAULT < index) // yellow pulses
    {
        ls_leds_pulses_2sec(index - LS_LIGHTSENSE_THRESHOLD_DEFAULT, GRB_YELLOW);
    }
}

/**
 * Return true if passed index pointer was set to match current threshold
 */
bool _ls_state_settings_both_current_light_threshold_index(int *index)
{
    int threshold = ls_settings_get_light_threshold_on();
    for (int i = 0; i < LS_LIGHTSENSE_THRESHOLDS_COUNT; i++)
    {
        if (threshold == (int[]){LS_LIGHTSENSE_THRESHOLDS_ON_MV}[i])
        {
            *index = i;
            return true;
        }
    }
    return false;
}

#define LS_STATE_SETTINGS_LIGHTSENSE_TRILL_LIMIT 8
ls_State ls_state_settings_both(ls_event event)
{
#ifdef LSDEBUG_STATES
    ls_debug_printf("CONTROLS_SETTINGS_BOTH handling event\n");
#endif
    ls_State successor;
    successor.func = ls_state_settings_both;
    BaseType_t control_value;
    int index;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_laser_set_mode_off();
        ls_stepper_stop();
        ls_servo_off();
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        ls_leds_cycle(LEDCYCLE_CONTROLS_BOTH);
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (_ls_state_settings_both_current_light_threshold_index(&index))
        {
            _ls_state_settings_both_status_leds(index);
            for (int i = 0; i < LS_STATE_SETTINGS_LIGHTSENSE_TRILL_LIMIT; i++)
            {
                ls_buzzer_note(_ls_state_settings_lightsense_scale[LS_LIGHTSENSE_THRESHOLD_DEFAULT], pdMS_TO_TICKS(150));
                ls_buzzer_note(_ls_state_settings_lightsense_scale[index], pdMS_TO_TICKS(150));
            }
        }
        break;

    case LSEVT_CONTROLS_SLIDER1: // light threshold
        control_value = *((BaseType_t *)event.value);
        // map input to index
        index = _map(_constrain(control_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP),
                     LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 0, LS_LIGHTSENSE_THRESHOLDS_COUNT - 1);
        if (index != _ls_state_settings_lightsense_index)
        {
            ls_settings_set_light_threshold_on(ls_lightsense_threshold_on_mv(index));
            ls_settings_set_light_threshold_off(ls_lightsense_threshold_off_mv(index));
#ifdef LSDEBUG_SETTINGS
            ls_debug_printf("Setting lightsense thresholds to index %d: on at %dmV; off at %dmV.\n",
                            index, ls_settings_get_light_threshold_on(), ls_settings_get_light_threshold_off());
#endif
            _ls_state_settings_both_status_leds(index);
            _ls_state_settings_lightsense_index = index;
            _ls_state_settings_lightsense_trills = 0;
        }
        if (_ls_state_settings_lightsense_trills < LS_STATE_SETTINGS_LIGHTSENSE_TRILL_LIMIT)
        {
            _ls_state_settings_lightsense_trills++;
            ls_buzzer_note(_ls_state_settings_lightsense_scale[LS_LIGHTSENSE_THRESHOLD_DEFAULT], pdMS_TO_TICKS(50));
            ls_buzzer_note(_ls_state_settings_lightsense_scale[index], pdMS_TO_TICKS(50));
        }
        break;
    case LSEVT_CONTROLS_SLIDER2:; // sleep light enable
        control_value = *((BaseType_t *)event.value);
        if (control_value >= LS_CONTROLS_READING_TOP && !ls_settings_is_sleep_light_enabled())
        {
            ls_settings_set_sleep_light_enable(true);
            ls_buzzer_snore();
        }
        if (control_value <= LS_CONTROLS_READING_BOTTOM  && ls_settings_is_sleep_light_enabled())
        {
            ls_settings_set_sleep_light_enable(false);
            ls_buzzer_snore();
        }
        break;
    case LSEVT_CONTROLS_OFF:
        successor.func = ls_state_wakeup; // because we turned laser off, must do wakeup and pre-laser warning
        break;
    case LSEVT_CONTROLS_UPPER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering upper from both settings control\n")
#endif
            ls_state_set_prelaserwarn_successor(ls_state_settings_upper);
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_state_set_home_successor(ls_state_prelaserwarn);
            successor.func = ls_state_home;
        }
        else
        {
            successor.func = ls_state_prelaserwarn;
        }
        break;
    case LSEVT_CONTROLS_LOWER:
#ifdef LSDEBUG_STATES
        ls_debug_printf("Entering lower from both settings control\n")
#endif
            ls_state_set_prelaserwarn_successor(ls_state_settings_lower);
        if (ls_map_get_status() == LS_MAP_STATUS_OK)
        {
            ls_state_set_home_successor(ls_state_prelaserwarn);
            successor.func = ls_state_home;
        }
        else
        {
            successor.func = ls_state_prelaserwarn;
        }
        break;
    case LSEVT_TILT_DETECTED:
        successor.func = ls_state_error_tilt;
        break;
    case LSEVT_NOOP:
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
    default:; // do nothing for this event
    }         // switch event type
    if (ls_state_settings_both != successor.func)
    {
        ls_substate_home_require_rehome();
        ls_buzzer_effect(LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE);
        ls_settings_save();
    }
    return successor;
}
