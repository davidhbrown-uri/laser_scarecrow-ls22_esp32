#include "selftest.h"
#include "settings.h"
#include "servo.h"
#include "stepper.h"
#include "buzzer.h"
#include "util.h"
#include "laser.h"
#include "tape.h"
#include "tapemode.h"

int _selftest_stepper_behavior_sequence = 0;
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
        ls_stepper_set_maximum_steps_per_second(LS_STEPPER_STEPS_PER_SECOND_HOMING);
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
void selftest_event_handler(ls_event event)
{
    BaseType_t *event_value = event.value;
    switch (event.type)
    {
    case LSEVT_STATE_ENTRY:
        ls_servo_jumpto(LS_SERVO_US_MID);
        vTaskDelay(pdMS_TO_TICKS(LS_SERVO_SELFTEST_HOLD_MS));
        ls_buzzer_play(LS_BUZZER_PRE_LASER_WARNING);
        while(ls_buzzer_in_use())
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
    case LSEVT_MAGNET_LEAVE:
        ls_buzzer_play(LS_BUZZER_CLICK);
        break;
    case LSEVT_LIGHT_DAY:
        ls_buzzer_play(LS_BUZZER_PLAY_WAKE);
        break;
    case LSEVT_LIGHT_NIGHT:
        ls_buzzer_play(LS_BUZZER_PLAY_SLEEP);
        break;
    case LSEVT_CONTROLS_CONNECTED:
        ls_buzzer_play(LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER);
        break;
    case LSEVT_CONTROLS_DISCONNECTED:
        ls_buzzer_play(LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE);
        break;
    case LSEVT_CONTROLS_SPEED:
        if (!ls_buzzer_in_use())
        {
            ls_buzzer_tone(_map(*event_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 1000, 2000));
        }
        break;
    case LSEVT_CONTROLS_TOPANGLE:
        if (!ls_buzzer_in_use())
        {
            ls_buzzer_tone(_map(*event_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 2000, 4000));
        }
        break;
    case LSEVT_CONTROLS_BOTTOMANGLE:
        if (!ls_buzzer_in_use())
        {
            ls_buzzer_tone(_map(*event_value, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP, 500, 1000));
        }
        break;
    case LSEVT_TILT_DETECTED:
        ls_buzzer_play(LS_BUZZER_PLAY_TILT_FAIL);
        break;
    case LSEVT_TILT_OK:
        ls_buzzer_play(LS_BUZZER_ALERT_1S);
        break;
    default:;
    }
}