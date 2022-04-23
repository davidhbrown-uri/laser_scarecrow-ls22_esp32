#include "settings.h"
#include "config.h"
#include "util.h"

static BaseType_t _ls_settings_stepper_speed, _ls_settings_servo_top, _ls_settings_servo_bottom;
static BaseType_t _ls_settings_stepper_random_max, _ls_settings_light_threshold_on, _ls_settings_light_threshold_off;

void ls_settings_set_defaults(void)
{
    ls_settings_set_stepper_speed(LS_STEPPER_STEPS_PER_SECOND_DEFAULT);
    ls_settings_set_servo_top(LS_SERVO_US_MIN);
    ls_settings_set_servo_bottom(LS_SERVO_US_MAX);

    ls_settings_set_stepper_random_max(LS_STEPPER_MOVEMENT_STEPS_MAX);
    ls_settings_set_light_threshold_on(LS_LIGHTSENSE_DAY_THRESHOLD);
    ls_settings_set_light_threshold_off(LS_LIGHTSENSE_NIGHT_THRESHOLD);
}

void ls_settings_read_from_flash(void)
{
#ifdef LSDEBUG_SETTINGS
    // caution... happens before creating mutexes, so can't use ls_debug_printf
    printf("Reading settings from flash\n");
#endif
    ;
}

void ls_settings_save_to_flash(void)
{
#ifdef LSDEBUG_SETTINGS
    ls_debug_printf("Saving settings to flash\n");
#endif
    ;
}

BaseType_t ls_settings_map_control_to_stepper_speed(BaseType_t adc)
{
    return _map(_constrain(adc, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP),
                LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP,
                LS_STEPPER_STEPS_PER_SECOND_MIN, LS_STEPPER_STEPS_PER_SECOND_MAX);
}
void ls_settings_set_stepper_speed(BaseType_t steps_per_second)
{
    _ls_settings_stepper_speed = _constrain(steps_per_second, LS_STEPPER_STEPS_PER_SECOND_MIN, LS_STEPPER_STEPS_PER_SECOND_MAX);
}
BaseType_t ls_settings_get_stepper_speed(void)
{
    return _ls_settings_stepper_speed;
}

BaseType_t ls_settings_map_control_to_servo_top(BaseType_t adc)
{
    return _map(_constrain(adc, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP),
                LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP,
                LS_SERVO_US_MIN, LS_SERVO_US_MAX);
}
void ls_settings_set_servo_top(BaseType_t microseconds)
{
    // preserve fixed angle: if the top is equal to the bottom, change the bottom to match
    if (_ls_settings_servo_top == _ls_settings_servo_bottom)
    {
        _ls_settings_servo_bottom = _constrain(microseconds, LS_SERVO_US_MIN, LS_SERVO_US_MAX);
    }
    _ls_settings_servo_top = _constrain(microseconds, LS_SERVO_US_MIN, LS_SERVO_US_MAX);
    _ls_settings_servo_bottom = _constrain(_ls_settings_servo_bottom, _ls_settings_servo_top, LS_SERVO_US_MAX);
}
BaseType_t ls_settings_get_servo_top(void)
{
    return _ls_settings_servo_top;
}

BaseType_t ls_settings_map_control_to_servo_bottom(BaseType_t adc)
{
    return _map(_constrain(adc, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP),
                LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP,
                ls_settings_get_servo_top(), LS_SERVO_US_MAX);
}
void ls_settings_set_servo_bottom(BaseType_t microseconds)
{
    _ls_settings_servo_bottom = _constrain(microseconds, ls_settings_get_servo_top(), LS_SERVO_US_MAX);
}
BaseType_t ls_settings_get_servo_bottom(void)
{
    return _ls_settings_servo_bottom;
}

void ls_settings_set_stepper_random_max(BaseType_t steps)
{
    _ls_settings_stepper_random_max = _constrain(steps, LS_STEPPER_MOVEMENT_STEPS_MIN, LS_STEPPER_MOVEMENT_STEPS_MAX);
}
BaseType_t ls_settings_get_stepper_random_max(void)
{
    return _ls_settings_stepper_random_max;
}

void ls_settings_set_light_threshold_on(BaseType_t adc)
{
    _ls_settings_light_threshold_on = _constrain(adc, 0, 4095);
}
BaseType_t ls_settings_get_light_threshold_on(void)
{
    return _ls_settings_light_threshold_on;
}

void ls_settings_set_light_threshold_off(BaseType_t adc)
{
    _ls_settings_light_threshold_off = _constrain(adc, 0, 4095);
}
BaseType_t ls_settings_get_light_threshold_off(void)
{
    return _ls_settings_light_threshold_off;
}