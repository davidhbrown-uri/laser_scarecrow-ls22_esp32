#include "settings.h"
#include "config.h"
#include "util.h"

static BaseType_t _ls_settings_stepper_speed, _ls_settings_servo_top, _ls_settings_servo_bottom;

void ls_settings_set_defaults(void)
{
    ls_settings_set_stepper_speed(LS_STEPPER_STEPS_PER_SECOND_DEFAULT);
    ls_settings_set_servo_top(LS_SERVO_US_MIN);
}

void ls_settings_read_from_flash(void)
{
    ;
}

void ls_settings_save_to_flash(void)
{
    ;
}
BaseType_t ls_settings_map_control_to_servo_speed(BaseType_t adc)
{
    return _map(_constrain(adc, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP),
        LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP,
        LS_STEPPER_STEPS_PER_SECOND_MIN, LS_STEPPER_STEPS_PER_SECOND_MAX
        );
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
    _ls_settings_servo_top = _constrain(microseconds, LS_SERVO_US_MIN, LS_SERVO_US_MAX);
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