#include "settings.h"
#include "config.h"

static BaseType_t _ls_settings_speed, _ls_settings_top_angle, _ls_settings_bottom_angle;

static BaseType_t _map(BaseType_t x, BaseType_t in_min, BaseType_t in_max, BaseType_t out_min, BaseType_t out_max)
{
  if ((in_max - in_min) > (out_max - out_min)) {
    return (x - in_min) * (out_max - out_min+1) / (in_max - in_min+1) + out_min;
  }
  else
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}

static BaseType_t _constrain(BaseType_t x, BaseType_t min, BaseType_t max)
{
    if (x < min) { return min; }
    if (x > max) { return max; }
    return x;
}

void ls_settings_set_defaults(void)
{

}

void ls_settings_read_from_flash(void)
{
    ;
}

void ls_settings_save_to_flash(void)
{
    ;
}

void ls_settings_set_speed(BaseType_t steps_per_second)
{
    _ls_settings_speed = steps_per_second;
}
BaseType_t ls_settings_get_speed(void)
{
    return _ls_settings_speed;
}

void ls_settings_set_top_angle(BaseType_t);
BaseType_t ls_settings_get_top_angle(void);

void ls_settings_set_bottom_range(BaseType_t);
BaseType_t ls_settings_get_bottom_angle(void);