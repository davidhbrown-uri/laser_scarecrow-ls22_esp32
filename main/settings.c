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
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "settings.h"
#include "config.h"
#include "util.h"

static BaseType_t _ls_settings_stepper_speed, _ls_settings_servo_top, _ls_settings_servo_bottom;
static BaseType_t _ls_settings_stepper_random_max, _ls_settings_light_threshold_on, _ls_settings_light_threshold_off;
static BaseType_t _ls_settings_servo_pulse_delta, _ls_settings_servo_random_pause_ms, _ls_settings_servo_sweep_pause_ms;
static BaseType_t _ls_settings_tilt_threshold_detected, _ls_settings_tilt_threshold_ok, _ls_settings_minimum_rpm;
static bool _ls_settings_sleep_light_enable;

static nvs_handle_t _ls_settings_nvs_handle;
// caution: NVS keys and namespaces are restricted to 15 characters
#define LS_SETTINGS_NVS_NAMESPACE "ls_settings"
#define LS_SETTINGS_NVS_KEY_STEPPER_SPEED "stepper_speed"
#define LS_SETTINGS_NVS_KEY_SERVO_TOP "servo_top"
#define LS_SETTINGS_NVS_KEY_SERVO_BOTTOM "servo_bottom"
#define LS_SETTINGS_NVS_KEY_SERVO_DELTA "servo_delta"
#define LS_SETTINGS_NVS_KEY_LIGHTSENSE_ON "light_on"
#define LS_SETTINGS_NVS_KEY_LIGHTSENSE_OFF "light_off"
#define LS_SETTINGS_NVS_KEY_SLEEP_LIGHT_ENABLE "sleep_light"
#define LS_SETTINGS_NVS_KEY_MINIMUM_RPM "min_rpm"

void ls_settings_set_defaults(void)
{
    ls_settings_set_stepper_speed(LS_STEPPER_STEPS_PER_SECOND_DEFAULT);
    ls_settings_set_servo_top(LS_SERVO_US_MIN); // all the way at the top
    ls_settings_set_servo_bottom(100);          // all the way to the bottom

    ls_settings_set_stepper_random_max(LS_STEPPER_RANDOM_HOP_STEPS_MAX);
    ls_settings_set_light_threshold_on((int)((int[]){LS_LIGHTSENSE_THRESHOLDS_ON_MV})[LS_LIGHTSENSE_THRESHOLD_DEFAULT]);
    ls_settings_set_light_threshold_off((int)((int[]){LS_LIGHTSENSE_THRESHOLDS_OFF_MV})[LS_LIGHTSENSE_THRESHOLD_DEFAULT]);

    ls_settings_set_servo_random_pause_ms(LS_SERVO_RANDOM_PAUSE_MS);
    ls_settings_set_servo_sweep_pause_ms(LS_SERVO_SWEEP_PAUSE_MS);
    ls_settings_set_servo_pulse_delta(LS_SERVO_DELTA_PER_TICK_DEFAULT);

    ls_settings_set_tilt_threshold_mg_detected(LS_TILT_THRESHOLD_DETECTED_MG);
    ls_settings_set_tilt_threshold_mg_ok(LS_TILT_THRESHOLD_OK_MG);

    ls_settings_set_sleep_light_enable(LS_SETTINGS_SLEEP_LIGHT_ENABLE_DEFAULT);

    ls_settings_set_minimum_rpm(LS_SETTINGS_MINIMUM_RPM_DEFAULT);
}

void ls_settings_reset_defaults(void)
{
    ls_settings_set_defaults();
    ls_settings_save();
}

static void _ls_settings_open_nvs(void)
{
    // https://github.com/espressif/esp-idf/blob/079b5b1857242d6288478c62dc36a843718882e9/examples/storage/nvs_rw_value/main/nvs_value_example_main.c
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open
    err = nvs_open(LS_SETTINGS_NVS_NAMESPACE, NVS_READWRITE, &_ls_settings_nvs_handle);
#ifdef LSDEBUG_SETTINGS
    if (err != ESP_OK)
    {
        ls_debug_printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        ls_debug_printf("NVS Settings storage opened\n");
    }
#endif
}

/**
 * @brief calls both nvs_commit() and nvs_close()
 *
 */
static void _ls_settings_close_nvs(void)
{
    printf((nvs_commit(_ls_settings_nvs_handle) != ESP_OK) ? "NVS commit FAILED!\n" : "NVS commit done\n");
    // Close
    nvs_close(_ls_settings_nvs_handle);
}

static esp_err_t _ls_settings_read_from_nvs(char *name, int32_t *value_pointer)
{
    // https://github.com/espressif/esp-idf/blob/079b5b1857242d6288478c62dc36a843718882e9/examples/storage/nvs_rw_value/main/nvs_value_example_main.c
    esp_err_t err = nvs_get_i32(_ls_settings_nvs_handle, name, value_pointer);
#ifdef LSDEBUG_SETTINGS
    switch (err)
    {
    case ESP_OK:
        ls_debug_printf("NVS found %s = %d\n", name, *value_pointer);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ls_debug_printf("NVS has no value for %s\n", name);
        break;
    default:
        ls_debug_printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
#endif
    return err;
}

void ls_settings_read(void)
{
    int32_t nvs_value;
#ifdef LSDEBUG_SETTINGS
    ls_debug_printf("Reading settings from NVS\n");
#endif
    _ls_settings_open_nvs();
    if (ESP_OK == _ls_settings_read_from_nvs(LS_SETTINGS_NVS_KEY_STEPPER_SPEED, &nvs_value))
    {
        ls_settings_set_stepper_speed(nvs_value);
    }
    if (ESP_OK == _ls_settings_read_from_nvs(LS_SETTINGS_NVS_KEY_SERVO_TOP, &nvs_value))
    {
        ls_settings_set_servo_top(nvs_value);
    }
    if (ESP_OK == _ls_settings_read_from_nvs(LS_SETTINGS_NVS_KEY_SERVO_BOTTOM, &nvs_value))
    {
        ls_settings_set_servo_bottom(nvs_value);
    }
    if (ESP_OK == _ls_settings_read_from_nvs(LS_SETTINGS_NVS_KEY_SERVO_DELTA, &nvs_value))
    {
        ls_settings_set_servo_pulse_delta(nvs_value);
    }
    if (ESP_OK == _ls_settings_read_from_nvs(LS_SETTINGS_NVS_KEY_LIGHTSENSE_OFF, &nvs_value))
    {
        ls_settings_set_light_threshold_off(nvs_value);
    }
    if (ESP_OK == _ls_settings_read_from_nvs(LS_SETTINGS_NVS_KEY_LIGHTSENSE_ON, &nvs_value))
    {
        ls_settings_set_light_threshold_on(nvs_value);
    }
    if (ESP_OK == _ls_settings_read_from_nvs(LS_SETTINGS_NVS_KEY_SLEEP_LIGHT_ENABLE, &nvs_value))
    {
        ls_settings_set_sleep_light_enable((bool)nvs_value);
    }
    if (ESP_OK == _ls_settings_read_from_nvs(LS_SETTINGS_NVS_KEY_SLEEP_LIGHT_ENABLE, &nvs_value))
    {
        ls_settings_set_sleep_light_enable((bool)nvs_value);
    }
    _ls_settings_close_nvs();
}

void ls_settings_save(void)
{
#ifdef LSDEBUG_SETTINGS
    ls_debug_printf("Saving settings to NVS\n");
#endif
    _ls_settings_open_nvs();
    if (ESP_OK == nvs_set_i32(_ls_settings_nvs_handle, LS_SETTINGS_NVS_KEY_STEPPER_SPEED,
                              ls_settings_get_stepper_speed()))
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings saved stepper speed=%d\n", ls_settings_get_stepper_speed());
#endif
    }
    else
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings could not save stepper speed\n");
#endif
    }
    if (ESP_OK == nvs_set_i32(_ls_settings_nvs_handle, LS_SETTINGS_NVS_KEY_SERVO_TOP,
                              ls_settings_get_servo_top()))
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings saved servo_top=%d\n", ls_settings_get_servo_top());
#endif
    }
    else
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings could not save servo top\n");
#endif
    }
    if (ESP_OK == nvs_set_i32(_ls_settings_nvs_handle, LS_SETTINGS_NVS_KEY_SERVO_BOTTOM,
                              ls_settings_get_servo_bottom()))
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings saved servo_bottom=%d\n", ls_settings_get_servo_bottom());
#endif
    }
    else
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings could not save servo bottom\n");
#endif
    }
    if (ESP_OK == nvs_set_i32(_ls_settings_nvs_handle, LS_SETTINGS_NVS_KEY_SERVO_DELTA,
                              ls_settings_get_servo_pulse_delta()))
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings saved servo pulse delta=%d\n", ls_settings_get_servo_pulse_delta());
#endif
    }
    else
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings could not save servo pulse delta.\n");
#endif
    }
    if (ESP_OK == nvs_set_i32(_ls_settings_nvs_handle, LS_SETTINGS_NVS_KEY_LIGHTSENSE_OFF,
                              ls_settings_get_light_threshold_off()))
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings saved light threshold OFF=%d\n", ls_settings_get_light_threshold_off());
#endif
    }
    else
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings could not save light threshold OFF.\n");
#endif
    }
    if (ESP_OK == nvs_set_i32(_ls_settings_nvs_handle, LS_SETTINGS_NVS_KEY_LIGHTSENSE_ON,
                              ls_settings_get_light_threshold_on()))
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings saved light threshold ON=%d\n", ls_settings_get_light_threshold_on());
#endif
    }
    else
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings could not save light threshold on.\n");
#endif
    }
    if (ESP_OK == nvs_set_i32(_ls_settings_nvs_handle, LS_SETTINGS_NVS_KEY_SLEEP_LIGHT_ENABLE,
                              (int32_t) ls_settings_is_sleep_light_enabled()))
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings saved sleep light enabled=%d\n", (int32_t) ls_settings_is_sleep_light_enabled());
#endif
    }
    else
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings could not save sleep light enabled.\n");
#endif
    }
    if (ESP_OK == nvs_set_i32(_ls_settings_nvs_handle, LS_SETTINGS_NVS_KEY_MINIMUM_RPM,
                              (int32_t) ls_settings_get_minimum_rpm()))
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings saved minimum RPM=%d\n", (int32_t) ls_settings_get_minimum_rpm());
#endif
    }
    else
    {
        ;
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Settings could not save minimum RPM.\n");
#endif
    }
    _ls_settings_close_nvs();
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
#ifdef LSDEBUG_SETTINGS
    ls_debug_printf("Setting stepper_speed = %d\n", _ls_settings_stepper_speed);
#endif
}

BaseType_t ls_settings_get_stepper_speed(void)
{
    return _ls_settings_stepper_speed;
}

BaseType_t ls_settings_map_control_to_servo_top(BaseType_t adc)
{
    return _map(_constrain(adc, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP),
                LS_CONTROLS_READING_TOP, LS_CONTROLS_READING_BOTTOM, // yes, inverted!
                LS_SERVO_US_MIN, LS_SERVO_US_MAX);
}
void ls_settings_set_servo_top(BaseType_t microseconds)
{
    _ls_settings_servo_top = _constrain(microseconds, LS_SERVO_US_MIN, LS_SERVO_US_MAX);
//    _ls_settings_servo_bottom = _constrain(_ls_settings_servo_bottom, _ls_settings_servo_top, LS_SERVO_US_MAX);
#ifdef LSDEBUG_SETTINGS
    ls_debug_printf("Setting servo_top = %d\n", _ls_settings_servo_top);
#endif
}
BaseType_t ls_settings_get_servo_top(void)
{
    return _ls_settings_servo_top;
}

BaseType_t ls_settings_map_control_to_servo_bottom(BaseType_t adc)
{
    return _map(_constrain(adc, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP),
                LS_CONTROLS_READING_TOP, LS_CONTROLS_READING_BOTTOM, // yes, inverted!
                0, 100);
}
void ls_settings_set_servo_bottom(BaseType_t percent)
{
    _ls_settings_servo_bottom = _constrain(percent, 0, 100);
#ifdef LSDEBUG_SETTINGS
    ls_debug_printf("Setting servo_bottom = %d%%\n", _ls_settings_servo_bottom);
#endif
}
BaseType_t ls_settings_get_servo_bottom(void)
{
    return _ls_settings_servo_bottom;
}

void ls_settings_set_stepper_random_max(BaseType_t steps)
{
    _ls_settings_stepper_random_max = _constrain(steps, LS_STEPPER_RANDOM_HOP_STEPS_MIN, LS_STEPPER_RANDOM_HOP_STEPS_MAX);
}
BaseType_t ls_settings_get_stepper_random_max(void)
{
    return _ls_settings_stepper_random_max;
}

void ls_settings_set_light_threshold_on(int mV)
{
    _ls_settings_light_threshold_on = _constrain(mV, 0, 3300);
}
BaseType_t ls_settings_get_light_threshold_on(void)
{
    return _ls_settings_light_threshold_on;
}

void ls_settings_set_light_threshold_off(int mV)
{
    _ls_settings_light_threshold_off = _constrain(mV, 0, 3300);
}
BaseType_t ls_settings_get_light_threshold_off(void)
{
    return _ls_settings_light_threshold_off;
}

BaseType_t ls_settings_map_control_to_servo_pulse_delta(BaseType_t adc)
{
    adc = _make_log_response(
        _map(_constrain(adc, LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP),
             LS_CONTROLS_READING_BOTTOM, LS_CONTROLS_READING_TOP,
             0, 2047),
        11);
    return _map(adc, 0, 2047, LS_SERVO_DELTA_PER_TICK_MIN, LS_SERVO_DELTA_PER_TICK_MAX);
}
void ls_settings_set_servo_pulse_delta(BaseType_t microseconds_per_tick)
{
    _ls_settings_servo_pulse_delta = _constrain(microseconds_per_tick, LS_SERVO_DELTA_PER_TICK_MIN, LS_SERVO_DELTA_PER_TICK_MAX);
}
BaseType_t ls_settings_get_servo_pulse_delta(void)
{
    return _ls_settings_servo_pulse_delta;
}

void ls_settings_set_servo_random_pause_ms(BaseType_t duration_ms)
{
    _ls_settings_servo_random_pause_ms = _constrain(duration_ms, 0, 10000);
}
BaseType_t ls_settings_get_servo_random_pause_ms(void)
{
    return _ls_settings_servo_random_pause_ms;
}
void ls_settings_set_servo_sweep_pause_ms(BaseType_t duration_ms)
{
    _ls_settings_servo_sweep_pause_ms = _constrain(duration_ms, 0, 10000);
}
BaseType_t ls_settings_get_servo_sweep_pause_ms(void)
{
    return _ls_settings_servo_sweep_pause_ms;
}

void ls_settings_set_tilt_threshold_mg_detected(BaseType_t milli_gs)
{
    _ls_settings_tilt_threshold_detected = milli_gs;
}
BaseType_t ls_settings_get_tilt_threshold_mg_detected(void)
{
    return _ls_settings_tilt_threshold_detected;
}

void ls_settings_set_tilt_threshold_mg_ok(BaseType_t milli_gs)
{
    _ls_settings_tilt_threshold_ok = milli_gs;
}
BaseType_t ls_settings_get_tilt_threshold_mg_ok(void)
{
    return _ls_settings_tilt_threshold_ok;
}

void ls_settings_set_sleep_light_enable(bool enabled)
{
    _ls_settings_sleep_light_enable = enabled;
}
bool ls_settings_is_sleep_light_enabled(void)
{
    return _ls_settings_sleep_light_enable;
}

void ls_settings_set_minimum_rpm(BaseType_t rpm)
{
    _ls_settings_minimum_rpm = rpm;
}
BaseType_t ls_settings_get_minimum_rpm(void)
{
    return _ls_settings_minimum_rpm;
}
