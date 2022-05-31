#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "settings.h"
#include "config.h"
#include "util.h"

static BaseType_t _ls_settings_stepper_speed, _ls_settings_servo_top, _ls_settings_servo_bottom;
static BaseType_t _ls_settings_stepper_random_max, _ls_settings_light_threshold_on, _ls_settings_light_threshold_off;
static BaseType_t _ls_settings_servo_pulse_delta, _ls_settings_servo_random_pause_ms, _ls_settings_servo_sweep_pause_ms;
static BaseType_t _ls_settings_tilt_threshold_detected, _ls_settings_tilt_threshold_ok;

static nvs_handle_t _ls_settings_nvs_handle;
// caution: NVS keys and namespaces are restricted to 15 characters
#define LS_SETTINGS_NVS_NAMESPACE "ls_settings"
#define LS_SETTINGS_NVS_KEY_STEPPER_SPEED "stepper_speed"
#define LS_SETTINGS_NVS_KEY_SERVO_TOP "servo_top"
#define LS_SETTINGS_NVS_KEY_SERVO_BOTTOM "servo_bottom"
#define LS_SETTINGS_NVS_KEY_SERVO_DELTA "servo_delta"
#define LS_SETTINGS_NVS_KEY_LIGHTSENSE_DAY "light_day"
#define LS_SETTINGS_NVS_KEY_LIGHTSENSE_NIGHT "light_night"

void ls_settings_set_defaults(void)
{
    ls_settings_set_stepper_speed(LS_STEPPER_STEPS_PER_SECOND_DEFAULT);
    ls_settings_set_servo_top(LS_SERVO_US_MIN);
    ls_settings_set_servo_bottom(LS_SERVO_US_MAX);

    ls_settings_set_stepper_random_max(LS_STEPPER_MOVEMENT_STEPS_MAX);
    ls_settings_set_light_threshold_on(LS_LIGHTSENSE_DAY_THRESHOLD);
    ls_settings_set_light_threshold_off(LS_LIGHTSENSE_NIGHT_THRESHOLD);

    ls_settings_set_servo_random_pause_ms(LS_SERVO_RANDOM_PAUSE_MS);
    ls_settings_set_servo_sweep_pause_ms(LS_SERVO_SWEEP_PAUSE_MS);
    ls_settings_set_servo_pulse_delta(LS_SERVO_DELTA_PER_TICK_DEFAULT);

    ls_settings_set_tilt_threshold_mg_detected(LS_TILT_THRESHOLD_DETECTED_MG);
    ls_settings_set_tilt_threshold_mg_ok(LS_TILT_THRESHOLD_OK_MG);
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
    _ls_settings_servo_bottom = _constrain(_ls_settings_servo_bottom, _ls_settings_servo_top, LS_SERVO_US_MAX);
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
                ls_settings_get_servo_top(), LS_SERVO_US_MAX);
}
void ls_settings_set_servo_bottom(BaseType_t microseconds)
{
    _ls_settings_servo_bottom = _constrain(microseconds, ls_settings_get_servo_top(), LS_SERVO_US_MAX);
#ifdef LSDEBUG_SETTINGS
    ls_debug_printf("Setting servo_bottom = %d\n", _ls_settings_servo_bottom);
#endif
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

void ls_settings_set_light_thresholds_from_0to10(int index)
{
    if (index < 0 || index > 10)
    {
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Ignoring invalid lightsense threshold index %d must be 0..10 inclusive.\n", index);
#endif
        return;
    }
#ifdef LSDEBUG_SETTINGS
        ls_debug_printf("Setting lightsense thresholds from index value %d (range 0..10 inclusive).\n", index);
#endif
    // off will range from 0 to 2010; 5 => 505 (near default)
    ls_settings_set_light_threshold_off(20 * (index * index) + (1 * index) + 0);
    // off will range from 50 to 4070; 5 => 1060 (near default)
    ls_settings_set_light_threshold_on(40 * (index * index) + (2 * index) + 50);
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