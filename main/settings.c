#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "settings.h"
#include "config.h"
#include "util.h"

static BaseType_t _ls_settings_stepper_speed, _ls_settings_servo_top, _ls_settings_servo_bottom;
static BaseType_t _ls_settings_stepper_random_max, _ls_settings_light_threshold_on, _ls_settings_light_threshold_off;

static nvs_handle_t _ls_settings_nvs_handle;
// caution: NVS keys and namespaces are restricted to 15 characters
#define LS_SETTINGS_NVS_NAMESPACE "ls_settings"
#define LS_SETTINGS_NVS_KEY_STEPPER_SPEED "stepper_speed"
#define LS_SETTINGS_NVS_KEY_SERVO_TOP "servo_top"
#define LS_SETTINGS_NVS_KEY_SERVO_BOTTOM "servo_bottom"

void ls_settings_set_defaults(void)
{
    ls_settings_set_stepper_speed(LS_STEPPER_STEPS_PER_SECOND_DEFAULT);
    ls_settings_set_servo_top(LS_SERVO_US_MIN);
    ls_settings_set_servo_bottom(LS_SERVO_US_MAX);

    ls_settings_set_stepper_random_max(LS_STEPPER_MOVEMENT_STEPS_MAX);
    ls_settings_set_light_threshold_on(LS_LIGHTSENSE_DAY_THRESHOLD);
    ls_settings_set_light_threshold_off(LS_LIGHTSENSE_NIGHT_THRESHOLD);
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
    printf("\nOpening Non-Volatile Storage (NVS) handle... ");
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
    // // preserve fixed angle: if the top is equal to the bottom, change the bottom to match
    // if (_ls_settings_servo_top == _ls_settings_servo_bottom)
    // {
    //     _ls_settings_servo_bottom = _constrain(microseconds, LS_SERVO_US_MIN, LS_SERVO_US_MAX);
    // }
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