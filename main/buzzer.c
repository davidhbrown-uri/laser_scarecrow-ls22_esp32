#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "config.h"
#include "buzzer.h"
#include "debug.h"
#include "events.h"

#define BUZZER_SPEED (LEDC_HIGH_SPEED_MODE)
#define BUZZER_TIMER (LEDC_TIMER_0)
#define BUZZER_CLOCK (LEDC_AUTO_CLK)
#define BUZZER_RESOLUTION (LEDC_TIMER_1_BIT)
#define BUZZER_CHANNEL (LEDC_CHANNEL_0)
#define BUZZER_DUTY (1)

bool _ls_buzzer_in_use = false;

static enum _ls_buzzer_scale {
    LS_BUZZER_SCALE_bb = 967,  // b
    LS_BUZZER_SCALE_C = 1024,  // C
    LS_BUZZER_SCALE_D = 1149,  // D
    LS_BUZZER_SCALE_E = 1289,  // E
    LS_BUZZER_SCALE_F = 1367,  // F
    LS_BUZZER_SCALE_G = 1534,  // G
    LS_BUZZER_SCALE_A = 1722,  // A
    LS_BUZZER_SCALE_B = 1933,  // B
    LS_BUZZER_SCALE_CC = 2048, // C'
};

static void _ls_buzzer_frequency(uint32_t freq)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = BUZZER_SPEED,
        .duty_resolution = BUZZER_RESOLUTION,
        .freq_hz = freq,
        .timer_num = BUZZER_TIMER,
        .clk_cfg = BUZZER_CLOCK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
        .channel = BUZZER_CHANNEL,
        .duty = BUZZER_DUTY, // half of 2**duty_resolution, 50% duty cycle
        .gpio_num = LSGPIO_BUZZERENABLE,
        .speed_mode = BUZZER_SPEED,
        .timer_sel = BUZZER_TIMER,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void ls_buzzer_init(void)
{
    ls_buzzer_queue = xQueueCreate(32, sizeof(enum ls_buzzer_effects));
}

void ls_buzzer_play(enum ls_buzzer_effects effect)
{
    xQueueSend(ls_buzzer_queue, (void *)&effect, 0); // don't block if queue full
}

bool ls_buzzer_in_use(void)
{
    return (_ls_buzzer_in_use || (uxQueueMessagesWaiting(ls_buzzer_queue) > 0));
}

static void _ls_buzzer_effect_click(void)
{
#ifdef LSDEBUG_BUZZER
    ls_debug_printf("Buzzer Click\n");
#endif
    _ls_buzzer_frequency(500);
    vTaskDelay(1);
}

static void _ls_buzzer_play_note(enum _ls_buzzer_scale note, int ms_duration)
{
    _ls_buzzer_frequency((uint32_t)note);
    vTaskDelay(pdMS_TO_TICKS(ms_duration));
    ESP_ERROR_CHECK(ledc_stop(BUZZER_SPEED, BUZZER_CHANNEL, 0));
}

static void _ls_buzzer_effect_alternate_high(int duration_ms)
{
#ifdef LSDEBUG_BUZZER
    ls_debug_printf("Buzzer Alternate High\n");
#endif
    for (int i = 0; i < pdMS_TO_TICKS(duration_ms); i += 2)
    {
        _ls_buzzer_frequency(3100); // resonant frequency of bucket piezo
        vTaskDelay(1);
        _ls_buzzer_frequency(4000); // resonant frequency of knobs pizo
        vTaskDelay(1);
    }
    ESP_ERROR_CHECK(ledc_stop(BUZZER_SPEED, BUZZER_CHANNEL, 0));
}

static void _ls_buzzer_pre_laser_warning(void)
{
#ifdef LSDEBUG_BUZZER
    ls_debug_printf("Begin pre-laser warning\n");
#endif
    for (int i = 0; i < 2; i++)
    {
        _ls_buzzer_effect_alternate_high(1500);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    for (int i = 0; i < 4; i++)
    {
        _ls_buzzer_effect_alternate_high(500 - i * 100);
        vTaskDelay(pdMS_TO_TICKS(500 - i * 100));
    }
    ls_event event;
    event.type = LSEVT_BUZZER_WARNING_COMPLETE;
    event.value = NULL;
    xQueueSendToBack(ls_event_queue, (void *)&event, pdMS_TO_TICKS(10000));
}

void ls_buzzer_handler_task(void *pvParameter)
{
    enum ls_buzzer_effects received;
    while (1)
    {
        if (xQueueReceive(ls_buzzer_queue, &received, portMAX_DELAY) != pdTRUE)
        {
#ifdef LSDEBUG_BUZZER
            ls_debug_printf("No buzz requested maximum delay... getting very bored.\n");
#endif
        }
        else
        {
            _ls_buzzer_in_use = true;
            switch (received)
            {
            case LS_BUZZER_CLICK:
                _ls_buzzer_effect_click();
                break;
            case LS_BUZZER_ALERT_1S:
#ifdef LSDEBUG_BUZZER
                ls_debug_printf("Buzzer Alert 1s\n");
#endif
                _ls_buzzer_frequency(3100);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            case LS_BUZZER_ALTERNATE_HIGH:
                _ls_buzzer_effect_alternate_high(1000);
                break;
            case LS_BUZZER_PRE_LASER_WARNING:
                _ls_buzzer_pre_laser_warning();
                break;
            case LS_BUZZER_PLAY_TAPE_ENABLE:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_B, 200);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 200);
                break;
            case LS_BUZZER_PLAY_TAPE_DISABLE:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_D, 200);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 200);
                break;
            case LS_BUZZER_PLAY_TAPE_MISREAD:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_B, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                vTaskDelay(pdMS_TO_TICKS(200)); // rest
                break;
            case LS_BUZZER_PLAY_HOME_SUCCESS:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_E, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_G, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 300);
                vTaskDelay(pdMS_TO_TICKS(300)); // rest
                break;
            case LS_BUZZER_PLAY_HOME_FAIL:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_A, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_A, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_D, 400);
                vTaskDelay(pdMS_TO_TICKS(300)); // rest
                break;
            case LS_BUZZER_PLAY_MAP_FAIL:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_A, 200);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 300);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_D, 400);
                vTaskDelay(pdMS_TO_TICKS(500)); // rest
                break;
            case LS_BUZZER_PLAY_TILT_FAIL:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_B, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_A, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_G, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_E, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_D, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_bb, 400);
                vTaskDelay(pdMS_TO_TICKS(500)); // rest
                break;
            case LS_BUZZER_PLAY_MANUAL_CONTROL_ENTER:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_G, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_A, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_B, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                break;
            case LS_BUZZER_PLAY_MANUAL_CONTROL_LEAVE:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_B, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_A, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_G, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                break;
                case LS_BUZZER_PLAY_ROOT:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 200);
                break;
                case LS_BUZZER_PLAY_OCTAVE:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 200);
                break;
            default:;
#ifdef LSDEBUG_BUZZER
                ls_debug_printf("Unknown ls_buzzer_effect %d -- I'm confused", received);
#endif
            }
        }
        ESP_ERROR_CHECK(ledc_stop(BUZZER_SPEED, BUZZER_CHANNEL, 0));
        _ls_buzzer_in_use = false;
    }
}
