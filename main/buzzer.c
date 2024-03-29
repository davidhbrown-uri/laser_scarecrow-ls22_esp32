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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "config.h"
#include "buzzer.h"
#include "debug.h"
#include "events.h"
#include "util.h"

#define BUZZER_SPEED (LEDC_HIGH_SPEED_MODE)
#define BUZZER_TIMER (LEDC_TIMER_0)
#define BUZZER_CLOCK (LEDC_AUTO_CLK)
#define BUZZER_RESOLUTION (LEDC_TIMER_1_BIT)
#define BUZZER_CHANNEL (LEDC_CHANNEL_0)
#define BUZZER_DUTY (1)

bool _ls_buzzer_in_use = false;
#define LS_BUZZER_REQUEST_DEFAULT_FREQUENCY 1000
#define LS_BUZZER_REQUEST_DEFAULT_TICKS 1

struct ls_buzzer_request_t {
    enum ls_buzzer_effects effect;
    BaseType_t frequency;
    TickType_t ticks;
}ls_buzzer_request_t;

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
    ls_buzzer_queue = xQueueCreate(32, sizeof(ls_buzzer_request_t));
}

void ls_buzzer_effect(enum ls_buzzer_effects effect)
{
    struct ls_buzzer_request_t request;
    request.effect = effect;
    request.frequency = LS_BUZZER_REQUEST_DEFAULT_FREQUENCY;
    request.ticks = LS_BUZZER_REQUEST_DEFAULT_TICKS;
    xQueueSend(ls_buzzer_queue, (void *)&request, 0); // don't block if queue full
}

bool ls_buzzer_in_use(void)
{
    return (_ls_buzzer_in_use || (uxQueueMessagesWaiting(ls_buzzer_queue) > 0));
}


static void _ls_buzzer_effect_click(int frequency)
{
#ifdef LSDEBUG_BUZZER
    ls_debug_printf("Buzzer Click at %dHz\n", frequency);
#endif
    _ls_buzzer_frequency(frequency);
    vTaskDelay(1);
}

static void _ls_buzzer_play_note(enum ls_buzzer_scale note, int ms_duration)
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
        _ls_buzzer_frequency(4000); // resonant frequency of external control piezo
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
    struct ls_buzzer_request_t received;
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
            switch (received.effect)
            {
            case LS_BUZZER_CLICK:
                _ls_buzzer_effect_click(500);
                break;
            case LS_BUZZER_CLICK_HIGH:
                _ls_buzzer_effect_click(600);
                break;
            case LS_BUZZER_ALERT_1S:
#ifdef LSDEBUG_BUZZER
                ls_debug_printf("Buzzer Alert 1s\n");
#endif
                _ls_buzzer_frequency(1000);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            case LS_BUZZER_PLAY_TONE:
                _ls_buzzer_frequency(received.frequency);
                vTaskDelay(received.ticks);
                ESP_ERROR_CHECK(ledc_stop(BUZZER_SPEED, BUZZER_CHANNEL, 0));
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
            case LS_BUZZER_PLAY_SETTINGS_CONTROL_ENTER:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_G, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_A, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_B, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                break;
            case LS_BUZZER_PLAY_SETTINGS_CONTROL_LEAVE:
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
            case LS_BUZZER_PLAY_WAKE:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_D, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_E, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_G, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_A, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_B, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                break;
            case LS_BUZZER_PLAY_SLEEP:
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_B, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_A, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_G, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_F, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_E, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_D, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_CC, 100);
                _ls_buzzer_play_note(LS_BUZZER_SCALE_C, 100);
                break;
            case LS_BUZZER_PLAY_NOTHING:
                ESP_ERROR_CHECK(ledc_stop(BUZZER_SPEED, BUZZER_CHANNEL, 0));
                vTaskDelay(1);
                break;
            default:;
#ifdef LSDEBUG_BUZZER
                ls_debug_printf("Unknown ls_buzzer_effect %d -- I'm confused", received.effect);
#endif
            }
        }
        ESP_ERROR_CHECK(ledc_stop(BUZZER_SPEED, BUZZER_CHANNEL, 0));
        _ls_buzzer_in_use = false;
    }
}
void ls_buzzer_note(enum ls_buzzer_scale note, TickType_t ticks)
{
    struct ls_buzzer_request_t request;
    request.effect = LS_BUZZER_PLAY_TONE;
    request.frequency = _constrain((BaseType_t) note, 500, 22000);
    request.ticks = ticks;
    xQueueSend(ls_buzzer_queue, (void *)&request, 0); // don't block if queue full
};

void ls_buzzer_tone(BaseType_t frequency_hz)
{
    struct ls_buzzer_request_t request;
    request.effect = LS_BUZZER_PLAY_TONE;
    request.frequency = (BaseType_t) _constrain(frequency_hz, 500, 22000);;
    request.ticks = LS_BUZZER_REQUEST_DEFAULT_TICKS;
    xQueueSend(ls_buzzer_queue, (void *)&request, 0); // don't block if queue full
}
