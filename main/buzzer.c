#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "config.h"
#include "buzzer.h"

void buzzer_init(void)
{
    ls_buzzer_queue = xQueueCreate( 32, sizeof(enum ls_buzzer_effects));
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 10000, //10kHz 
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 512, // half of timer resolution, 50% duty cycle
        .gpio_num = LSGPIO_BUZZERENABLE,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);
    ledc_timer_pause(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);
}

void buzzer_play(enum ls_buzzer_effects effect)
{
    xQueueSend(ls_buzzer_queue, (void *) &effect, 0); // don't block if queue full
}

static void buzzer_effect_click(void)
{
    // 500Hz for 100ms
    ESP_ERROR_CHECK(ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, 500));
    ESP_ERROR_CHECK(ledc_timer_resume(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(ledc_timer_pause(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0));
}

void buzzer_handler_task(void *pvParameter)
{
    enum ls_buzzer_effects received;
    while(1)
    {
        if(xQueueReceive(ls_buzzer_queue, &received, portMAX_DELAY) != pdTRUE)
        {
            printf("No buzz requested maximum delay... getting very bored.");
        }
        else {
            switch (received) {
                case LS_BUZZER_CLICK:
                    printf("Click\n");
                    buzzer_effect_click();
                    break;
                default:
                    printf("Unknown ls_buzzer_effect %d -- I'm confused", received);
            }
        }
    }
}

