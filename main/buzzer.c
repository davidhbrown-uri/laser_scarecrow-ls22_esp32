#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "config.h"
#include "buzzer.h"

#define BUZZER_SPEED (LEDC_HIGH_SPEED_MODE)
#define BUZZER_TIMER (LEDC_TIMER_0)
#define BUZZER_CLOCK (LEDC_AUTO_CLK)
#define BUZZER_RESOLUTION (LEDC_TIMER_1_BIT)
#define BUZZER_CHANNEL (LEDC_CHANNEL_0)
#define BUZZER_DUTY (1)

static void _buzzer_frequency(uint32_t freq)
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

void buzzer_init(void)
{
    ls_buzzer_queue = xQueueCreate(32, sizeof(enum ls_buzzer_effects));
}

void buzzer_play(enum ls_buzzer_effects effect)
{
    xQueueSend(ls_buzzer_queue, (void *)&effect, 0); // don't block if queue full
}

static void buzzer_effect_click(void)
{
    _buzzer_frequency(500);
     vTaskDelay(1);
}

static void buzzer_effect_alternate_high(void)
{
    for(int i = 0; i < pdMS_TO_TICKS(1000); i+=2)
    {
    _buzzer_frequency(3100); // resonant frequency of bucket piezo
    vTaskDelay(1);
    _buzzer_frequency(4000); // resonant frequency of knobs pizo   
    vTaskDelay(1);
    }
}

void buzzer_handler_task(void *pvParameter)
{
    enum ls_buzzer_effects received;
    while (1)
    {
        if (xQueueReceive(ls_buzzer_queue, &received, portMAX_DELAY) != pdTRUE)
        {
            printf("No buzz requested maximum delay... getting very bored.");
        }
        else
        {
            switch (received)
            {
            case LS_BUZZER_CLICK:
                printf("Buzzer Click\n");
                buzzer_effect_click();
                break;
            case LS_BUZZER_ALERT_1S:
                printf("Buzzer Alert 1s\n");
                _buzzer_frequency(3100);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            case LS_BUZZER_ALTERNATE_HIGH:
                printf("Buzzer Alternate High\n");
                buzzer_effect_alternate_high();
                break;
            default:
                printf("Unknown ls_buzzer_effect %d -- I'm confused", received);
            }
        }
        ESP_ERROR_CHECK(ledc_stop(BUZZER_SPEED, BUZZER_CHANNEL, 0));
    }
}
