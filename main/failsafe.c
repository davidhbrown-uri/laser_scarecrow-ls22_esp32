#include "failsafe.h"
#include "config.h"
#include "events.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "freertos/queue.h"


void IRAM_ATTR _ls_failsafe_heartbeat_isr(void *pvParameter)
{
    timer_set_counter_value(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, 0ULL);
}

void _ls_failsafe_alarm_callback(void)
{
    gpio_set_level(LSGPIO_LASERPOWERENABLE, 0);
    ls_event event;
    event.type = LSEVT_FAILSAFE_HEARTBEAT_MISSED;
    event.value = 0;
    xQueueSendToFront(ls_event_queue, (void *)&event, NULL);
}

void ls_failsafe_init(void)
{
    // set up interrupt
    gpio_set_intr_type(LSGPIO_FAILSAFE_HEARTBEAT, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(LSGPIO_FAILSAFE_HEARTBEAT, &_ls_failsafe_heartbeat_isr, NULL);

    // set up watchdog timer
    timer_config_t failsafe_watchdog_timer_config = {
        .divider = 80, // base is 80MHz => 1MHz
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_DIS,
    };
    ESP_ERROR_CHECK(timer_init(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, &failsafe_watchdog_timer_config));
    ESP_ERROR_CHECK(timer_set_counter_value(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, 0ULL));
    // alarm every 1.5ms (1500µs)
    ESP_ERROR_CHECK(timer_set_alarm_value(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, 1500));
    ESP_ERROR_CHECK(timer_enable_intr(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER));
    ESP_ERROR_CHECK(timer_isr_callback_add(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, _ls_failsafe_alarm_callback, NULL, 0));
}

void ls_failsafe_start(void)
{
    ESP_ERROR_CHECK(timer_set_counter_value(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, 0ULL));
    ESP_ERROR_CHECK(timer_start(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER));
}

void ls_failsafe_pause(void)
{
    ESP_ERROR_CHECK(timer_pause(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER));
}


