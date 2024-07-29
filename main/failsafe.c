#include "failsafe.h"
#include "config.h"
#include "events.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "freertos/queue.h"


#define LS_FAILSAFE_ALARM_INTERVAL_US 1500ULL
#define LS_FAILSAFE_MINIMUM_EDGES 3ULL
#define LS_FAILSAFE_ALARM_FAILURES_LIMIT 100
static BaseType_t IRAM_ATTR s_ls_failsafe_alarm_failures = 0;
static uint64_t IRAM_ATTR s_ls_failsafe_heartbeat_edge_count = 0ULL;
#ifdef LSDEBUG_FAILSAFE
static uint64_t IRAM_ATTR s_ls_failsafe_heartbeat_edge_count_last = 0ULL;
#endif
// static void IRAM_ATTR _ls_failsafe_heartbeat_counter_isr(void *pvParameter)
// {
//     _ls_failsafe_heartbeat_edge_count++;
// }
static void IRAM_ATTR _ls_failsafe_heartbeat_edge_isr(void *pvParameter)
{
    s_ls_failsafe_heartbeat_edge_count++;
 }

uint64_t ls_failsafe_edge_count(void) { return s_ls_failsafe_heartbeat_edge_count; }

static bool IRAM_ATTR _ls_failsafe_alarm_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    if (s_ls_failsafe_heartbeat_edge_count < LS_FAILSAFE_MINIMUM_EDGES) {
        gpio_set_level(LSGPIO_LASERPOWERENABLE, 0);
        s_ls_failsafe_alarm_failures++;
        if(s_ls_failsafe_alarm_failures > LS_FAILSAFE_ALARM_FAILURES_LIMIT)
        {
            ls_event event;
            event.type = LSEVT_FAILSAFE_HEARTBEAT_MISSED;
            event.value = 0;
            xQueueSendToFrontFromISR(ls_event_queue, (void *)&event, &high_task_awoken);
        }
    }
    else {
        s_ls_failsafe_alarm_failures = 0;
    }
#ifdef LSDEBUG_FAILSAFE
    s_ls_failsafe_heartbeat_edge_count_last = s_ls_failsafe_heartbeat_edge_count;
#endif
    s_ls_failsafe_heartbeat_edge_count = 0ULL;
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

void ls_failsafe_init(void)
{
    gpio_set_intr_type(LSGPIO_FAILSAFE_HEARTBEAT, GPIO_INTR_ANYEDGE);

    // set up watchdog timer
    timer_config_t failsafe_watchdog_timer_config = {
        .divider = 80, // base is 80MHz => 1MHz
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    ESP_ERROR_CHECK(timer_init(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, &failsafe_watchdog_timer_config));
    ESP_ERROR_CHECK(timer_set_counter_value(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, 0));
    // alarm every 1.5ms (1500µs)
    ESP_ERROR_CHECK(timer_set_alarm_value(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, LS_FAILSAFE_ALARM_INTERVAL_US));
    ESP_ERROR_CHECK(timer_enable_intr(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER));
    ESP_ERROR_CHECK(timer_isr_callback_add(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER, _ls_failsafe_alarm_isr_callback, NULL, 0));
}

void ls_failsafe_start(void)
{
#ifdef LSDEBUG_FAILSAFE
    ls_debug_printf("FAILSAFE: starting alarm timer\n");
#endif
    s_ls_failsafe_heartbeat_edge_count = 0ULL;
    // set up interrupt to start counting edges
    gpio_isr_handler_add(LSGPIO_FAILSAFE_HEARTBEAT, &_ls_failsafe_heartbeat_edge_isr, NULL);
    // start alarm timer
    ESP_ERROR_CHECK(timer_start(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER));
}

void ls_failsafe_pause(void)
{
#ifdef LSDEBUG_FAILSAFE
    ls_debug_printf("FAILSAFE: pausing alarm timer\n");
#endif
    ESP_ERROR_CHECK(timer_pause(LS_FAILSAFE_TIMER_GROUP, LS_FAILESAFE_TIMER));
    gpio_isr_handler_remove(LSGPIO_FAILSAFE_HEARTBEAT);
}

#ifdef LSDEBUG_FAILSAFE
void ls_failsafe_debug_task(void *pvParameter)
{
    while (1)
    {
        ls_debug_printf("FAILSAFE: %llu heartbeat edges counted in last alarm interval\n", s_ls_failsafe_heartbeat_edge_count_last);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
#endif