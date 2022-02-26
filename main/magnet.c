#include "magnet.h"
#include "config.h"
#include "events.h"
#include "stepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

bool ls_magnet_is_detected(void)
{
    // note that the sensor pulls low when triggered
    return gpio_get_level(LSGPIO_MAGNETSENSE) ? false : true;
}

extern QueueHandle_t ls_event_queue;
int32_t magnet_position;

void IRAM_ATTR magnet_event_isr(void *pvParameter)
{
    // note that the sensor pulls low when triggered
    magnet_position = ls_stepper_get_position();
    ls_event event;
    event.type = gpio_get_level(LSGPIO_MAGNETSENSE) ? LSEVT_MAGNET_LEAVE : LSEVT_MAGNET_ENTER;
    event.value = &magnet_position;
    xQueueSendFromISR(ls_event_queue, (void *)&event, NULL);
}

void ls_magnet_isr_begin(void)
{
    // set the magnet sensor to trigger an interrupt as it enters and as it leaves
    gpio_set_intr_type(LSGPIO_MAGNETSENSE, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0); // default, no flags.
    gpio_isr_handler_add(LSGPIO_MAGNETSENSE, &magnet_event_isr, NULL);
}