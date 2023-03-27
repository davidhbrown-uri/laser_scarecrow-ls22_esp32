#include "homing.h"
#include "config.h"
#include "events.h"
#include "stepper.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "soc.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

// config.h currently (2023-03-27) includes: #define LSGPIO_MAGNETSENSE 4

#define LS_HOMING_INTERRUPT_SOURCE = ETS_GPIO_INTR_SOURCE

extern QueueHandle_t ls_event_queue;

static void IRAM_ATTR _ls_homing_interrupt_handler(void *args)
{

}

static void ls_homing_init(void)
{

// reinitialize GPIO for interrupt
	gpio_config_t gpioConfig;
	gpioConfig.pin_bit_mask = GPIO_SEL_4;
	gpioConfig.mode         = GPIO_MODE_INPUT;
	gpioConfig.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
	gpioConfig.intr_type    = GPIO_INTR_ANYEDGE;
	gpio_config(&gpioConfig);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(LSGPIO_MAGNETSENSE, _ls_homing_interrupt_handler, NULL	);
}