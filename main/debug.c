#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "debug.h"
#include "stepper.h"
#include "config.h"

#ifdef LSDEBUG_COVERAGE
// https://esp32.com/viewtopic.php?t=5793 for how to read state of OUTPUT pin
void ls_coverage_debug_task(void *pvParameter)
{
    int gpio = lsgpio_laserpowerenable();
    BaseType_t outreg = gpio < 32 ? GPIO_OUT_REG : GPIO_OUT1_REG;
    gpio_num_t pin = (gpio_num_t)(gpio & 0x1F);
    int state = 0;
    ls_debug_printf("Beginning LSDEBUG_COVERAGE output\n");
    while (1)
    {
        // pin is output - read the GPIO_OUT_REG register
        state = (GPIO_REG_READ(outreg) >> pin) & 1U;
        if (1 == state)
        {
            ls_debug_printf("%d\n", ls_stepper_get_position());
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
#endif