#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "config.h"

void app_main(void)
{
    lsgpio_initialize();
    printf("Initialized GPIO\n");
    ESP_ERROR_CHECK(i2c_master_init());
    printf("Initialized I2C\n");
}
