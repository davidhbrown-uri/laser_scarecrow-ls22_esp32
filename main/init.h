#pragma once
#include "sdkconfig.h"
#include "driver/i2c.h"

void ls_gpio_initialize(void);
void check_efuse(void) ;
esp_err_t ls_i2c_master_init(void);
