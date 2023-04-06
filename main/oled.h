#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "config.h"

extern SemaphoreHandle_t i2c_mux;

typedef enum ls_oled_status_t {
    LS_OLED_NOT_FOUND,
    LS_OLED_PRESENT,
    LS_OLED_UNKNOWN
} ls_oled_status_t;

enum ls_oled_status_t ls_oled_status(void);
void ls_oled_init(void);
void ls_oled_show_logo(void);
void ls_oled_blank_screen(void);
void ls_oled_goto_line(uint8_t line_number);
void ls_oled_clear_line(uint8_t line_number);
void ls_oled_println(char * format, ...);
