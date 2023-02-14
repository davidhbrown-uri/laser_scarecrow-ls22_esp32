/**
 * @file tmc2209.h
 * @author David Brown (dave@davidhbrown.us)
 * @brief Routines to control the TMC 2209 stepper driver via UART
 * @version 0.1
 * @date 2022-12-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

/**
 * @brief Initialize the TMC 2209 driver on power-up
 * 
 */
void ls_tmc2209_init(void);

void ls_tmc2209_enable(void);

void ls_tmc2209_sleep(void);

// required by TMC-API https://github.com/trinamic/TMC-API

uint8_t tmc2209_CRC8(uint8_t *data, size_t length);
void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength);
