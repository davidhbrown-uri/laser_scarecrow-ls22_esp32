/**
 * @file tmc2209.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-02-01
 *
 * @copyright Copyright (c) 2023
 *
 * inspired by but not copying from  (because GPL 3 and because I need only a tiny fraction of its capability)
 * https://www.haraldkreuzer.net/en/news/esp32-library-trinamic-tmc2208-stepper-motor-driver
 * https://github.com/HarryVienna/ESP32-TMC2208-UART-Component
 * https://github.com/HarryVienna/ESP32-TMC2208-UART-Component/blob/develop/components/stepper_driver/src/stepper_driver_tmc2208.c
 *
 */
#include "tmc2209.h"
#include "config.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

void ls_tmc2209_init()
{
    const uart_port_t uart_num = UART_NUM_2;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
};
// Configure UART parameters
ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, LSGPIO_STEPPERTX, LSGPIO_STEPPERRX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));    
}

/**
 * @brief Calculate CRC in last byte of datagram (from 2209's datasheet, changed UCHAR to uint8_t)
 * @link https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf
 * 
 * @param datagram 
 * @param datagramLength 
 */
void swuart_calcCRC(uint8_t *datagram, uint8_t datagramLength)
{
    int i, j;
    uint8_t *crc = datagram + (datagramLength - 1); // CRC located in last byte of message
    uint8_t currentByte;
    *crc = 0;
    for (i = 0; i < (datagramLength - 1); i++)
    {                              // Execute for all bytes of a message
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++)
        {
            if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
            {
                *crc = (*crc << 1) ^ 0x07;
            }
            else
            {
                *crc = (*crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    }     // for message byte
}