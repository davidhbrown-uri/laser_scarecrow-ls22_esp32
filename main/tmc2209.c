/**
 * @file tmc2209.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-02-01
 * @link https://github.com/trinamic/TMC-API
 * @copyright Copyright (c) 2023
 *
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_sntp.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "config.h"
#include "tmc2209.h"

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

// => UART wrapper
// Write [writeLength] bytes from the [data] array.
// If [readLength] is greater than zero, read [readLength] bytes from the
// [data] array.
void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength)
{
    // plan from https://github.com/trinamic/TMC-EvalSystem/blob/master/hal/Startrampe/tmc/UART.c (UART_readWrite())
    // clear buffers
    uart_flush(LSGPIO_STEPPER_UART);
    // write writeLength bytes from data to UART
    uart_write_bytes(LSGPIO_STEPPER_UART, (const char *)data, writeLength);
    // ensure UART has sent, possibly by forcing a wait
    ESP_ERROR_CHECK(uart_wait_tx_done(LSGPIO_STEPPER_UART, 100)); // wait timeout is 100 RTOS ticks (TickType_t)
    if (readLength <= 0)
    {
        return; // 0;
    }
    // DHB: on TMC 2209, don't we have to clear the read buffer if data was written because one-pin UART?
    // (maybe this method is never used for an immediate read after write?)
    // with a timeout, wait for the expected number of bytes to arrive
    // if they don't, return -1
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us_start = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    // read readLength bytes into the data buffer
    int length = 0;
    while (length < readLength)
    {
        gettimeofday(&tv_now, NULL);
        int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        if (time_us - time_us_start > 10000)
        { // timeout after 10ms
            return;// -1;
        }
        ESP_ERROR_CHECK(uart_get_buffered_data_len(LSGPIO_STEPPER_UART, (size_t *)&length));
    }
    length = uart_read_bytes(LSGPIO_STEPPER_UART, data, length, 100);
    return; // 0;
}
// <= UART wrapper

// => CRC wrapper
// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc2209_CRC8(uint8_t *data, size_t length)
{
        int i, j;
    uint8_t crc = 0; // this function returns it instead of storing it back in data
    uint8_t currentByte;
    for (i = 0; i < (length - 1); i++)
    {                              // Execute for all bytes of a message
        currentByte = data[i]; // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    }     // for message byte
    return crc;
}
// <= CRC wrapper

void ls_tmc2209_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(LSGPIO_STEPPER_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LSGPIO_STEPPER_UART, LSGPIO_STEPPERTX, LSGPIO_STEPPERRX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(LSGPIO_STEPPER_UART, uart_buffer_size,
                                        uart_buffer_size, 10, &uart_queue, 0));
}

void ls_tmc2209_enable(void)
{
    ESP_ERROR_CHECK(ESP_OK);
}

void ls_tmc2209_sleep(void)
{
    ESP_ERROR_CHECK(ESP_OK);
}
