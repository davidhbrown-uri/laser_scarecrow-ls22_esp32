/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022  David H. Brown

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include "freertos/FreeRTOS.h"
#include "mpu6050.h"
#include "driver/i2c.h"

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


static esp_err_t mpu6050_write_reg_byte(uint8_t register_number, uint8_t data)
{
    // see page 35 of MPU-6000/MPU-6050 Product Specification; single-byte write sequence
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // AD+W (ACK)
    i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN); // RA (ACK)
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN); // DATA (ACK)
    i2c_master_stop(cmd); // P
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_read_reg_uint8(uint8_t register_number, uint8_t *data)
{
    // see page 36 of MPU-6000/MPU-6050 Product Specification; single-byte read sequence
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // AD+W (ACK)
    i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN); // RA (ACK)
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN); // AD+R (ACK)
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK); // (DATA) ACK
    i2c_master_stop(cmd); // P
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
static esp_err_t mpu6050_read_reg_int16(uint8_t register_number, int16_t *data)
{
    uint8_t data_h=0, data_l=0;
    // see page 36 of MPU-6000/MPU-6050 Product Specification; burst read sequence
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // AD+W (ACK)
    i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN); // RA (ACK)
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN); // AD+R (ACK)
    i2c_master_read_byte(cmd, &data_h, I2C_MASTER_ACK); // (DATA) ACK
    i2c_master_read_byte(cmd, &data_l, I2C_MASTER_NACK); // (DATA) NACK
    i2c_master_stop(cmd); // P
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    // reassemble 2s-complement value from separate high and low bytes
    *data = ((data_h & 0x80) ? -32878 : 0) + (data_h & 0x7f) * 256 + data_l;
    return ret;
}

/**
 * @brief Initialize the MPU6050 accelerometer
 * @todone Figure out why every other time the ESP32 is reset, this device is reporting only zero values.
 * 
 */
void mpu6050_begin(void)
{
    ESP_ERROR_CHECK(mpu6050_write_reg_byte(MPU6050_PWR_MGMT_1, 0x80));//reset
    uint8_t address = 255;
    ESP_ERROR_CHECK(mpu6050_read_reg_uint8(MPU6050_WHO_AM_I, &address));
    printf("MPU6050 base address is %d\n", address);
    // page 41 of the MPU60x0 register map document has a note that seemed helpful with the zero value reporting:
        // When using SPI interface, user should use DEVICE_RESET (register 107) as well as SIGNAL_PATH_RESET (register 104) to ensure the reset is performed properly. The sequence used should be:
        // 1. Set DEVICE_RESET = 1 (register PWR_MGMT_1)
        // 2. Wait 100ms
        // 3. Set GYRO_RESET = ACCEL_RESET = TEMP_RESET = 1 (register SIGNAL_PATH_RESET)
        // 4. Wait 100ms
    // just the primary device reset + delay, not the signal path reset appears to be sufficient
    ESP_ERROR_CHECK(mpu6050_write_reg_byte(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_RESET_BIT));
    // wait 100ms
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(mpu6050_write_reg_byte(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_CYCLE_BIT));
    ESP_ERROR_CHECK(mpu6050_write_reg_byte(MPU6050_PWR_MGMT_2, MPU6050_PWR_MGMT_2_ZA_ONLY));
}

float mpu6050_read_accel_z(void)
{
    int16_t value=0;
    ESP_ERROR_CHECK(mpu6050_read_reg_int16(MPU6050_ACCEL_ZOUT_H, &value));
    return ((float) value / 16384.0);
}
float mpu6050_read_temp(void)
{
    int16_t value=0;
    ESP_ERROR_CHECK(mpu6050_read_reg_int16(MPU6050_TEMP_H, &value));
    return ((float) value) / 340.0 + 36.53;
}
