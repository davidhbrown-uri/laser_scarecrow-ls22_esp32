/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2023 David H. Brown

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
#include "lis2dh12.h"
#include "i2c.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define LIS2DH_REGISTER_CFG_REG 0x1F
#define LIS2DH_CFG_REG_TEMP_EN 0xC0

#define LIS2DH_REGISTER_CTRL_REG1 0x20
#define LIS2DH_CTRL_REG1_ODR10Hz 0x20
#define LIS2DH_CTRL_REG1_LPEN 0x08
#define LIS2DH_CTRL_REG1_ZEN 0x04
#define LIS2DH_CTRL_REG1_YEN 0x02
#define LIS2DH_CTRL_REG1_XEN 0x01

#define LIS2DH_REGISTER_CTRL_REG4 0x23
#define LIS2DH_CTRL_REG4_SCALE_2G 0x00
#define LIS2DH_CTRL_REG4_HIGHRES 0x08
#define LIS2DH_CTRL_REG4_LOWRES 0x00

#define LIS2DH_REGISTER_OUT_Z_L 0x2C
#define LIS2DH_REGISTER_OUT_Z_H 0x2D

#define LIS2DH_REGISTER_OUT_TEMP_L 0x0C
#define LIS2DH_REGISTER_OUT_TEMP_H 0x0D

void lis2dh12_begin(void)
{
    // set 2G 8-bit data
    ls_i2c_write_reg_byte(LS_I2C_ADDRESS_LIS2DH12, LIS2DH_REGISTER_CTRL_REG4,
                          LIS2DH_CTRL_REG4_SCALE_2G | LIS2DH_CTRL_REG4_LOWRES);
    // set 10Hz low-power Z-axis only
    ls_i2c_write_reg_byte(LS_I2C_ADDRESS_LIS2DH12, LIS2DH_REGISTER_CTRL_REG1,
                          LIS2DH_CTRL_REG1_ODR10Hz | LIS2DH_CTRL_REG1_LPEN | LIS2DH_CTRL_REG1_ZEN);
// enable temperature
//    ls_i2c_write_reg_byte(LS_I2C_ADDRESS_LIS2DH12, LIS2DH_REGISTER_CFG_REG, LIS2DH_CFG_REG_TEMP_EN);
}
float lis2dh120_read_accel_z(void)
{
    // using low-resolution (8-bit signed)
    uint8_t raw=0;
    ESP_ERROR_CHECK(ls_i2c_read_reg_uint8(LS_I2C_ADDRESS_LIS2DH12, LIS2DH_REGISTER_OUT_Z_H, &raw));
    int8_t value = ((raw & 0x80) ? -128 : 0) + (raw & 0x7f);
    return ((float) value / 64.0); // +/- 2g    
}
//float lis2dh12_read_temp(void);