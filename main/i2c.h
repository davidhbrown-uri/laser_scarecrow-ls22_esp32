#pragma once
// FreeRTOS.h defines bool type
#include "freertos/FreeRTOS.h"

enum ls_i2c_accelerometer_device_t {
    LS_I2C_ACCELEROMETER_NONE,
    LS_I2C_ACCELEROMETER_LIS2DH12,
    LS_I2C_ACCELEROMETER_KXTJ3,
    LS_I2C_ACCELEROMETER_MPU6050
};

#define LS_I2C_ADDRESS_LIS2DH12 0x18
#define LS_I2C_ADDRESS_KXTJ3 0x0E
#define LS_I2C_ADDRESS_MPU6050 0x69


/**
 * @brief attempt to initialize and install the EPS32 i2c driver and detect accelerometer
 * 
 * @return true if successful
 * @return false if not successful
 */
bool ls_i2c_init(void);

/**
 * @brief Check the I2C bus for the presence of a device at the given address
 * 
 * @return true received acknowldgment
 * @return false timeout or other error
 */
bool ls_i2c_probe_address(uint8_t);

#define ls_i2c_has_lis2dh12() ls_i2c_probe_address(LS_I2C_ADDRESS_LIS2DH12)
#define ls_i2c_has_kxtj3() ls_i2c_probe_address(LS_I2C_ADDRESS_KXTJ3)
#define ls_i2c_has_mpu6050() ls_i2c_probe_address(LS_I2C_ADDRESS_MPU6050)

enum ls_i2c_accelerometer_device_t ls_i2c_accelerometer_device(void);

esp_err_t ls_i2c_write_reg_byte(uint8_t device_address, uint8_t register_number, uint8_t data);
esp_err_t ls_i2c_read_reg_uint8(uint8_t device_address, uint8_t register_number, uint8_t *data);

/**
 * @brief Acceleration downward on board should always be near 1.0g
 * 
 * @return float or NaN if no device
 */
float ls_i2c_read_accel_z(void);

/**
 * @brief Some accelerometers provide temperature data, too which might be useful
 * 
 * @return float or NaN if no device
 */
//float ls_i2c_read_temp(void);

void ls_tilt_task(void *pvParameter);
