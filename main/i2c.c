#include "i2c.h"
#include <stdio.h>
#include "driver/i2c.h"
#include "config.h"
#include "debug.h"
#include "mpu6050.h"
#include "kxtj3.h"
#include "lis2dh12.h"
#include "math.h"
#include "events.h"
#include "freertos/semphr.h"

// https://github.com/espressif/esp-idf/blob/a82e6e63d98bb051d4c59cb3d440c537ab9f74b0/examples/peripherals/i2c/i2c_tools/main/cmd_i2ctools.c
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

static bool _ls_i2c_initialized = false;
static enum ls_i2c_accelerometer_device_t _ls_i2c_accelerometer_device = LS_I2C_ACCELEROMETER_NONE;

/**
 * @brief i2c master initialization
 * @link https://github.com/espressif/esp-idf/blob/a82e6e63d98bb051d4c59cb3d440c537ab9f74b0/examples/peripherals/i2c/i2c_simple/main/i2c_simple_main.c
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = LSIC2_PORT;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LSI2C_SDA,
        .scl_io_num = LSI2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LSI2C_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t ls_i2c_write_reg_byte(uint8_t device_address, uint8_t register_number, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                            // S
    i2c_master_write_byte(cmd, device_address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // SAD+W (ACK)
    i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN);                        // RA (ACK)
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);                                   // DATA (ACK)
    i2c_master_stop(cmd);                                                             // P
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t ls_i2c_read_reg_uint8(uint8_t device_address, uint8_t register_number, uint8_t *data)
{
    // see page 23 of KXTJ3-1057 specification
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                            // S
    i2c_master_write_byte(cmd, device_address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // SAD+W (ACK)
    i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN);                        // RA (ACK)
    i2c_master_start(cmd);                                                            // S
    i2c_master_write_byte(cmd, device_address << 1 | I2C_MASTER_READ, ACK_CHECK_EN);  // SAD+R (ACK)
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);                                 // (DATA) ACK
    i2c_master_stop(cmd);                                                             // P
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

bool ls_i2c_init(void)
{
    if (!_ls_i2c_initialized && i2c_master_init() == ESP_OK)
    {
        _ls_i2c_initialized = true;
    }
    return _ls_i2c_initialized;
}

bool ls_i2c_probe_address(uint8_t address)
{
    if (!ls_i2c_init())
    {
        return false;
    }

    // from https://github.com/espressif/esp-idf/blob/a82e6e63d98bb051d4c59cb3d440c537ab9f74b0/examples/peripherals/i2c/i2c_tools/main/cmd_i2ctools.c lines 130ff
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(LSIC2_PORT, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}
enum ls_i2c_accelerometer_device_t ls_i2c_accelerometer_device(void)
{
    if (_ls_i2c_accelerometer_device != LS_I2C_ACCELEROMETER_NONE)
    {
        return _ls_i2c_accelerometer_device;
    }
    if (ls_i2c_has_lis2dh12())
    {
        lis2dh12_begin();
        _ls_i2c_accelerometer_device = LS_I2C_ACCELEROMETER_LIS2DH12;
    }
    if (ls_i2c_has_kxtj3())
    {
        kxtj3_begin();
        _ls_i2c_accelerometer_device = LS_I2C_ACCELEROMETER_KXTJ3;
    }
    if (ls_i2c_has_mpu6050())
    {
        mpu6050_begin();
        _ls_i2c_accelerometer_device = LS_I2C_ACCELEROMETER_MPU6050;
    }
    return _ls_i2c_accelerometer_device;
}

float ls_i2c_read_accel_z(void)
{
    ls_i2c_init();
    switch (ls_i2c_accelerometer_device())
    {
    case LS_I2C_ACCELEROMETER_KXTJ3:
        return kxtj3_read_accel_z();
        break;
    case LS_I2C_ACCELEROMETER_LIS2DH12:
        return lis2dh120_read_accel_z();
        break;
    case LS_I2C_ACCELEROMETER_MPU6050:
        return mpu6050_read_accel_z();
        break;
    case LS_I2C_ACCELEROMETER_NONE:
        return nanf("");
        break;
    }
    return nanf("");
}
/*
float ls_i2c_read_temp(void)
{
    ls_i2c_init();
    switch(_ls_i2c_accelerometer_device)
    {
        case LS_I2C_ACCELEROMETER_KXTJ3:
        return NAN;
        break;
        case LS_I2C_ACCELEROMETER_LIS2DH12:
        return NAN;
        break;
        case LS_I2C_ACCELEROMETER_MPU6050:
        return NAN;
        break;
        case LS_I2C_ACCELEROMETER_NONE:
        return NAN;
    }
}
*/

void ls_tilt_task(void *pvParameter)
{
    ls_event event;
    event.type = LSEVT_NOOP;
    event.value = NULL;
    while (1)
    {
        float raw = ls_i2c_read_accel_z();
//        xQueueSendToBack(ls_event_queue, (void *)&event, 0);
#ifdef LSDEBUG_I2C
        ls_debug_printf("I2C Z-acceleration=%0.2f\n", raw);
#endif
        vTaskDelay(pdMS_TO_TICKS(LS_TILT_REPORT_RATE_MS));
    }
}