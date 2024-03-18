/*
    Control software for URI Laser Scarecrow, 2022 Model
    Copyright (C) 2022-2024 David H. Brown

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
#include "i2c.h"
#include "config.h"
#include "debug.h"
#include "driver/i2c.h"
#include "events.h"
#include "freertos/semphr.h"
#include "kxtj3.h"
#include "lis2dh12.h"
#include "math.h"
#include "mpu6050.h"
#include "settings.h"
#include <stdio.h>


// https://github.com/espressif/esp-idf/blob/a82e6e63d98bb051d4c59cb3d440c537ab9f74b0/examples/peripherals/i2c/i2c_tools/main/cmd_i2ctools.c
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

enum _ls_tilt_task_tilt_status_t {
  LS_TILT_TASK_TILT_STATUS_OK,
  LS_TILT_TASK_TILT_STATUS_DETECTED,
  LS_TILT_TASK_TILT_STATUS_UNDEFINED
};

static bool _ls_i2c_initialized = false;
static enum ls_i2c_accelerometer_device_t _ls_i2c_accelerometer_device =
    LS_I2C_ACCELEROMETER_NONE;

/**
 * @brief i2c master initialization
 * @link
 * https://github.com/espressif/esp-idf/blob/a82e6e63d98bb051d4c59cb3d440c537ab9f74b0/examples/peripherals/i2c/i2c_simple/main/i2c_simple_main.c
 */
static esp_err_t i2c_master_init(void) {

  int i2c_master_port = LSI2C_PORT;

  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = LSI2C_SDA,
      .scl_io_num = LSI2C_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE, // have our own pullup resistors, too
      .scl_pullup_en = GPIO_PULLUP_ENABLE, // have our own pullup resistors, too
      .master.clk_speed = LSI2C_FREQ_HZ,
  };

  i2c_param_config(i2c_master_port, &conf);
  xSemaphoreTake(i2c_mux, LSI2C_MUX_TICKS);
  esp_err_t status =
      i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                         I2C_MASTER_TX_BUF_DISABLE, 0);
  xSemaphoreGive(i2c_mux);
#ifdef LSDEBUG_I2C
  ls_debug_printf(
      "I2C bus master initializing on port %d returning esp_err_t = %d.\n",
      LSI2C_PORT, status);
#endif
  return status;
}

esp_err_t ls_i2c_write_reg_byte(uint8_t device_address, uint8_t register_number,
                                uint8_t data) {
  xSemaphoreTake(i2c_mux, LSI2C_MUX_TICKS);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd); // S
  i2c_master_write_byte(cmd, device_address << 1 | I2C_MASTER_WRITE,
                        ACK_CHECK_EN);                       // SAD+W (ACK)
  i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN); // RA (ACK)
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);            // DATA (ACK)
  i2c_master_stop(cmd);                                      // P
  esp_err_t ret =
      i2c_master_cmd_begin(LSI2C_PORT, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  xSemaphoreGive(i2c_mux);
  return ret;
}

esp_err_t ls_i2c_read_reg_uint8(uint8_t device_address, uint8_t register_number,
                                uint8_t *data) {
  xSemaphoreTake(i2c_mux, LSI2C_MUX_TICKS);
  // see page 23 of KXTJ3-1057 specification
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd); // S
  i2c_master_write_byte(cmd, device_address << 1 | I2C_MASTER_WRITE,
                        ACK_CHECK_EN);                       // SAD+W (ACK)
  i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN); // RA (ACK)
  i2c_master_start(cmd);                                     // S
  i2c_master_write_byte(cmd, device_address << 1 | I2C_MASTER_READ,
                        ACK_CHECK_EN);              // SAD+R (ACK)
  i2c_master_read_byte(cmd, data, I2C_MASTER_NACK); // (DATA) ACK
  i2c_master_stop(cmd);                             // P
  esp_err_t ret =
      i2c_master_cmd_begin(LSI2C_PORT, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  xSemaphoreGive(i2c_mux);
  return ret;
}

bool ls_i2c_init(void) {
  if (!_ls_i2c_initialized && i2c_master_init() == ESP_OK) {
    _ls_i2c_initialized = true;
    vTaskDelay(1); // allow bus to settle down
#ifdef LSDEBUG_I2C
    ls_debug_printf("I2C LIS2DH: %d\n", ls_i2c_has_lis2dh12());
    ls_debug_printf("I2C KXTJ3: %d\n", ls_i2c_has_kxtj3());
    ls_debug_printf("I2C MPU6050: %d\n", ls_i2c_has_mpu6050());
#endif
  }
  return _ls_i2c_initialized;
}

bool ls_i2c_probe_address(uint8_t address) {
  if (!ls_i2c_init()) {
    return false;
  }

  // from
  // https://github.com/espressif/esp-idf/blob/a82e6e63d98bb051d4c59cb3d440c537ab9f74b0/examples/peripherals/i2c/i2c_tools/main/cmd_i2ctools.c
  // lines 130ff
  xSemaphoreTake(i2c_mux, LSI2C_MUX_TICKS);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(LSI2C_PORT, cmd, 50 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  xSemaphoreGive(i2c_mux);
  return (ret == ESP_OK);
}
enum ls_i2c_accelerometer_device_t ls_i2c_accelerometer_device(void) {
  if (_ls_i2c_accelerometer_device != LS_I2C_ACCELEROMETER_NONE) {
    return _ls_i2c_accelerometer_device;
  }
  if (ls_i2c_has_lis2dh12()) {
    lis2dh12_begin();
    _ls_i2c_accelerometer_device = LS_I2C_ACCELEROMETER_LIS2DH12;
  }
  if (ls_i2c_has_kxtj3()) {
    kxtj3_begin();
    _ls_i2c_accelerometer_device = LS_I2C_ACCELEROMETER_KXTJ3;
  }
  if (ls_i2c_has_mpu6050()) {
    mpu6050_begin();
    _ls_i2c_accelerometer_device = LS_I2C_ACCELEROMETER_MPU6050;
  }
  // if we haven't yet detected an accelerometer, we may need to try harder.

  if (LS_I2C_ACCELEROMETER_NONE == _ls_i2c_accelerometer_device) {
    if (ESP_OK == mpu6050_attempt_reset()) {
      printf("Detected MPU6050 accelerometer via MPU6050 reset\r\n");
      _ls_i2c_accelerometer_device = LS_I2C_ACCELEROMETER_MPU6050;
    }
  }
  return _ls_i2c_accelerometer_device;
}

/**
 * @brief Determine whether the unit has tipped
 *
 * Note that in 2023, the accelerometer is mounted upside-down, so these values
 * are inverted.
 *
 * @return float
 */
float ls_i2c_read_accel_z(void) {
  ls_i2c_init();
  switch (ls_i2c_accelerometer_device()) {
  case LS_I2C_ACCELEROMETER_KXTJ3:
    return 0.0 - kxtj3_read_accel_z();
    break;
  case LS_I2C_ACCELEROMETER_LIS2DH12:
    return 0.0 - lis2dh120_read_accel_z();
    break;
  case LS_I2C_ACCELEROMETER_MPU6050:
    return 0.0 - mpu6050_read_accel_z();
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

static enum _ls_tilt_task_tilt_status_t _ls_tilt_task_raw_to_status(float raw) {
  BaseType_t milli_gs = (BaseType_t)(raw * 1000);
#ifdef LSDEBUG_I2C
//        ls_debug_printf("milli-g's=%d\n", milli_gs);
#endif
  if (milli_gs < ls_settings_get_tilt_threshold_mg_detected()) {
    return LS_TILT_TASK_TILT_STATUS_DETECTED;
  }
  if (milli_gs > ls_settings_get_tilt_threshold_mg_ok()) {
    return LS_TILT_TASK_TILT_STATUS_OK;
  }
  return LS_TILT_TASK_TILT_STATUS_UNDEFINED;
}
#define LS_TILT_TASK_TILT_STATUS_READINGS_COUNT 5
void ls_tilt_task(void *pvParameter) {
  ls_event event;
  event.type = LSEVT_NOOP;
  event.value = NULL;
  enum _ls_tilt_task_tilt_status_t
      readings[LS_TILT_TASK_TILT_STATUS_READINGS_COUNT];
  enum _ls_tilt_task_tilt_status_t current_status =
      LS_TILT_TASK_TILT_STATUS_UNDEFINED;
  // ls_i2c_init(); // done by main
  // ls_i2c_accelerometer_device(); // done by main
  // the MPU6050, in particular, can take a while to initialize
  vTaskDelay(pdMS_TO_TICKS(5000));
  // and the produces a few reading of bogus data
  for (int i = 0; i < 5; i++) {
    ls_i2c_read_accel_z(); // throw it away
  }
  for (int i = 0; i < LS_TILT_TASK_TILT_STATUS_READINGS_COUNT; i++) {
    readings[i] = _ls_tilt_task_raw_to_status(ls_i2c_read_accel_z());
  }
  int readings_index = 0;
  while (1) {
    float raw = ls_i2c_read_accel_z();
    enum _ls_tilt_task_tilt_status_t status = _ls_tilt_task_raw_to_status(raw);
//        xQueueSendToBack(ls_event_queue, (void *)&event, 0);
#ifdef LSDEBUG_TILT
    ls_debug_printf("I2C Z-acceleration=%0.2f [%d]\n", raw, status);
#endif
    readings[readings_index++] = status;
    readings_index = readings_index % LS_TILT_TASK_TILT_STATUS_READINGS_COUNT;
    bool all_agree = true;
    for (int i = 1; i < LS_TILT_TASK_TILT_STATUS_READINGS_COUNT; i++) {
      all_agree = all_agree && (readings[i] == readings[i - 1]);
    }
    if (all_agree) {
      if (readings[0] != current_status) {
        switch (readings[0]) {
        case LS_TILT_TASK_TILT_STATUS_OK:
          event.type = LSEVT_TILT_OK;
          xQueueSendToBack(ls_event_queue, (void *)&event, 0);
          break;
        case LS_TILT_TASK_TILT_STATUS_DETECTED:
          event.type = LSEVT_TILT_DETECTED;
          xQueueSendToBack(ls_event_queue, (void *)&event, 0);
          break;
        default:;
        }
        current_status = readings[0];
#ifdef LSDEBUG_TILT
        ls_debug_printf("I2C tilt status now = %d\n", current_status);
#endif
      } // new status
    }   // all agree
    vTaskDelay(pdMS_TO_TICKS(LS_TILT_REPORT_RATE_MS));
  }
}