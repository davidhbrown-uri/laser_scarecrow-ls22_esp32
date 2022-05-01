#include "freertos/FreeRTOS.h"
#include "kxtj3.h"
#include "driver/i2c.h"

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

static uint8_t kxtj3_addr = KSTJ3_ADDR;

static esp_err_t kxtj3_write_reg_byte(uint8_t register_number, uint8_t data)
{
    // see page 23 of KXTJ3-1057 specification
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, kxtj3_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // SAD+W (ACK)
    i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN); // RA (ACK)
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN); // DATA (ACK)
    i2c_master_stop(cmd); // P
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t kxtj3_read_reg_uint8(uint8_t register_number, uint8_t *data)
{
    // see page 23 of KXTJ3-1057 specification
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, kxtj3_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // SAD+W (ACK)
    i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN); // RA (ACK)
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, kxtj3_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN); // SAD+R (ACK)
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK); // (DATA) ACK
    i2c_master_stop(cmd); // P
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
// unused unless we need high-resolution measurement (maybe bluetooth compass?)
/*
static esp_err_t kxtj3_read_reg_int16(uint8_t register_number, int16_t *data)
{
    uint8_t data_h=0, data_l=0;
    // see page 23 of KXTJ3-1057 specification
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, kxtj3_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // SAD+W (ACK)
    i2c_master_write_byte(cmd, register_number, ACK_CHECK_EN); // RA (ACK)
    i2c_master_start(cmd); // S
    i2c_master_write_byte(cmd, kxtj3_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN); // SAD+R (ACK)
    i2c_master_read_byte(cmd, &data_l, I2C_MASTER_ACK); // (DATA) ACK
    i2c_master_read_byte(cmd, &data_h, I2C_MASTER_NACK); // (DATA) NACK
    i2c_master_stop(cmd); // P
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    // reassemble 2s-complement value from separate high and low bytes
    *data = ((data_h & 0x80) ? -32878 : 0) + (data_h & 0x7f) * 256 + data_l;
    return ret;
}
*/

static void kxtj3_fail(void)
{
    printf("KXTJ3 failed software reset sequence; power-cycle required.\n");
    /** @todo something useful like lock out the device and make some distinctive noise */
}

static esp_err_t kxtj3_software_reset_sequence(void)
{
    kxtj3_addr = KSTJ3_ADDR;
    // SEE TN017 Power-On Procedure, especially section 3.1
    // 3.1.3 Software Reset Timing Diagram says to wait for power-on time before attempting reset
    // 30ms maximum power-on time... 
    vTaskDelay(1);
    // 3.1.2 (a)
    esp_err_t response = kxtj3_write_reg_byte(KXTJ3_FLIP_CHECK, 0);
    if(ESP_FAIL == response) { //"slave hasn't ACK the transfer"
    // attempt to reset using "flipped" address
    // 3.1.3 says to wait for reset time before flipped reset
    // 2ms maximum software reset time... 
    vTaskDelay(1);
        kxtj3_addr = KSTJ3_ADDR_FLIPPED;
        response = kxtj3_write_reg_byte(KXTJ3_FLIP_CHECK, 0);
        if(ESP_FAIL==response)
        {
            printf("KXTJ3 did not respond on either KXTJ3_ADDR or KXTJ3_ADDR_FLIPPED.\n");
            kxtj3_fail();
            return ESP_FAIL;
        }
    }
    // 3.1.3 says to wait for reset time before commands are acceptable
    // 2ms maximum software reset time... 
    vTaskDelay(1);
    // 3.1.2 (b)
    response = kxtj3_write_reg_byte(KXTJ3_CTRL_REG2, 0);
    if(ESP_FAIL==response)
    {
        printf("KXTJ3 did not write 0x00 to Control Register 2.\n");
        kxtj3_fail();
        return ESP_FAIL;
    }
    // 3.1.2 (c) -- initiate software reset
    response = kxtj3_write_reg_byte(KXTJ3_CTRL_REG2, 0x80);
    if(ESP_FAIL==response)
    {
        printf("KXTJ3 did not acknowledge software reset (0x80 to CTRL_REG2).\n");
        kxtj3_fail();
        return ESP_FAIL;
    }
    // 2ms maximum software reset time... 
    vTaskDelay(1);
    // 3.1.2 (d) 
    // Read WHO_AM_I using Primary address
    kxtj3_addr = KSTJ3_ADDR;
    uint8_t value = 255;
    kxtj3_read_reg_uint8(KXTJ3_WHO_AM_I_REG, &value);
    if (value != KXTJ3_WHO_AM_I_VALUE)
    {
        printf("KXTJ3 WHO_AM_I reported %d but should be %d.\n", value, KXTJ3_WHO_AM_I_VALUE);
        kxtj3_fail();
        return ESP_FAIL;
    }
    // 3.1.2 (e) 
    // Read DCST_RESP using Primary address
    kxtj3_read_reg_uint8(KXTJ3_DCST_RESP_REG, &value);
    if (value != KXTJ3_DCST_RESP_VALUE)
    {
        printf("KXTJ3 DCST_RESP reported %d but should be %d.\n", value, KXTJ3_DCST_RESP_VALUE);
        kxtj3_fail();
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Initialize the KXTJ3-1057 accelerometer
 * 
 */
void kxtj3_begin(void)
{
   if(1) {
       kxtj3_software_reset_sequence();
   }
    // reset value of CTRL_REG1 is 0x00: standby mode, but I suppose there's no harm in making sure.
    ESP_ERROR_CHECK(kxtj3_write_reg_byte(KXTJ3_CTRL_REG1, 0x00));//standby
    // all the defaults seem good... 
    ESP_ERROR_CHECK(kxtj3_write_reg_byte(KXTJ3_CTRL_REG1, 0x80));//operating
}

float kxtj3_read_accel_z(void)
{
    // using low-resolution (8-bit signed)
    uint8_t raw=0;
    ESP_ERROR_CHECK(kxtj3_read_reg_uint8(KXTJ3_ACCEL_ZOUT_H, &raw));
    int8_t value = ((raw & 0x80) ? -128 : 0) + (raw & 0x7f);
    return ((float) value / 64.0); // +/- 2g
}

