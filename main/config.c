#include "config.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define LSGPIO_OUTPUT_PIN_SEL (\
    (1ULL<<LSGPIO_LASERPOWERENABLE) | \
    (1ULL<<LSGPIO_LASERHEATERENABLE) | \
    (1ULL<<LSGPIO_SERVOPOWERENABLE) | \
    (1ULL<<LSGPIO_REFLECTANCEENABLE) | \
    (1ULL<<LSGPIO_STEPPERSTEP) | \
    (1ULL<<LSGPIO_STEPPERDIRECTION) | \
    (1ULL<<LSGPIO_STEPPERSLEEP) | \
    (1ULL<<LSGPIO_BUZZERENABLE) \
    )
#define LSGPIO_INPUT_PIN_SEL (\
    (1ULL<<LSGPIO_KNOB3) | \
    (1ULL<<LSGPIO_KNOB4) | \
    (1ULL<<LSGPIO_KNOB5) | \
    (1ULL<<LSGPIO_KNOB6) | \
    (1ULL<<LSGPIO_LASERTEMP) | \
    (1ULL<<LSGPIO_LIGHTSENSE) | \
    (1ULL<<LSGPIO_MAGNETSENSE) | \
    (1ULL<<LSGPIO_REFLECTANCESENSE) | \
    (1ULL<<LSGPIO_TAPESETTING) \
)

void lsgpio_initialize(void)
{
    // a structure to hold all the GPIO configuration data
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = LSGPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    // ... and turn off (set low) the outputs
    gpio_set_level(LSGPIO_LASERPOWERENABLE, 0);
    gpio_set_level(LSGPIO_LASERHEATERENABLE, 0);
    gpio_set_level(LSGPIO_REFLECTANCEENABLE, 0);
    gpio_set_level(LSGPIO_STEPPERSLEEP, 0);
    // ... might as well set these low, too, just for consistency
    gpio_set_level(LSGPIO_STEPPERDIRECTION, 0);
    gpio_set_level(LSGPIO_STEPPERSTEP, 0);
    gpio_set_level(LSGPIO_BUZZERENABLE, 0);
    gpio_set_level(LSGPIO_SERVOPULSE, 0);

    // now our inputs
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = LSGPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

esp_err_t lsi2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LSI2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = LSI2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LSI2C_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}