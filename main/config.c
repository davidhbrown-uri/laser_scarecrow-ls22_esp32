#include "config.h"
#include "driver/gpio.h"

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
    // there is a bit of leakage current throuhg 
}
