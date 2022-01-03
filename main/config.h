#pragma once
#include "driver/i2c.h"

void lsgpio_initialize(void);
esp_err_t lsi2c_master_init(void);

// comment out to compile for production boards
#define LSBOARD_TESTNOV21

// assignments of our devices to ESP32 peripherals
#define LSBUZZER_HS_LEDC_CHANNEL 0

// Laser Scarecrow GPIO and ADC channel 0 pin usage
// GPIO => ADC1 channel mapping:
// 25=>8, 32=> 4, 33=>5, 34=>6, 35=>7, 36=>0, 39=>3 
// ADC channels
#define LSGPIO_LIGHTSENSE 36
#define LSADC1_LIGHTSENSE ADC1_CHANNEL_0
#define LSGPIO_LASERTEMP 39
#define LSADC1_LASERTEMP ADC1_CHANNEL_3
#define LSGPIO_REFLECTANCESENSE 34
#define LSADC1_REFLECTANCESENSE ADC1_CHANNEL_6
#define LSGPIO_TAPESETTING 35
#define LSADC1_TAPESETTING ADC1_CHANNEL_7
// GPIO => ADC2 channel mapping:
// 0=>1, 2=>2, 4=>0, 15=>3, 13=>4, 12=>5, 14=>6, 15=>3, 27=>7
#define LSGPIO_KNOB3 14
#define LSADC2_KNOB3 ADC2_CHANNEL_6
#define LSGPIO_KNOB4 12
#define LSADC2_KNOB4 ADC2_CHANNEL_5
#define LSGPIO_KNOB5 13
#define LSADC2_KNOB5 ADC2_CHANNEL_4
#define LSGPIO_KNOB6 15
#define LSADC2_KNOB6 ADC2_CHANNEL_3
// MAGNETSENSE is digital input (ISR), not ADC
#define LSGPIO_MAGNETSENSE 4
// Binary output
#define LSGPIO_LASERPOWERENABLE 32
#define LSGPIO_SERVOPOWERENABLE 25
#define LSGPIO_LASERHEATERENABLE 26
#define LSGPIO_REFLECTANCEENABLE 27
#define LSGPIO_STEPPERDIRECTION 19
#define LSGPIO_STEPPERSTEP 18
#define LSGPIO_STEPPERSLEEP 5
// PWM output (LEDC)
#define LSGPIO_SERVOPULSE 33
#define LSGPIO_BUZZERENABLE 2
// I2C (using controller 0; pins selected to match Arduino usage; maybe they have a reason?)
#define LSIC2_PORT I2C_NUM_0
#define LSI2C_SDA 21
#define LSI2C_SCL 22
#define LSI2C_FREQ_HZ 100000
// currently unused
#define LSGPIO_SPARE2 23
#define LSGPIO_SPARE3 1
#define LSGPIO_SPARE4 3


// some pins were arranged differently on the test board
#ifdef LSBOARD_TESTNOV21
    //temporary, for diagnosing buzzer electronics issue using spare GPIO pin 
    // #undef LSGPIO_BUZZERENABLE
    // #define LSGPIO_BUZZERENABLE 23
    #undef LSGPIO_STEPPERDIRECTION
    #define LSGPIO_STEPPERDIRECTION 5
    #undef LSGPIO_STEPPERSLEEP
    #define LSGPIO_STEPPERSLEEP 19
    #undef LSGPIO_LASERHEATERENABLE
    #define LSGPIO_LASERHEATERENABLE 32
    #undef LSGPIO_LASERPOWERENABLE
    #define LSGPIO_LASERPOWERENABLE 33
    #undef LSGPIO_SERVOPULSE
    #define LSGPIO_SERVOPULSE 25
    #undef LSGPIO_SERVOPOWERENABLE
    #define LSGPIO_SERVOPOWERENABLE 26
#endif