#pragma once

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
// "KNOB#" refers to the pin # on the connector labeled "Knobs" on the January '22 boards
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

// default parameters for the servo
#define LS_SERVO_US_MIN 750
#define LS_SERVO_US_MAX 2250
#define LS_SERVO_US_MID 1500
#define LS_SERVO_MCPWM_UNIT MCPWM_UNIT_0
#define LS_SERVO_MCPWM_IO_SIGNALS MCPWM0A
#define LS_SERVO_MCPWM_TIMER MCPWM_TIMER_0
#define LS_SERVO_MCPWM_GENERATOR MCPWM_OPR_A

// default parameters for the stepper movement
#define LS_STEPPER_STEPS_PER_ROTATION 3200
// FULLSPEED determines how many steps it takes to get to full speed. Probably 5%-10% of STEPS_PER_SECOND_MAX?
#define LS_STEPPER_STEPS_FULLSPEED 120
#define LS_STEPPER_MOVEMENT_STEPS_MIN 160
#define LS_STEPPER_MOVEMENT_STEPS_MAX 1200
#define LS_STEPPER_MOVEMENT_REVERSE_PER255 64
#define LS_STEPPER_STEPS_PER_SECOND_MIN 128
// motor/laser seems to have no trouble at 4800 which is probably too fast
// is having trouble registering magnet reliably that fast, though.
#define LS_STEPPER_STEPS_PER_SECOND_MAX 1800

// values read by ADC from external controls
#define LS_CONTROLS_ADC_MAX_DISCONNECT 100
#define LS_CONTROLS_ADC_MIN_CONNECT 1700
#define LS_CONTROLS_ADC_MAX_CONNECT 1900

// values read by ADC for tape reflectance sensor
// based on testing conducted March 18 '22
#define LS_REFLECTANCE_ADC_MAX_WHITE_BUCKET 1750
#define LS_REFLECTANCE_ADC_MIN_BLACK_TAPE 2750
#define LS_REFLECTANCE_ADC_MIN_BLACK_BUCKET 2000
#define LS_REFLECTANCE_ADC_MAX_SILVER_TAPE 500

// approximate midpoints between settings (3 boards tested Apr 2 '22)
#define LS_TAPEMODE_THRESHOLD_1 300
#define LS_TAPEMODE_THRESHOLD_2 975
#define LS_TAPEMODE_THRESHOLD_3 1750
#define LS_TAPEMODE_THRESHOLD_4 2525
#define LS_TAPEMODE_THRESHOLD_5 3525

// map resolution: read tape sensor every n steps
#define LS_MAP_RESOLUTION (LS_STEPPER_STEPS_PER_ROTATION / 400)
#define LS_MAP_ALLOWABLE_GRAY_PERCENT 4

