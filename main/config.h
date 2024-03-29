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
#pragma once
#include "debug.h"
#include "driver/gpio.h"

// these GPIO assignments changed from the Nov '21 test board (rectangular)
// to the January/April '22 kit boards. On detecting the MPU6050 accelerometer
// that was on only the Nov '21 boards, these can be return different values.
void ls_config_set_gpio_nov21(void);
gpio_num_t IRAM_ATTR lsgpio_laserpowerenable(void);
gpio_num_t lsgpio_laserheaterenable(void);
gpio_num_t lsgpio_stepperdirection(void);
gpio_num_t lsgpio_steppersleep(void);
gpio_num_t lsgpio_servopowerenable(void);
gpio_num_t lsgpio_servopulse(void);

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
#define LSGPIO_LASERPOWERENABLE (lsgpio_laserpowerenable())
#define LSGPIO_SERVOPOWERENABLE (lsgpio_servopowerenable())
#define LSGPIO_LASERHEATERENABLE (lsgpio_laserheaterenable())
#define LSGPIO_REFLECTANCEENABLE 27
#define LSGPIO_STEPPERDIRECTION (lsgpio_stepperdirection())
#define LSGPIO_STEPPERSTEP 18
#define LSGPIO_STEPPERSLEEP (lsgpio_steppersleep())
// PWM output (LEDC)
#define LSGPIO_SERVOPULSE (lsgpio_servopulse())
#define LSGPIO_BUZZERENABLE 2
// I2C (using controller 0; pins selected to match Arduino usage; maybe they have a reason?)
#define LSI2C_PORT I2C_NUM_0
#define LSI2C_SDA 21
#define LSI2C_SCL 22
#define LSI2C_FREQ_HZ 400000
// currently unused
#define LSGPIO_SPARE2 23
#define LSGPIO_SPARE3 1
#define LSGPIO_SPARE4 3

// default parameters for the servo
#define LS_SERVO_US_MIN 750
#define LS_SERVO_US_MAX 2250
#define LS_SERVO_US_MID 1500
#define LS_SERVO_DELTA_PER_TICK_DEFAULT 2
#define LS_SERVO_DELTA_PER_TICK_MAX 100
#define LS_SERVO_DELTA_PER_TICK_MIN 1
#define LS_SERVO_RANDOM_PAUSE_MS 300
#define LS_SERVO_SWEEP_PAUSE_MS 4000
#define LS_SERVO_MCPWM_UNIT MCPWM_UNIT_0
#define LS_SERVO_MCPWM_IO_SIGNALS MCPWM0A
#define LS_SERVO_MCPWM_TIMER MCPWM_TIMER_0
#define LS_SERVO_MCPWM_GENERATOR MCPWM_OPR_A
// selftest holds the servo at midpoint this long to allow adjustment
#define LS_SERVO_SELFTEST_HOLD_MS 5000

// default parameters for the stepper movement
/// stepper motor is a standard 200-step-per-rotation motor
#define LS_STEPPER_FULLSTEPS_PER_ROTATION 200
#define LS_STEPPER_MICROSTEPS_PER_STEP 16
#define LS_STEPPER_STEPS_PER_ROTATION (LS_STEPPER_FULLSTEPS_PER_ROTATION * LS_STEPPER_MICROSTEPS_PER_STEP)
#define LS_STEPPER_MOVEMENT_STEPS_MIN 160
#define LS_STEPPER_MOVEMENT_STEPS_MAX (LS_STEPPER_STEPS_PER_ROTATION / 2)
#define LS_STEPPER_MOVEMENT_REVERSE_PER255 96
// this value must be low enough that changes in direction are reasonably non-jerky
#define LS_STEPPER_STEPS_PER_SECOND_MIN 240
// motor/laser seems to have no trouble at 4800 which is probably too fast
// is having trouble registering magnet reliably that fast, though.
#define LS_STEPPER_STEPS_PER_SECOND_MAX 3600
#define LS_STEPPER_STEPS_PER_SECOND_MAPPING 1800
#define LS_STEPPER_STEPS_PER_SECOND_WARNING 7200
#define LS_STEPPER_STEPS_PER_SECOND_DEFAULT 2400
// LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_SECOND will be added or subtracted to the steps per second when accelerating or decelerating
#define LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_SECOND 8000
#define LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_TICK ( LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_SECOND / pdMS_TO_TICKS(1000))

// values read by ADC from external controls
#define LS_CONTROLS_ADC_MAX_DISCONNECT 200
#define LS_CONTROLS_ADC_MIN_CONNECT 1600
#define LS_CONTROLS_ADC_MAX_CONNECT 1900
// to ensure the full range of value can be selected,
// any ADC reading >= LS_CONTROLS_READING_TOP is considered max
#define LS_CONTROLS_READING_TOP 4040
// any ADC reading <= LS_CONTROLS_READING_BOTTOM is considered min
#define LS_CONTROLS_READING_BOTTOM 50
// any ADC reading must change by this much from its previous value to be registered.
#define LS_CONTROLS_READING_MOVE_THRESHOLD 20
#define LS_CONTROLS_READINGS_TO_AVERAGE 5
// when controls are connected, readings are sent every tick, so 50 reads=~5sec
#define LS_CONTROLS_FASTREADS_AFTER_MOVE 50
// a second connection must happen within this many microseconds to enter the secondary controls
#define LS_CONTROLS_SECONDARY_US_TIME 6000000L

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

// we want two tape map entries per fullstep (microstepping must be 2 or larger)
#define LS_MAP_ENTRY_COUNT (LS_STEPPER_FULLSTEPS_PER_ROTATION * 2)
// map resolution: read tape sensor every n steps
#define LS_MAP_RESOLUTION (LS_STEPPER_STEPS_PER_ROTATION / LS_MAP_ENTRY_COUNT)
#define LS_MAP_ALLOWABLE_MISREAD_PERCENT 12
#define LS_MAP_HISTOGRAM_BINCOUNT 32

#define LS_HOME_ATTEMPTS_ALLOWED 3
#define LS_HOME_HOMINGS_TO_AVERAGE 5
#define LS_HOME_INITIAL_ROTATIONS 5
#define LS_HOME_STEPPER_STEPS_PER_SECOND 400
#define LS_HOME_INITIAL_STEPPER_STEPS_PER_SECOND 1800
#define LS_HOME_BACKUP_ADDITIONAL_STEPS (LS_STEPPER_STEPS_PER_ROTATION / 10)
#define LS_HOME_FORWARD_STEPS (LS_STEPPER_STEPS_PER_ROTATION / 4)
#define LS_HOME_OFFSET_THRESHOLD_TO_REHOME (LS_STEPPER_STEPS_PER_ROTATION/20)

// how often should we check rehome if using the map? 15000=15s debug/test, 1800000=30min production
#ifdef LSDEBUG_HOMING
#define LS_STATE_REHOME_TIMER_PERIOD_MS 15000
#else
#define LS_STATE_REHOME_TIMER_PERIOD_MS 1800000
#endif

// light levels based on sample data recorded in lightsense.h; roughly 40lux on, 20lux off
#define LS_LIGHTSENSE_DAY_THRESHOLD 1000
#define LS_LIGHTSENSE_NIGHT_THRESHOLD 500
#define LS_LIGHTSENSE_READING_INTERVAL_MS 4000
#define LS_LIGHTSENSE_READINGS_TO_SWITCH 4 

#define LS_TILT_THRESHOLD_DETECTED_MG 890
#define LS_TILT_THRESHOLD_OK_MG 920
#ifdef LSDEBUG_I2C
// the accelerometer will check this frequently if I2C debug is active
#define LS_TILT_REPORT_RATE_MS 200
#else
// the accelerometer will check this frequently during normal operation
#define LS_TILT_REPORT_RATE_MS 400
#endif

#define LS_EVENT_NOOP_TIMEOUT_MS 50000