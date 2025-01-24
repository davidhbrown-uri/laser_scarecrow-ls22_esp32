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
#pragma once
#include "debug.h"
#include "driver/gpio.h"

// whenever settings code or config values are modified, increment this. Format can be YYYYMMDDHH;
// LS_SETTINGS_VERSION must fit within a 32-bit signed integer 
// (2^31-1 = 2147483647 gets us to the 47th hour of the 36th day in the 48th month of 2147 ;-) )
#define LS_SETTINGS_VERSION 2025012021

// Configure flags for attached hardware and general behavior
#define LS_HAS_DUAL_LASER
#undef LS_HAS_TAPE_SENSOR
#define LS_HAS_SERVO2
#define LS_HAS_LIGHTSENSE2

// Configure flag sanity checks
#ifdef LS_HAS_TAPE_SENSOR
#ifdef LS_HAS_DUAL_LASER
#error the tape sensor and second laser/servo cannot coexist (only two ends of PVC Tee)
#endif
#ifdef LS_HAS_LIGHTSENSE2
#error the tape sensor and second light sensor cannot coexist (same GPIO)
#endif 
#endif
// each laser has its own servo
#ifdef LS_HAS_DUAL_LASER
#define LS_HAS_SERVO2
#endif


/* ESP32 Devkit C GPIO
(Unlisted GPIO are not available... not broken out or used internally [SPIRAM, flash, USB, etc])

02 => Buzzer
04 => Magnet Sense
05 => NeoPixel (on reset, sets SDIO [Secure Digital] slave sampling edge... don't think we use this?)
12 => n.c. (on reset, sets VDD_FLASH; must be low on reset if connected; if we need this, may need to use https://docs.espressif.com/projects/esptool/en/latest/esp32/espefuse/set-flash-voltage-cmd.html to set correct value)
13 => Slider 2
14 => Slider 1
15 => Switches (on reset, sets LOG; seems to have no ill effect)
18 => Stepper Step
19 => Stepper Direction
21 => I2C SDA
22 => I2C SCL
23 => Stepper Enable
25 => Servo Enable
26 => Failsafe Hearbeat
27 => Servo 2 Pulse
32 => Laser Power Enable
33 => Servo Pulse (1)
34 => [IN] Reflectance Sense / Light Sense 2
35 => [IN] Tape Mode Setting (jumpers)
36 => [IN] Light Sense
39 => [IN] n.c. -- not appropriate for Failsafe Heartbeat (interrupt): 
https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/gpio.html
"Please do not use the interrupt of GPIO36 and GPIO39 when using ADC or Wi-Fi with sleep"
(we use ADC)
*/




// assignments of our devices to ESP32 peripherals
#define LSBUZZER_HS_LEDC_CHANNEL 0

// Laser Scarecrow GPIO and ADC channel 0 pin usage
// GPIO => ADC1 channel mapping:
// 25=>8, 32=> 4, 33=>5, 34=>6, 35=>7, 36=>0, 39=>3
// ADC channels
#define LSGPIO_LIGHTSENSE 36
#define LSADC1_LIGHTSENSE ADC1_CHANNEL_0
#ifdef LS_HAS_LIGHTSENSE2
#define LSGPIO_LIGHTSENSE2 34
#define LSADC1_LIGHTSENSE2 ADC1_CHANNEL_6
#endif
#ifdef LS_HAS_TAPE_SENSOR
#define LSGPIO_REFLECTANCESENSE 34
#define LSADC1_REFLECTANCESENSE ADC1_CHANNEL_6
#endif
#define LSGPIO_TAPESETTING 35
#define LSADC1_TAPESETTING ADC1_CHANNEL_7

// GPIO => ADC2 channel mapping:
// 0=>1, 2=>2, 4=>0, 15=>3, 13=>4, 12=>5, 14=>6, 15=>3, 27=>7
#define LSGPIO_SLIDER1 14
#define LSADC2_SLIDER1 ADC2_CHANNEL_6
#define LSGPIO_SLIDER2 13
#define LSADC2_SLIDER2 ADC2_CHANNEL_4
#define LSGPIO_SWITCHES 15
#define LSADC2_SWITCHES ADC2_CHANNEL_3

// MAGNETSENSE is digital input (ISR), not ADC
#define LSGPIO_MAGNETSENSE 4
// FAILSAFE_HEARTBEAT is a digital input (ISR)
#define LSGPIO_FAILSAFE_HEARTBEAT 26
// Binary output
#define LSGPIO_LASERPOWERENABLE 32
#define LSGPIO_SERVOPOWERENABLE 25
#ifdef LS_HAS_TAPE_SENSOR
#define LSGPIO_REFLECTANCEENABLE 27
#endif
#define LSGPIO_STEPPERDIRECTION 19
#define LSGPIO_STEPPERSTEP 18
// STEPPER_TXRX is STEPPER_ENABLE in EN-Diag mode
#define LSGPIO_STEPPERENABLE 23

// Stepper TMC2209 is enabled when pin is brought low
#define STEPPERENABLE_ENABLE 0
#define STEPPERENABLE_DISABLE 1

// PWM output (LEDC)
#define LSGPIO_SERVOPULSE 33
#ifdef LS_HAS_SERVO2
// GPIO27 is tape sensor enable in 2023-2024 models
#define LSGPIO_SERVOPULSE2 27
#endif
#define LSGPIO_BUZZERENABLE 2
// I2C (using controller 0; pins selected to match Arduino usage; maybe they
// have a reason?)
#define LSI2C_PORT I2C_NUM_0
#define LSI2C_SDA 21
#define LSI2C_SCL 22
#define LSI2C_FREQ_HZ 100000
// how many ticks a task can block waiting for the i2c_mux
#define LSI2C_MUX_TICKS 50

// https://github.com/JSchaenzle/ESP32-NeoPixel-WS2812-RMT/blob/master/Kconfig
// --- use menuconfig to set, but be sure the Kconfig file correctly labels the
// parameters! #define CONFIG_WS2812_NUM_LEDS 3 #define
// CONFIG_WS2812_LED_RMT_TX_GPIO 5 #define CONFIG_WS2812_LED_RMT_TX_CHANNEL 0
// // XL-5050RGBC-WS2812B
// // T0H 0.25μs => 10; T0LK 1μs => 40; T1H 0.85μs => 34; T1L 0.4μs => 16
// #define CONFIG_WS2812_T0H 10
// #define CONFIG_WS2812_T0L 40
// #define CONFIG_WS2812_T1H 34
// #define CONFIG_WS2812_T1L 16

// physical limits for the servo:
#define LS_SERVO_US_MIN 750
// 2250 not safe for dual; can jam/break laser wire
// #define LS_SERVO_US_MAX 2250
#define LS_SERVO_US_MAX 2000
// positions available to use in state-settings:
// -10deg not reasonable
// #define LS_SERVO_US_NEG10DEG 888
#define LS_SERVO_US_NEG5DEG 944
#define LS_SERVO_US_0DEG 1000
#define LS_SERVO_US_10DEG 1111
#define LS_SERVO_US_20DEG 1222
#define LS_SERVO_US_30DEG 1333
#define LS_SERVO_US_45DEG 1500
#define LS_SERVO_US_60DEG 1666
#define LS_SERVO_US_90DEG 2000
// defaults
#define LS_SERVO_US_MAX_LIMIT LS_SERVO_US_60DEG
#define LS_SERVO_US_MIN_LIMIT LS_SERVO_US_0DEG
// initial position during self-test
#define LS_SERVO_US_ASSEMBLY_REFERENCE LS_SERVO_US_0DEG
// selftest holds the servo at LS_SERVER_US_ASSEMBLY_REFERENCE this long to
// allow adjustment
#define LS_SERVO_SELFTEST_HOLD_MS 5000
#define LS_SERVO_DELTA_PER_TICK_DEFAULT 2
#define LS_SERVO_DELTA_PER_TICK_MAX 15
#define LS_SERVO_DELTA_PER_TICK_MIN 1
#define LS_SERVO_RANDOM_PAUSE_MS 300
#define LS_SERVO_SWEEP_PAUSE_MS 4000
// see https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/peripherals/mcpwm.html
#define LS_SERVO_MCPWM_UNIT MCPWM_UNIT_0
#define LS_SERVO_MCPWM_IO_SIGNALS MCPWM0A
#define LS_SERVO_MCPWM_TIMER MCPWM_TIMER_0
#define LS_SERVO_MCPWM_GENERATOR MCPWM_OPR_A
#ifdef LS_HAS_SERVO2
#define LS_SERVO2_MCPWM_UNIT MCPWM_UNIT_0
#define LS_SERVO2_MCPWM_IO_SIGNALS MCPWM1A
#define LS_SERVO2_MCPWM_TIMER MCPWM_TIMER_1
#define LS_SERVO2_MCPWM_GENERATOR MCPWM_OPR_A
#endif

// default parameters for the stepper movement
/// stepper motor is a standard 200-step-per-rotation motor
#define LS_STEPPER_FULLSTEPS_PER_ROTATION 200
#define LS_STEPPER_MICROSTEPS_PER_STEP 16
#define LS_STEPPER_STEPS_PER_ROTATION (LS_STEPPER_FULLSTEPS_PER_ROTATION * LS_STEPPER_MICROSTEPS_PER_STEP)
#define LS_STEPPER_RANDOM_HOP_STEPS_MIN (LS_STEPPER_STEPS_PER_ROTATION / 20)
#define LS_STEPPER_RANDOM_HOP_STEPS_MAX (LS_STEPPER_STEPS_PER_ROTATION / 2)
#define LS_STEPPER_RANDOM_HOP_REVERSE_PER255 96
// this value must be low enough that changes in direction are reasonably non-jerky
// (0 produces div/0 panic, so that's too low!)
#define LS_STEPPER_STEPS_PER_SECOND_MIN 240
// motor/laser seems to have no trouble at 4800 which is probably too fast
// is having trouble registering magnet reliably that fast, though.
// https://www.omc-stepperonline.com/support/what-is-the-maximum-speed-highest-frequency-of-the-stepper-motor
// gives max 1000 RPM; recommended working speed 100-500 RPM. 500RPM = 8.3333 rotations per second; *200*16 => 26666 micro-steps per second

// These "steps per second" values were originally implemented as 
// *alarms* per second where the timer ISR did half a pulse per alarm.
// So, the effective rotation speed was only half what these values implied.

#define LS_STEPPER_STEPS_PER_SECOND_MAX 3600
#define LS_STEPPER_STEPS_PER_SECOND_MAPPING 1800
#define LS_STEPPER_STEPS_PER_SECOND_WARNING 7200
#define LS_STEPPER_STEPS_PER_SECOND_DEFAULT 2700
// LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_SECOND will be added or subtracted to the steps per second when accelerating or decelerating
#define LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_SECOND 8000
// a constant doesn't work as well with the wider range
#define LS_STEPPER_SPINNING_ALARMS_CHANGE_DIVISOR (60ULL)
#define LS_STEPPER_SPINNING_ALARMS_CHANGE_MINIMUM (1ULL)

#define LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_TICK (LS_STEPPER_MOVEMENT_STEPS_DELTA_PER_SECOND / pdMS_TO_TICKS(1000))

#define LS_STEPPER_RANDOM_SPIN_REVERSE_PER255 50
#define LS_STEPPER_RANDOM_SPIN_MINIMUM_SECONDS 3
#define LS_STEPPER_RANDOM_SPIN_MAXIMUM_SECONDS 10

// Rotation Check - Magnet timeout - shorter is safer, but more likely to have false positive
// 4000 ms produced a timeout every few minutes (adequate for testing, 
// though about half the time the magnet was detected while doing the initial stop)
// 10000 ms or so should be reasonable for deployment
#define LS_STATE_MAGNET_TIMEOUT_PERIOD_MS 12000
#define LS_STATE_MAGNET_SLOWSPIN_MS_PER_RPM 1000
#define LS_STATE_MAGNET_SLOWSPIN_INITIAL_RPM 30
#define LS_STATE_MAGNET_SLOWSPIN_FINAL_RPM 15

#define LS_FAILSAFE_TIMER_GROUP TIMER_GROUP_0
#define LS_FAILESAFE_TIMER TIMER_1
// microseconds (µs) before failesafe alarm should trigger
#define LS_FAILESAFE_ALARM_US 1500

#ifdef LS_FAILSAFE_TESTING
// for testing the failsafe, 150-250 RPM
// #define LS_FAILSAFE_TESTING
#define LS_LASER_ENABLE_MINIMUM_RPM 100
#define LS_SETTINGS_MINIMUM_RPM_SCANNING 150
#define LS_SETTINGS_MAXIMUM_RPM_SCANNING 250
#else
// to drop exposure to IIIA instead of IIIB:
// correct value is 176RPM, so add a margin for 180
// normal values 190-300 RPM
//#define LS_LASER_ENABLE_MINIMUM_RPM 180
#define LS_LASER_ENABLE_MINIMUM_RPM_100MM 180
#define LS_SETTINGS_MINIMUM_RPM_SCANNING_100MM 190
#define LS_LASER_ENABLE_MINIMUM_RPM_1M 21
#define LS_SETTINGS_MINIMUM_RPM_SCANNING_1M 22
// #define LS_SETTINGS_MINIMUM_RPM_SCANNING 190
// #define LS_SETTINGS_DEFAULT_MAX_RPM 240
// 100MM values also for IIIA mode
#define LS_SETTINGS_DEFAULT_MAX_RPM_100MM 240
#define LS_SETTINGS_DEFAULT_MAX_RPM_1M 100
#define LS_SETTINGS_MAXIMUM_RPM_SCANNING 330
#endif

#define LS_SETTINGS_MINIMUM_RPM_MOVEMENT 5
/*
2023 dual switch setup with upper 22k, lower 10k resistors to 3V3 and 10k to
ground Testing on 2023-04-05 using final boards, 15ft Cat5 and 2x RJ45 cable
glands Observed min/max over about 15-20 seconds (11dB attenuator) Off:
160-172mV  (USB: 220-228mV) Upper (22k): 993-1017mV (USB: 773-777mV) Lower
(10k): 1554-1562mV (USB: 1123-1124mV) Both: (~6.9k): 1835-1843mV   (USB:
1279-1281mV)
*/
// values read by ADC from external controls
/* The calibrated mV ADC value read from the switches input should be compared
to these thresholds to determine the controls status: 0 < ADC <
LS_CONTROLS_SWITCH_THRESHOLD_UPPER => LS_CONTROLS_STATUS_OFF;
LS_CONTROLS_SWITCH_THRESHOLD_UPPER < ADC < LS_CONTROLS_SWITCH_THRESHOLD_LOWER =>
LS_CONTROLS_STATUS_UPPER; LS_CONTROLS_SWITCH_THRESHOLD_LOWER < ADC <
LS_CONTROLS_SWITCH_THRESHOLD_BOTH => LS_CONTROLS_STATUS_UPPER;
LS_CONTROLS_SWITCH_THRESHOLD_BOTH < ADC < 4096 => LS_CONTROLS_STATUS_BOTH;
*/
#define LS_CONTROLS_SWITCH_THRESHOLD_UPPER 500

#define LS_CONTROLS_SWITCH_THRESHOLD_LOWER 1200

#define LS_CONTROLS_SWITCH_THRESHOLD_BOTH 1650

#define LSADCATTEN_SLIDER ADC_ATTEN_11db
#define LSADCATTEN_SWITCHES ADC_ATTEN_11db
// to ensure the full range of value can be selected,
// any ADC mV reading >= LS_CONTROLS_READING_TOP is considered max
#define LS_CONTROLS_READING_TOP 3000
// any ADC mV reading <= LS_CONTROLS_READING_BOTTOM is considered min
#define LS_CONTROLS_READING_BOTTOM 200
// any ADC mV reading must change by this much from its previous value to be
// registered.
#define LS_CONTROLS_READING_MOVE_THRESHOLD 40
#define LS_CONTROLS_READINGS_TO_AVERAGE 5
// when controls are connected, readings are sent every tick, so 50 reads=~5sec
#define LS_CONTROLS_FASTREADS_AFTER_MOVE 50

// raw values read by ADC for 2023 units, 100% sampling of tape reflectance
// sensor, Nov '22 do not expect to use with black buckets, so just two values
// needed (more reflectance means more voltage to ground instead of ADC pin, so
// light is low and dark is high)
#define LS_REFLECTANCE_ADC_MAX_LIGHT 1300
#define LS_REFLECTANCE_ADC_MIN_DARK 2000

// approximate midpoints (raw ADC) between settings (3 boards tested Apr 2 '22)
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
#define LS_HOME_INITIAL_STEPPER_STEPS_PER_SECOND 2400
#define LS_HOME_BACKUP_ADDITIONAL_STEPS (LS_STEPPER_STEPS_PER_ROTATION / 10)
#define LS_HOME_FORWARD_STEPS (LS_STEPPER_STEPS_PER_ROTATION / 4)
#define LS_HOME_OFFSET_THRESHOLD_TO_REHOME (LS_STEPPER_STEPS_PER_ROTATION / 20)

// how often should we check rehome if using the map? 15000=15s debug/test,
// 1800000=30min production
#ifdef LSDEBUG_HOMING
#define LS_STATE_REHOME_TIMER_PERIOD_MS 15000
#else
#define LS_STATE_REHOME_TIMER_PERIOD_MS 1800000
#endif

// Thresholds based on sample data recorded in lightsense.h
// must be comma-separated list
#define LS_LIGHTSENSE_THRESHOLDS_ON_MV                                         \
  80, 100, 130, 190, 265, 430, 600, 1070, 1850
#define LS_LIGHTSENSE_THRESHOLDS_OFF_MV                                        \
  75, 95, 120, 180, 255, 395, 540, 980, 1690
#define LS_LIGHTSENSE_THRESHOLDS_COUNT 9
#define LS_LIGHTSENSE_THRESHOLD_DEFAULT 4
#define LS_LIGHTSENSE_READING_INTERVAL_MS 4000
#define LS_LIGHTSENSE_READINGS_TO_SWITCH 4
#define LS_LIGHTSENSE_READINGS_TO_AVERAGE 4

#define LS_TILT_THRESHOLD_DETECTED_MG 900
#define LS_TILT_THRESHOLD_OK_MG 950
#ifdef LSDEBUG_I2C
// the accelerometer will check this frequently if I2C debug is active
#define LS_TILT_REPORT_RATE_MS 500
#else
// the accelerometer will check this frequently during normal operation
#define LS_TILT_REPORT_RATE_MS 1000
#endif

// 18 steps in each cycle; 200ms/step; 1800 should allow three cycles
#define LS_FAILURE_LEDS_OFF_AFTER 11000
// after waiting for the LEDs also wait this long to play the tune and lights
// again
#define LS_FAILURE_REPEAT_INTERVAL 24000

// The LS_EVENT_NOOP_TIMEOUT_MS also controls the "snoring" rate of the sleep
// mode If no event is queued within this time, a LSEVT_NOOP will be sent to the
// current state
#define LS_EVENT_NOOP_TIMEOUT_MS 110000

#define LS_SETTINGS_SLEEP_LIGHT_ENABLE_DEFAULT false
