#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum ls_lightsense_mode_t {
    LS_LIGHTSENSE_MODE_NIGHT,
    LS_LIGHTSENSE_MODE_DAY,
    LS_LIGHTSENSE_MODE_STARTUP
};

enum ls_lightsense_level_t {
    LS_LIGHTSENSE_LEVEL_NIGHT,
    LS_LIGHTSENSE_LEVEL_DAY,
    LS_LIGHTSENSE_LEVEL_INDETERMINATE
};

enum ls_lightsense_mode_t ls_lightsense_current_mode(void);
int ls_lightsense_read_adc(void);
void ls_lightsense_read_task(void *pvParameter);


/* "Calibration" data
April 4 '22: measuring actual light levels with HoldPeak HP881-C meter and one sample phototransistor
TimeHMM lux reading - comment
530 0.2 0
542 0.1 0 - some birdsong audible
555 0.8 0 - no birds can perceive color and write without flashlight; dawn glow visible to east
605 8.2 200 - thought I saw a bird but none at feeder
616 50 1600 - birds in trees hopping in branches but not at feeder; sun not visible
621 115 3600 - birds flying but not at feeder
624 178 4000 - first bird at feeder
628 220 4020
... 276 4040
... 280 4050
... 315 4060
633 330 4070
... 350 4080
635 370 4070 - birds at feeder with me only about 6' away
... 385 3080
... 400 4080
... 430 4085 - cat returned from morning prowl
640 500 4090 - daybreak; slight overcast; overcast sun position visible in treetops


April 13 '22:
Set desired light level with meterusing filtered cool white LED source, 
then read sampled phototransistors individually.
First reading at each level is same device as initial test; others not consistently ordered.
@3lux : 180,  30,  30,  96,   0, 100,  27
@10lux: 684, 455, 535, 309, 398, 360, 340
@50 lux: 3240, 2370, 3030, 3600, 3200, 2400, 2222

Individual set light level so each phototransistor read as close to 500 as possible,
then measured with light meter (same first device)
500=>9 lx, 25, 12, 12, 8, 15, 14, 10, 12, 16 
*/
