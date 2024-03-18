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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_adc_cal.h"


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
int ls_lightsense_read_adc(adc_atten_t attenuation);
void ls_lightsense_read_task(void *pvParameter);
/**
 * First reads ADC with 0dB attenuation; if over 900mV, reads again with 11dB atten
 * Value returned has been converted from raw to mV using esp_adc_cal_raw_to_voltage()
*/
int ls_lightsense_read_hdr(void);

int ls_lightsense_threshold_on_mv(int index);
int ls_lightsense_threshold_off_mv(int index);
int ls_lightsense_threshold_count(void);

/* "Calibration data" April 7 2023 focusing on mapping low level lux to mV

lux -> mV   (raw)
2.8 -> 75   (0 raw)
3.3 -> 80   (20)
3.4 -> 84   (39)
3.6 -> 88   (56)
3.9 -> 95   (86)
4.0 -> 99   (101)
4.5 -> 112  (157)
5.0 -> 128  (221)
5.5 -> 143  (286)
6.0 -> 157  (344)
6.5 -> 170  (400)
7.0 -> 181  (444)
7.5 -> 190  (483)
8.0 -> 204  (543)
8.5 -> 223  (662)
9.0 -> 238  (683)
9.5 -> 251  (738)
10  -> 265  (798)
...
14  -> 395  (1343)
15  -> 432  (1496)
...
18  -> 537  (1939)
19  -> 575  (2120)
20  -> 604  (2218)
... above is 0dB atten; below is 11dB; mV should be consistent
27  -> 938  (976)
28  -> 982  (1029)
29  -> 1035 (1095)
30  -> 1070 (1138)
...
45  -> 1691 (1898)
50  -> 1852 (2096)

Suggest lux-on-off levels

nominal lux, on mV/ off mV
3   80  75
4   100 95
5   130 120
7.5 190 180

10  265 255

15  430 395
20  600 540  
30  1070 980
50  1850 1690

*/

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
500=>8, 9, 10, 12, 12, 12, 15, 14, 16, 25 lux

*/
