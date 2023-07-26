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
// loosely adapted from https://github.com/tockn/MPU6050_tockn

// Address pad is connected to ground on Jan '22 kit boards
#define KSTJ3_ADDR         0x0E
#define KSTJ3_ADDR_FLIPPED 0x0C
#define KXTJ3_CTRL_REG1 0x1B
#define KXTJ3_CTRL_REG2 0x1D
#define KXTJ3_ACCEL_ZOUT_L 0x0A
#define KXTJ3_ACCEL_ZOUT_H 0x0B
#define KXTJ3_WHO_AM_I_REG     0x0F
#define KXTJ3_WHO_AM_I_VALUE   0x35
#define KXTJ3_FLIP_CHECK 0x7F
#define KXTJ3_DCST_RESP_REG 0x0C
#define KXTJ3_DCST_RESP_VALUE 0x55

void kxtj3_begin(void);
float kxtj3_read_accel_z(void);
