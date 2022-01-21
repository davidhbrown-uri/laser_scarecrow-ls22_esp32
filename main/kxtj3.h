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
