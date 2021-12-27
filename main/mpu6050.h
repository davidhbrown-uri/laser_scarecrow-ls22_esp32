#pragma once
// loosely adapted from https://github.com/tockn/MPU6050_tockn

// "The reset value is 0x00 for all registers other than..."
// PWM_MGT_1 = SLEEP; WHO_AM_I=0x68
#define MPU6050_ADDR         0x69
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_CONFIG_2G 0x00
#define MPU6050_USER_CTRL   0x6a
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_PWR_MGMT_1_SLEEP_BIT 0x40
#define MPU6050_PWR_MGMT_1_CYCLE_BIT 0x20
#define MPU6050_PWR_MGMT_1_TEMP_DIS_BIT 0x08
#define MPU6050_PWR_MGMT_2   0x6c
// LP_WAKE_CTRL = b01 (5Hz wakes); STBY_XA=STBY_YA=STBY_XG=STBY_YG=STBY_ZG=1
#define MPU6050_PWR_MGMT_2_ZA_ONLY   0x77
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42
#define MPU6050_WHO_AM_I     0x75

void mpu6050_begin(void);
float mpu6050_read_accel_z(void);
float mpu6050_read_temp(void);