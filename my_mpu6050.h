#ifndef MY_MPU6050_H
#define MY_MPU6050_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

// MPU6050 I2C地址
#define MPU6050_WHO_AM_I 0x75

// MPU6050寄存器地址
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

// I2C節點
#define I2C_NODE DT_NODELABEL(mysensor)
extern const struct i2c_dt_spec dev_i2c;

// 函數原型
void init_mpu6050(void);
bool Who_am_i(void);
void read_accel(int16_t *pAx, int16_t *pAy, int16_t *pAz);
void read_gyro(int16_t *pGx, int16_t *pGy, int16_t *pGz);
float accel_to_g(int16_t accel_raw);
float gyro_to_deg_per_sec(int16_t gyro_raw);

#endif // MY_MPU6050_H
