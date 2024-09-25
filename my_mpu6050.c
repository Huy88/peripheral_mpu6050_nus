#include <version.h>
#include <zephyr/kernel.h>
#include "my_mpu6050.h"
#include <nrfx_clock.h>
#include <string.h>

// I2C設備結構
const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

// 將原始加速度數據轉換為重力加速度
float accel_to_g(int16_t accel_raw) {
    return accel_raw / 16384.0;
}

// 將原始陀螺儀數據轉換為每秒度數
float gyro_to_deg_per_sec(int16_t gyro_raw) {
    return gyro_raw / 131.0;
}

void write_to_mpu6050(uint16_t register_address, uint8_t value) {
    int ret;
    uint8_t tx_buf[2];
    tx_buf[0] = register_address;
    tx_buf[1] = value;

    ret = i2c_write_dt(&dev_i2c, tx_buf, sizeof(tx_buf));
    if (ret != 0) {
        printk("Failed to write %d to register %d, error %d\n", value, register_address, ret);
    } else {
        printk("Successfully wrote %d to register %d\n", value, register_address);
    }
}

void read_from_mpu(uint8_t register_address, uint8_t *destination, uint8_t number) {
    int ret;
    ret = i2c_burst_read_dt(&dev_i2c, register_address, destination, number);
    if (ret != 0) {
        printk("Failed to read from register %d\n", register_address);
    }
}

void read_accel(int16_t *pAx, int16_t *pAy, int16_t *pAz) {
    uint8_t buf[6];
    read_from_mpu(MPU6050_RA_ACCEL_XOUT_H, buf, 6);
    *pAx = (buf[0] << 8) | buf[1];
    *pAy = (buf[2] << 8) | buf[3];
    *pAz = (buf[4] << 8) | buf[5];
}

void read_gyro(int16_t *pGx, int16_t *pGy, int16_t *pGz) {
    uint8_t buf[6];
    read_from_mpu(MPU6050_RA_GYRO_XOUT_H, buf, 6);
    *pGx = (buf[0] << 8) | buf[1];
    *pGy = (buf[2] << 8) | buf[3];
    *pGz = (buf[4] << 8) | buf[5];
}

void init_mpu6050(void) {
    write_to_mpu6050(MPU6050_RA_PWR_MGMT_1, 0x00);
    write_to_mpu6050(MPU6050_RA_SMPLRT_DIV, 0x07);
    write_to_mpu6050(MPU6050_RA_CONFIG, 0x06);
    write_to_mpu6050(MPU6050_RA_GYRO_CONFIG, 0x00);
    write_to_mpu6050(MPU6050_RA_ACCEL_CONFIG, 0x00);
}

bool Who_am_i(void) {
    int ret;
    uint8_t who_am_i;
    read_from_mpu(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != 0) {
        printk("Failed to write/read I2C device address %x at Reg. %x \n", dev_i2c.addr, MPU6050_WHO_AM_I);
    }
    if (who_am_i == 0x70) {
        printk("True Device");
        return true;
    } else {
        printk("Who_am_i: %d (0x%x)", who_am_i, who_am_i);
        return false;
    }
}
