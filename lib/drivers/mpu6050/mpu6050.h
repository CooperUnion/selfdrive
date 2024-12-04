#ifndef MPU6050_H
#define MPU6050_H

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>

typedef struct {
	int16_t gyro_raw_xout;
	int16_t gyro_raw_yout;
	int16_t gyro_raw_zout;
} i2c_mpu6050_raw_gyro_t;

void mpu6050_init(i2c_master_dev_handle_t);

void mpu6050_get_raw_gyro(
	i2c_master_dev_handle_t i2c_dev, i2c_mpu6050_raw_gyro_t *gyro_raw_val);
#endif	// MPU6050_H
