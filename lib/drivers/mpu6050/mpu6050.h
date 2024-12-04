#ifndef MPU6050_H
#define MPU6050_H

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>

typedef struct {
	int16_t gyro_raw_xout;
	int16_t gyro_raw_yout;
	int16_t gyro_raw_zout;
} i2c_mpu6050_raw_gyro_t;

void mpu6050_init(void);

#endif	// MPU6050_H
