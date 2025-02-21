#ifndef MPU6050_H
#define MPU6050_H

#include "hardware/i2c.h"

#include <stdint.h>


void mpu6050_init(i2c_inst_t *i2c,
	uint		      sda,
	uint		      scl,
	uint8_t		      gyro_fs,
	uint8_t		      accel_fs,
	uint		      baudrate);

void mpu6050_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz);

void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az);

#endif /* MPU6050_H */
