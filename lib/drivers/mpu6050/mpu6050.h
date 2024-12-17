#ifndef MPU6050_H
#define MPU6050_H

#include "i2c_hal_esp32.h"

typedef enum {
	FS_SEL_250,
	FS_SEL_500,
	FS_SEL_1000,
	FS_SEL_2000
} mpu6050_fs_sel_t;

typedef enum {
	AFS_SEL_2G,
	AFS_SEL_4G,
	AFS_SEL_8G,
	AFS_SEL_16G
} mpu6050_afs_sel_t;

typedef struct {
	int16_t gyro_raw_xout;
	int16_t gyro_raw_yout;
	int16_t gyro_raw_zout;
} mpu6050_raw_gyro_t;

typedef struct {
	int16_t accel_raw_xout;
	int16_t accel_raw_yout;
	int16_t accel_raw_zout;
} mpu6050_raw_accel_t;

typedef struct mpu6050_handle_s {
	i2c_hal_handle_t i2c_hal_handle;
} mpu6050_handle_t;

typedef struct mpu6050_config_s {
	i2c_hal_config_t  i2c_hal_config;
	mpu6050_fs_sel_t  fs;
	mpu6050_afs_sel_t afs;
	gpio_num_t	  scl;
	gpio_num_t	  sda;
} mpu6050_config_t;

void mpu6050_init(
	mpu6050_handle_t *mpu6050_handle, mpu6050_config_t *mpu6050_config);
void mpu6050_get_raw_accel(
	mpu6050_handle_t *mpu6050_handle, mpu6050_raw_accel_t *accel_raw_val);
void mpu6050_get_raw_gyro(
	mpu6050_handle_t *mpu6050_handle, mpu6050_raw_gyro_t *gyro_raw_val);

#endif	// MPU6050_H
