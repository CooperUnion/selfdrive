#include "mpu6050.h"

#include "i2c_hal_esp32.h"
#include <esp_log.h>

#define MPU6050_DEV_ADDR    0x68  // AD0
#define MPU6050_DEV_FREQ_HZ 400000

#define MPU6050_WHO_AM_I    0x75
#define MPU6050_DEV_ID	    0x68
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_PWR_MGTM_1  0x6B
#define MPU6050_ACCEL_XOUT  0x3B
#define MPU6050_GYRO_XOUT   0x43


/* Static Prototypes */
static const char *TAG = "MPU6050.c";

static void mpu6050_configure(
	mpu6050_handle_t *mpu6050_handle, mpu6050_config_t *mpu6050_config);
static void mpu6050_power_up(mpu6050_handle_t *mpu6050_handle);

/*
 * Performs startup initialization:
 * Checks device id
 * Configures sensitivity
 * Powers up
 */
void mpu6050_init(
	mpu6050_handle_t *mpu6050_handle, mpu6050_config_t *mpu6050_config)
{
	mpu6050_config->i2c_hal_config.device_address = MPU6050_DEV_ADDR;
	mpu6050_config->i2c_hal_config.scl_speed_hz   = MPU6050_DEV_FREQ_HZ;
	mpu6050_config->i2c_hal_config.scl	      = mpu6050_config->scl;
	mpu6050_config->i2c_hal_config.sda	      = mpu6050_config->sda;

	i2c_hal_init(&(mpu6050_handle->i2c_hal_handle),
		&(mpu6050_config->i2c_hal_config));

	uint8_t dev_id;
	i2c_hal_read(&(mpu6050_handle->i2c_hal_handle),
		MPU6050_WHO_AM_I,
		&dev_id,
		1);

	if (dev_id != (uint8_t) MPU6050_DEV_ID) {
		ESP_LOGE(TAG, "Failure @ dev id: %x", dev_id);
		return;
	}
	ESP_LOGI(TAG, "MPU6050 RECOGNIZED!");

	// configure sensitivity
	mpu6050_configure(mpu6050_handle, mpu6050_config);
	ESP_LOGI(TAG, "GRYO & ACCEL CONFIG FINISHED!");

	mpu6050_power_up(mpu6050_handle);
	ESP_LOGI(TAG, "AWAKE!");
}

void mpu6050_get_raw_gyro(
	mpu6050_handle_t *mpu6050_handle, mpu6050_raw_gyro_t *gyro_raw_val)
{
	uint8_t data[6];
	i2c_hal_read(&(mpu6050_handle->i2c_hal_handle),
		MPU6050_GYRO_XOUT,
		(uint8_t *) &data,
		sizeof(data));

	gyro_raw_val->gyro_raw_xout = (int16_t) (data[0] << 8 | data[1]);
	gyro_raw_val->gyro_raw_yout = (int16_t) (data[2] << 8 | data[3]);
	gyro_raw_val->gyro_raw_zout = (int16_t) (data[4] << 8 | data[5]);
}

void mpu6050_get_raw_accel(
	mpu6050_handle_t *mpu6050_handle, mpu6050_raw_accel_t *accel_raw_val)
{
	uint8_t data[6];
	i2c_hal_read(&(mpu6050_handle->i2c_hal_handle),
		MPU6050_ACCEL_XOUT,
		(uint8_t *) &data,
		sizeof(data));

	accel_raw_val->accel_raw_xout = (int16_t) (data[0] << 8 | data[1]);
	accel_raw_val->accel_raw_yout = (int16_t) (data[2] << 8 | data[3]);
	accel_raw_val->accel_raw_zout = (int16_t) (data[4] << 8 | data[5]);
}

static void mpu6050_configure(
	mpu6050_handle_t *mpu6050_handle, mpu6050_config_t *mpu6050_config)
{
	uint8_t data[2] = {mpu6050_config->fs << 3, mpu6050_config->afs << 3};

	i2c_hal_write(&(mpu6050_handle->i2c_hal_handle),
		MPU6050_GYRO_CONFIG,
		data,
		sizeof(data));
}

static void mpu6050_power_up(mpu6050_handle_t *mpu6050_handle)
{
	uint8_t data;
	i2c_hal_read(&(mpu6050_handle->i2c_hal_handle),
		MPU6050_PWR_MGTM_1,
		&data,
		1);

	// Turn off sleep mode
	data &= (~0x40);

	i2c_hal_write(&(mpu6050_handle->i2c_hal_handle),
		MPU6050_PWR_MGTM_1,
		&data,
		1);
}
