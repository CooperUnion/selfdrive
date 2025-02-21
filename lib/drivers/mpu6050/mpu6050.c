#include "mpu6050.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define MPU6050_I2C_ADDR   0x68
#define MPU6050_PWR_MGMT_1 0x6B

#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_GYRO_XOUT  0x43
#define MPU6050_ACCEL_XOUT 0x3B


/* store i2c instance for read and write */
static i2c_inst_t *mpu6050_i2c = NULL;

void mpu6050_init(i2c_inst_t *i2c,
	uint		      sda,
	uint		      scl,
	uint8_t		      gyro_fs,
	uint8_t		      accel_fs,
	uint		      baudrate)
{
	mpu6050_i2c = i2c;


	i2c_init(mpu6050_i2c, baudrate);
	gpio_set_function(sda, GPIO_FUNC_I2C);
	gpio_set_function(scl, GPIO_FUNC_I2C);
	gpio_pull_up(sda);
	gpio_pull_up(scl);


	/* wake up */
	uint8_t buffer[2];
	buffer[0] = MPU6050_PWR_MGMT_1;
	buffer[1] = 0x00;  // clear sleeping bit
	int ret	  = i2c_write_blocking(
		  mpu6050_i2c, MPU6050_I2C_ADDR, buffer, 2, false);
	if (ret != 2) {
		printf("Error: Failed to wake up. Expected 2 bytes, got %d.\n",
			ret);
	}


	/* full scale config */
	uint8_t gyro_config_buf[2];
	gyro_config_buf[0] = MPU6050_GYRO_CONFIG;
	gyro_config_buf[1] = gyro_fs << 3; /* uses bits 4,3 */
	ret		   = i2c_write_blocking(
		       mpu6050_i2c, MPU6050_I2C_ADDR, gyro_config_buf, 2, false);
	if (ret != 2) {
		printf("Error: Failed to configure gyro. Expected 2 bytes, got %d.\n",
			ret);
	}


	uint8_t accel_config_buf[2];
	accel_config_buf[0] = MPU6050_ACCEL_CONFIG;
	accel_config_buf[1] = accel_fs << 3;
	ret		    = i2c_write_blocking(
		mpu6050_i2c, MPU6050_I2C_ADDR, accel_config_buf, 2, false);
	if (ret != 2) {
		printf("Error: Failed to configure accel. Expected 2 bytes, got %d.\n",
			ret);
	}
}

void mpu6050_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t gyro_reg = MPU6050_GYRO_XOUT;
	uint8_t data[6];

	int ret = i2c_write_blocking(
		mpu6050_i2c, MPU6050_I2C_ADDR, &gyro_reg, 1, true);
	if (ret != 1) {
		printf("Error: Failed to write gyro address. Expected 1 bytes, got %d.\n",
			ret);
	}

	ret = i2c_read_blocking(mpu6050_i2c, MPU6050_I2C_ADDR, data, 6, false);
	if (ret != 6) {
		printf("Error: Failed to read gyro data. Expected 6 bytes, got %d.\n",
			ret);
	}

	*gx = (int16_t) ((data[0] << 8) | data[1]);
	*gy = (int16_t) ((data[2] << 8) | data[3]);
	*gz = (int16_t) ((data[4] << 8) | data[5]);
}

void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t accel_reg = MPU6050_ACCEL_XOUT;
	uint8_t data[6];

	int ret = i2c_write_blocking(
		mpu6050_i2c, MPU6050_I2C_ADDR, &accel_reg, 1, true);
	if (ret != 1) {
		printf("Error: Failed to write accel address. Expected 1 bytes, got %d.\n",
			ret);
	}

	ret = i2c_read_blocking(mpu6050_i2c, MPU6050_I2C_ADDR, data, 6, false);
	if (ret != 6) {
		printf("Error: Failed to read accel data. Expected 6 bytes, got %d.\n",
			ret);
	}

	*ax = (int16_t) ((data[0] << 8) | data[1]);
	*ay = (int16_t) ((data[2] << 8) | data[3]);
	*az = (int16_t) ((data[4] << 8) | data[5]);
}
