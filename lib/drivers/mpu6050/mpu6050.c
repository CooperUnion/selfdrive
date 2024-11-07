#include "mpu6050.h"

#include <esp_log.h>

#define I2C_MASTER_PORT	  I2C_NUM_0
#define I2C_MASTER_SCL_IO (gpio_num_t)(40)
#define I2C_MASTER_SDA_IO (gpio_num_t)(37)


#define I2C_MPU6050_DEV_ADDR	0x68  // AD0
#define I2C_MPU6050_DEV_FREQ_HZ 400000

#define I2C_MPU6050_WHO_AM_I	0x75
#define I2C_MPU6050_GYRO_CONFIG 0x1B

#define I2C_MPU6050_DEV_ID 0x68

#define I2C_TIMEOUT_MS 500

typedef enum {
	FS_SEL_250,
	FS_SEL_500,
	FS_SEL_1000,
	FS_SEL_2000
} i2c_mpu6050_fs_sel_t;

typedef enum {
	AFS_SEL_2G,
	AFS_SEL_4G,
	AFS_SEL_8G,
	AFS_SEL_16G
} i2c_mpu6050_afs_sel_t;

static const char *TAG = "MPU6050.c";

static void i2c_mpu6050_read(i2c_master_dev_handle_t i2c_dev,
	uint8_t					     addr,
	uint8_t					    *data,
	size_t					     num_bytes)
{
	i2c_master_transmit_receive(
		i2c_dev, &addr, 1, data, num_bytes, I2C_TIMEOUT_MS);
}

static void i2c_mpu6050_write(
	i2c_master_dev_handle_t i2c_dev, uint8_t *data, size_t num_bytes)
{
	i2c_master_transmit(i2c_dev, data, num_bytes, I2C_TIMEOUT_MS);
}

void mpu6050_config(i2c_master_dev_handle_t i2c_dev,
	i2c_mpu6050_fs_sel_t		    fs_sel,
	i2c_mpu6050_afs_sel_t		    afs_sel)
{
	uint8_t data[2] = {fs_sel << 3, afs_sel << 3};
	i2c_mpu6050_write(i2c_dev, data, 2);
}

void mpu6050_init(void)
{
	i2c_master_bus_config_t i2c_master_conf = {
		.clk_source	   = I2C_CLK_SRC_DEFAULT,
		.i2c_port	   = I2C_NUM_0,
		.scl_io_num	   = I2C_MASTER_SCL_IO,
		.sda_io_num	   = I2C_MASTER_SDA_IO,
		.glitch_ignore_cnt = 7,
	};

	i2c_master_bus_handle_t bus_handle;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_conf, &bus_handle));

	i2c_device_config_t i2c_dev_conf = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address	 = I2C_MPU6050_DEV_ADDR,
		.scl_speed_hz	 = I2C_MPU6050_DEV_FREQ_HZ,
	};

	i2c_master_dev_handle_t dev_handle;

	ESP_ERROR_CHECK(i2c_master_bus_add_device(
		bus_handle, &i2c_dev_conf, &dev_handle));

	// Check Dev ID
	uint8_t dev_id;
	i2c_mpu6050_read(dev_handle, I2C_MPU6050_WHO_AM_I, &dev_id, 1);

	if (dev_id != (uint8_t) I2C_MPU6050_DEV_ID) {
		ESP_LOGE(TAG, "Failure @ dev id: %x\n", dev_id);
		return;
	}
	ESP_LOGI(TAG, "MPU6050 RECOGNIZED!\n");

	// Config GYRO & ACCEL
	mpu6050_config(dev_handle, FS_SEL_500, AFS_SEL_4G);
	ESP_LOGI(TAG, "GRYO & ACCEL CONFIG FINISHED!\n");
}
