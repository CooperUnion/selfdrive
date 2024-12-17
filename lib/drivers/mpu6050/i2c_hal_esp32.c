#include "i2c_hal_esp32.h"

#include "i2c_hal.h"

#include <string.h>

#define I2C_MASTER_PORT I2C_NUM_0

#define I2C_TIMEOUT_MS 500

i2c_hal_t i2c_hal = {
	.i2c_hal_init  = i2c_hal_init,
	.i2c_hal_read  = i2c_hal_read,
	.i2c_hal_write = i2c_hal_write,
};

void i2c_hal_init(
	i2c_hal_handle_t *i2c_hal_handle, i2c_hal_config_t *i2c_hal_config)
{
	i2c_master_bus_config_t i2c_master_conf = {
		.clk_source	   = I2C_CLK_SRC_DEFAULT,
		.i2c_port	   = I2C_MASTER_PORT,
		.scl_io_num	   = i2c_hal_config->scl,
		.sda_io_num	   = i2c_hal_config->sda,
		.glitch_ignore_cnt = 7,
	};

	i2c_master_bus_handle_t bus_handle;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_conf, &bus_handle));

	i2c_device_config_t i2c_dev_conf = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,

		// should probably make a check that device_address &
		// scl_speed_hz is configured properly before we continue
		.device_address = i2c_hal_config->device_address,
		.scl_speed_hz	= i2c_hal_config->scl_speed_hz,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(
		bus_handle, &i2c_dev_conf, &(i2c_hal_handle->dev_handle)));
}

void i2c_hal_read(i2c_hal_handle_t *i2c_hal_handle,
	const uint8_t		    addr,
	uint8_t			   *data,
	uint32_t		    num_bytes)
{
	i2c_master_transmit_receive(i2c_hal_handle->dev_handle,
		&addr,
		1,
		data,
		num_bytes,
		I2C_TIMEOUT_MS);
}

void i2c_hal_write(i2c_hal_handle_t *i2c_hal_handle,
	const uint8_t		     addr,
	uint8_t			    *data,
	uint32_t		     num_bytes)
{
	uint8_t tx[num_bytes + 1];
	tx[0] = addr;

	memcpy(tx + 1, data, num_bytes);

	i2c_master_transmit(
		i2c_hal_handle->dev_handle, tx, sizeof(tx), I2C_TIMEOUT_MS);
}
