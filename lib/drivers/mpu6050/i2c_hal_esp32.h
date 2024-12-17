#ifndef I2C_HAL_ESP32
#define I2C_HAL_ESP32

#include "i2c_hal.h"
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>

struct i2c_hal_handle_s {
	i2c_master_dev_handle_t dev_handle;
};

struct i2c_hal_config_s {
	uint8_t	 device_address;
	uint32_t scl_speed_hz;
	uint8_t	 scl;
	uint8_t	 sda;
};

void i2c_hal_init(
	i2c_hal_handle_t *i2c_hal_handle, i2c_hal_config_t *i2c_hal_config);
void i2c_hal_read(i2c_hal_handle_t *i2c_hal_handle,
	const uint8_t		    addr,
	uint8_t			   *data,
	uint32_t		    num_bytes);
void i2c_hal_write(i2c_hal_handle_t *i2c_hal_handle,
	const uint8_t		     addr,
	uint8_t			    *data,
	uint32_t		     num_bytes);

#endif	// I2C_HAL_ESP32
