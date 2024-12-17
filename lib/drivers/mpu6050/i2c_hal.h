#ifndef I2C_HAL_H
#define I2C_HAL_H

#include <stdint.h>

typedef struct i2c_hal_handle_s i2c_hal_handle_t;
typedef struct i2c_hal_config_s i2c_hal_config_t;

typedef struct {
	void (*i2c_hal_init)(i2c_hal_handle_t *i2c_hal_handle,
		i2c_hal_config_t	      *i2c_hal_config);
	void (*i2c_hal_read)(i2c_hal_handle_t *i2c_hal_handle,
		const uint8_t		       addr,
		uint8_t			      *data,
		uint32_t		       num_bytes);
	void (*i2c_hal_write)(i2c_hal_handle_t *i2c_hal_handle,
		const uint8_t			addr,
		uint8_t			       *data,
		uint32_t			num_bytes);
} i2c_hal_t;

extern i2c_hal_t i2c_hal;

#endif	// I2C_HAL_H
