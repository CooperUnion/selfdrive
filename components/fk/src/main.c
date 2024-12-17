#include <mpu6050.h>
#include <stddef.h>
#include <stdio.h>

int app_main(void)
{
	mpu6050_handle_t mpu6050_handle;
	mpu6050_config_t mpu6050_config = {
		.fs  = FS_SEL_500,
		.afs = AFS_SEL_4G,
		.scl = (gpio_num_t) (40),
		.sda = (gpio_num_t) (37),
	};

	mpu6050_init(&mpu6050_handle, &mpu6050_config);

	mpu6050_raw_gyro_t raw_gyro;

	float x, y, z;
	for (;;) {
		mpu6050_get_raw_gyro(&mpu6050_handle, &raw_gyro);
		x = raw_gyro.gyro_raw_xout / 65.5;
		y = raw_gyro.gyro_raw_yout / 65.5;
		z = raw_gyro.gyro_raw_zout / 65.5;
		printf("%f %f %f\n", x, y, z);
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	return EXIT_SUCCESS;
}
