#include <mpu6050.h>
#include <stddef.h>
#include <stdio.h>

int app_main(void)
{
	mpu6050_init();
	return EXIT_SUCCESS;
}
