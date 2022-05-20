#ifndef COMMON_H
#define COMMON_H

/*
 * COMMON TYPES
 */
#include <esp_err.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef unsigned int uint;
typedef float float32_t;

/*
 * COMMON MACROS
 */
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define ABS(X) ((X < 0) ? (X * -1) : (X))

#endif
