#ifndef BL_H
#define BL_H

#include <esp_err.h>

esp_err_t bl_init(void);
void bl_restart(void);

#endif
