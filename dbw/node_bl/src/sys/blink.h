#ifndef BLINK_H
#define BLINK_H

#include <esp_err.h>

esp_err_t blink_init(void);
esp_err_t blink_pulse(void);

#endif
