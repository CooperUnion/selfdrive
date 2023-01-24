#ifndef PEDAL_H
#define PEDAL_H

#include "common.h"

void enable_pedal_output();
void set_pedal_output(float32_t cmd);
float32_t current_pedal_percent(void);

#endif
