#ifndef PEDAL_H
#define PEDAL_H

#include "ember_common.h"

void enable_pedal_output();
void set_pedal_output(float32_t cmd);
float32_t current_pedal_percent(void);
uint32_t current_thr_a_dutyCycle(void);
uint32_t current_thr_f_dutyCycle(void);

#endif