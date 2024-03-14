#ifndef PEDAL_H
#define PEDAL_H

#include <ember_common.h>

float32_t current_pedal_percent(void);
uint32_t  current_thr_A_dutyCycle(void);
uint32_t  current_thr_F_dutyCycle(void);
void	  enable_pedal_output();
void	  set_pedal_output(float32_t cmd);

#endif	// PEDAL_H
