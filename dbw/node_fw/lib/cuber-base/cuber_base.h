#ifndef CUBER_BASE_H
#define CUBER_BASE_H

#include <stdbool.h>
#include <stdint.h>

void base_set_state_idle(void);
void base_set_state_dbw_active(void);
void base_set_state_estop(void);
void base_set_wdt_trigger(void);

#endif
