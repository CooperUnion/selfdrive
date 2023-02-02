#ifndef CUBER_BASE_H
#define CUBER_BASE_H

#include <stdbool.h>
#include <stdint.h>

void base_set_state_lost_can();
void base_set_state_estop(uint8_t choice);
void base_set_wdt_trigger();
bool base_estoped();

#endif
