#ifndef BASE_H
#define BASE_H

#include <stdbool.h>

bool base_dbw_currently_active();
void base_set_state_lost_can();
void base_set_state_bl();
void base_set_state_estop();
void base_set_wdt_trigger();

#endif
