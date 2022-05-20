#ifndef TIMING_H
#define TIMING_H

#include <driver/timer.h>
#include <esp_err.h>

#define TIMING_GROUP TIMER_GROUP_1
#define US_TIMER     TIMER_0

esp_err_t timing_init(void);

#endif
