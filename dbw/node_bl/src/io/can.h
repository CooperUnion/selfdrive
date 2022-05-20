#ifndef CAN_H
#define CAN_H

#include <driver/timer.h>
#include <esp_err.h>

#include "io/can_gen.h"

#define CAN_TIMER_GROUP TIMER_GROUP_1
#define CAN_TIMER       TIMER_0

esp_err_t can_init(void);

#endif
