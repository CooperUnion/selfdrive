#ifndef CAN_H
#define CAN_H

#include <driver/timer.h>
#include <esp_err.h>

#include "io/can_gen.h"

#define CAN_TIMER_GROUP TIMER_GROUP_1
#define CAN_TIMER       TIMER_0

typedef struct can_outgoing_s {
    uint32_t id;
    uint8_t dlc;
    bool extd;
    int (*pack)();
} can_outgoing_t;

typedef struct can_incoming_s {
    uint32_t id;
    void *out;
    int (*unpack)();
    uint64_t *timer_val;
} can_incoming_t;

esp_err_t can_init(void);
esp_err_t can_poll(void);
void can_register_incoming_msg(const can_incoming_t cfg);
esp_err_t can_send_iface(const can_outgoing_t *iface, const void *src);

#endif
