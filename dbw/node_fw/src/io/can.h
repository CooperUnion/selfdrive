#ifndef CAN_H
#define CAN_H

#include <driver/twai.h>

#include "common.h"
#include "io/can_gen.h"

typedef struct can_outgoing_t {
    uint32_t id;
    uint8_t dlc;
    bool extd;
    int (*pack)();
} can_outgoing_t;

typedef struct can_incoming_t {
    uint32_t id;
    void *out;
    int (*unpack)();
    uint32_t *delta_ms;
} can_incoming_t;

void can_register_incoming_msg(const can_incoming_t cfg);
void can_send_iface(const can_outgoing_t *i, const void *s);

#endif
