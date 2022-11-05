#ifndef EMBER_CAN_H
#define EMBER_CAN_H

#include <driver/twai.h>

#include "can_gen.h"
#include "common.h"

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
    uint64_t delta_ms;
    bool recieved;
} can_incoming_t;

void can_register_incoming_msg(can_incoming_t *cfg);
void can_send_iface(const can_outgoing_t *i, const void *s);

#endif
