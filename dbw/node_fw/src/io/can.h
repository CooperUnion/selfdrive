#ifndef CAN_H
#define CAN_H

#include <driver/twai.h>

void can_send_msg(const twai_message_t *message);

#endif
