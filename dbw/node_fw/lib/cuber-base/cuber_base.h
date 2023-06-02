#ifndef CUBER_BASE_H
#define CUBER_BASE_H

#include <stdbool.h>
#include <stdint.h>

enum cuber_sys_states {
    CUBER_SYS_STATE_UNDEF = 0,
    CUBER_SYS_STATE_INIT,
    CUBER_SYS_STATE_IDLE,
    CUBER_SYS_STATE_DBW_ACTIVE,
    CUBER_SYS_STATE_LOST_CAN,
    CUBER_SYS_STATE_BAD,
    CUBER_SYS_STATE_ESTOP,
};

enum cuber_sys_states base_get_state(void);
void base_request_state(enum cuber_sys_states state);

#endif
