#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <libsocketcan.h>
#include "core.h"
#include "mod_canhealth.h"

// impl. functions in core to kick watchdog

void mod_canhealth(const int THIS_THREAD)
{
    int canState;
    LOGMSG(mod_canhealth, 0, "Running");
    for (;;)
    {
        if (can_get_state("can0", &canState) < 0) {
            LOGMSG(mod_canhealth, 1, "Error getting CAN state!");
            core::estop();
        }
        if (canState != CAN_STATE_ERROR_ACTIVE) {
            LOGMSG(mod_canhealth, 1, "CAN UNHEALTHY (state %d), calling estop()", canState);
            core::estop();
        }
        core::status[THIS_THREAD]++;
        usleep(1000);
    }
}
