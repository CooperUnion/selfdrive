/*
 * safety.c -- brake module safety
 */

#include "base/base.h"
#include "io/can.h"
#include "sys/task_glue.h"

#include "brake.h"

// ######        DEFINES        ###### //

#define CMD_TIMEOUT_MS 200

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

// ######    RATE FUNCTIONS     ###### //

static void safety_100Hz(void);

struct rate_funcs safety_rf = {
    .call_init = safety_100Hz,
};

static void safety_100Hz()
{
    if (!base_dbw_active()) return;

    if (!can_Vel_Cmd_cfg.recieved) {
        static uint64_t prv_delta_ms;
        static bool     set;

        if (set) {
            if (can_Vel_Cmd_cfg.delta_ms - prv_delta_ms >= CMD_TIMEOUT_MS)
                base_set_state_estop(CAN_dbwESTOP_Reason_TIMEOUT_CHOICE);
        } else {
            prv_delta_ms = can_Vel_Cmd_cfg.delta_ms;
            set = true;
        }

        return;
    }

    if (can_Vel_Cmd_cfg.delta_ms >= CMD_TIMEOUT_MS)
        base_set_state_estop(CAN_dbwESTOP_Reason_TIMEOUT_CHOICE);
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
