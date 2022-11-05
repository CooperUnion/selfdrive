/*
 * safety.c -- encoder module safety
 */

#include "base/base.h"
#include "io/can.h"
#include "sys/task_glue.h"

#include "encoder.h"

// ######        DEFINES        ###### //

#define ENCODER_MAX_TICKS  85     // slightly over 5MPH
#define ENCODER_TIMEOUT_US 20000

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

// ######    RATE FUNCTIONS     ###### //

static void safety_100Hz(void);

struct rate_funcs safety_rf = {
    .call_100Hz = safety_100Hz,
};

static void safety_100Hz()
{
    if (!base_dbw_active()) return;

    if (
        (ABS(CAN_Encoder.encoderLeft) >= ENCODER_MAX_TICKS) ||
        (ABS(CAN_Encoder.encoderRight) >= ENCODER_MAX_TICKS)
    )
        base_set_state_estop(
            CAN_DBW_ESTOP_reason_LIMIT_EXCEEDED_CHOICE,
            __LINE__
        );

    if (CAN_Encoder.dtUs >= ENCODER_TIMEOUT_US)
        base_set_state_estop(CAN_DBW_ESTOP_reason_TIMEOUT_CHOICE, __LINE__);
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
