#include "steer.h"

#include <esp_attr.h>
#include <driver/gpio.h>

#include "cuber_base.h"
#include "ember_common.h"
#include "ember_taskglue.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

// ######    RATE FUNCTIONS     ###### //

static void steer_init();

ember_rate_funcs_S module_rf = {
    .call_init = steer_init,
};

static void steer_init()
{
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //

void CANTX_populate_STEER_SteeringData(struct CAN_Message_STEER_SteeringData * const m)
{
    m->STEER_angle             = 0;
    m->STEER_encoderTimeoutSet = 0;
    m->STEER_oDriveConnected   = 0;
}
