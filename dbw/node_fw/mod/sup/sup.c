#include "sup.h"

#include "common.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_tx.h"


// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static bool brake_authorized;
static bool throttle_authorized;
static bool steering_authorized;

// ######          CAN          ###### //

// ######    RATE FUNCTIONS     ###### //
static void sup_100Hz();

ember_rate_funcs_S module_rf = {
    .call_100Hz = sup_100Hz,
};

static void sup_100Hz()
{
    bool authorized;

    // BRAKE
    authorized  = true;
    authorized &= CANRX_is_message_DBW_VelocityCommand_ok();
    authorized &= CANRX_is_node_CTRL_ok();
    authorized &= !CANRX_get_CTRL_speedAlarm();
    brake_authorized = authorized;

    // THROTTLE
    // NOTE: at the moment, BRAKE & THROTTLE
    // consist of the same safety checks
    throttle_authorized = brake_authorized;
}

// ######        CAN TX         ###### //


void CANTX_populate_SUP_SupervisorAuthorized(
        struct CAN_Message_SUP_SupervisorAuthorized * const m)
{
    m->SUP_brakeAuthorized    = brake_authorized;
    m->SUP_throttleAuthorized = throttle_authorized;
    m->SUP_steeringAuthorized = steering_authorized;
}
