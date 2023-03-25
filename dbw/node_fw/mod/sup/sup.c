#include "sup.h"

#include "common.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_templates.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static bool brake_authorized;
static bool throttle_authorized;
static bool steer_authorized;

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
    authorized &= CANRX_is_node_BRAKE_ok();
    authorized &= CANRX_get_CTRL_sysStatus()  != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    authorized &= CANRX_get_BRAKE_sysStatus() != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    authorized &= !CANRX_get_CTRL_speedAlarm();
    brake_authorized = authorized;

    // STEER
    authorized  = true;
    authorized &= CANRX_is_message_DBW_VelocityCommand_ok();
    authorized &= CANRX_get_STEER_sysStatus() != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    steer_authorized = authorized;

    // THROTTLE
    authorized  = true;
    authorized &= CANRX_is_message_DBW_VelocityCommand_ok();
    authorized &= CANRX_is_node_CTRL_ok();
    authorized &= CANRX_is_node_THROTTLE_ok();
    authorized &= CANRX_get_CTRL_sysStatus()     != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    authorized &= CANRX_get_THROTTLE_sysStatus() != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    authorized &= !CANRX_get_CTRL_speedAlarm();
    throttle_authorized = authorized;
}

// ######        CAN TX         ###### //

void CANTX_populate_SUP_SupervisorAuthorized(
        struct CAN_Message_SUP_SupervisorAuthorized * const m)
{
    m->SUP_brakeAuthorized    = brake_authorized;
    m->SUP_throttleAuthorized = throttle_authorized;
    m->SUP_steerAuthorized    = steer_authorized;
}
