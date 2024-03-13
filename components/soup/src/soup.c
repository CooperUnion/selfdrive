#include "soup.h"

#include "firmware-base/state-machine.h"
#include <ember_taskglue.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <opencan_rx.h>
#include <opencan_templates.h>
#include <opencan_tx.h>

static void soup_100Hz();

static bool bbc_authorized;
static bool gas_authorized;
static bool skrt_authorized;

ember_rate_funcs_S module_rf = {
    .call_100Hz = soup_100Hz,
};

static void soup_100Hz()
{
	bool authorized;

	// BBC
	authorized = true;
	taskDISABLE_INTERRUPTS();
	authorized &= CANRX_is_message_DBW_VelocityCommand_ok()
	    || CANRX_is_message_DBW_RawVelocityCommand_ok();
	authorized &= CANRX_is_node_OM_ok();
	authorized &= CANRX_is_node_BBC_ok();
	authorized
	    &= CANRX_get_OM_sysStatus() != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	authorized &= CANRX_get_BBC_sysStatus()
	    != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	authorized &= !CANRX_get_OM_speedAlarm();
	taskENABLE_INTERRUPTS();
	bbc_authorized = authorized;

	// SKRT
	authorized = true;
	taskDISABLE_INTERRUPTS();
	authorized &= CANRX_is_message_DBW_SteeringCommand_ok();
	authorized &= CANRX_get_SKRT_sysStatus()
	    != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	taskENABLE_INTERRUPTS();
	skrt_authorized = authorized;

	// GAS
	authorized = true;
	taskDISABLE_INTERRUPTS();
	authorized &= CANRX_is_message_DBW_VelocityCommand_ok()
	    || CANRX_is_message_DBW_RawVelocityCommand_ok();
	authorized &= CANRX_is_node_OM_ok();
	authorized &= CANRX_is_node_GAS_ok();
	authorized
	    &= CANRX_get_OM_sysStatus() != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	authorized &= CANRX_get_GAS_sysStatus()
	    != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	authorized &= CANRX_get_SKRT_state() == CAN_SKRT_STATE_READY;
	authorized &= !CANRX_get_OM_speedAlarm();
	taskENABLE_INTERRUPTS();
	gas_authorized = authorized;

	if (bbc_authorized || skrt_authorized || gas_authorized) {
		base_request_state(SYS_STATE_DBW_ACTIVE);
	} else {
		base_request_state(SYS_STATE_IDLE);
	}
}

void CANTX_populate_SOUP_Authorization(
    struct CAN_Message_SOUP_Authorization * const m)
{
	m->SOUP_bbcAuthorized  = bbc_authorized;
	m->SOUP_gasAuthorized  = gas_authorized;
	m->SOUP_skrtAuthorized = skrt_authorized;
}
