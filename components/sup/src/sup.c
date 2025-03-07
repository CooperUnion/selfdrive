#include "sup.h"

#include "driver/gpio.h"
#include "firmware-base/state-machine.h"
#include <ember_taskglue.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <opencan_rx.h>
#include <opencan_templates.h>
#include <opencan_tx.h>

#define LED_PIN 2

static void blink_led_10hz();
static void init_led();
static void sup_100Hz();

static bool bbc_authorized;
static bool throttle_authorized;
static bool steer_authorized;

typedef enum {
	SEG_A  = 13,
	SEG_B  = 4,
	SEG_C  = 7,
	SEG_D  = 8,
	SEG_E  = 9,
	SEG_F  = 12,
	SEG_G  = 5,
	SEG_DP = 11
} SEVEN_SEG_PINS;

ember_rate_funcs_S module_rf = {
	.call_init  = init_led,
	.call_10Hz  = blink_led_10hz,
	.call_100Hz = sup_100Hz,
};

static void init_led()
{
	gpio_pad_select_gpio(LED_PIN);
	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

static void sup_100Hz()
{
	bool authorized;

	// BBC
	authorized = true;
	taskDISABLE_INTERRUPTS();
	authorized &= CANRX_is_message_DBW_VelocityCommand_ok()
		|| CANRX_is_message_DBW_RawVelocityCommand_ok();
	authorized &= CANRX_is_node_CTRL_ok();
	authorized &= CANRX_is_node_BBC_ok();
	authorized &= CANRX_get_CTRL_sysStatus()
		!= CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	authorized &= CANRX_get_BBC_sysStatus()
		!= CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	authorized &= !CANRX_get_CTRL_speedAlarm();
	taskENABLE_INTERRUPTS();
	bbc_authorized = authorized;

	// STEER
	authorized = true;
	taskDISABLE_INTERRUPTS();
	authorized &= CANRX_is_message_DBW_SteeringCommand_ok();
	authorized &= CANRX_get_STEER_sysStatus()
		!= CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	taskENABLE_INTERRUPTS();
	steer_authorized = authorized;

	// THROTTLE
	authorized = true;
	taskDISABLE_INTERRUPTS();
	authorized &= CANRX_is_message_DBW_VelocityCommand_ok()
		|| CANRX_is_message_DBW_RawVelocityCommand_ok();
	authorized &= CANRX_is_node_CTRL_ok();
	authorized &= CANRX_is_node_THROTTLE_ok();
	authorized &= CANRX_get_CTRL_sysStatus()
		!= CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	authorized &= CANRX_get_THROTTLE_sysStatus()
		!= CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
	authorized &= CANRX_get_STEER_state() == CAN_STEER_STATE_READY;
	authorized &= !CANRX_get_CTRL_speedAlarm();
	taskENABLE_INTERRUPTS();
	throttle_authorized = authorized;

	if (bbc_authorized || steer_authorized || throttle_authorized) {
		base_request_state(SYS_STATE_DBW_ACTIVE);
	} else {
		base_request_state(SYS_STATE_IDLE);
	}
}

void CANTX_populate_SUP_Authorization(
	struct CAN_Message_SUP_Authorization * const m)
{
	m->SUP_bbcAuthorized	  = bbc_authorized;
	m->SUP_throttleAuthorized = throttle_authorized;
	m->SUP_steerAuthorized	  = steer_authorized;
}
