#include "sup.h"

#include "driver/gpio.h"
#include "firmware-base/state-machine.h"
#include <ember_taskglue.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <opencan_rx.h>
#include <opencan_templates.h>
#include <opencan_tx.h>

typedef enum {
	SEG_A  = 13,
	SEG_B  = 4,
	SEG_C  = 7,
	SEG_D  = 8,
	SEG_E  = 9,
	SEG_F  = 12,
	SEG_G  = 5,
	SEG_DP = 11,
	SEG_1  = 1,
	SEG_2  = 2,
	SEG_3  = 3,
	SEG_4  = 6,
} SEVEN_SEG_PINS;

static void bts_authorization();
static void init_led();
static void sup_100Hz();
static void init_pin(SEVEN_SEG_PINS pin);
static void set_one();
static void set_zero();


static bool bbc	     = 0;
static bool throttle = 0;
static bool steer    = 0;
static bool bbc_authorized;
static bool throttle_authorized;
static bool steer_authorized;


static const SEVEN_SEG_PINS zero[]
	= {SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F};

static const SEVEN_SEG_PINS one[] = {SEG_B, SEG_C};

static enum {
	bbc_state,
	throttle_state,
	steer_state
} current_state = bbc_state;

static void init_pin(SEVEN_SEG_PINS pin)
{
	gpio_pad_select_gpio(pin);
	gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

static void init_led()
{
	init_pin(SEG_A);
	init_pin(SEG_B);
	init_pin(SEG_C);
	init_pin(SEG_D);
	init_pin(SEG_E);
	init_pin(SEG_F);
	init_pin(SEG_G);
	init_pin(SEG_DP);
	init_pin(SEG_1);
	init_pin(SEG_2);
	init_pin(SEG_3);
	init_pin(SEG_4);

	// set default state to 0

	set_zero();
	gpio_set_level(SEG_1, 1);
	gpio_set_level(SEG_2, 1);
	gpio_set_level(SEG_3, 1);
}

static void set_one()
{
	for (int i = 0; i < 6; i++) {
		gpio_set_level(zero[i], 1);
	}
	for (int i = 0; i < 2; i++) {
		gpio_set_level(one[i], 0);
	}
	gpio_set_level(SEG_G, 1);
}

static void set_zero()
{
	for (int i = 0; i < 6; i++) {
		gpio_set_level(zero[i], 0);  // active low
	}
	gpio_set_level(SEG_G, 1);
}

static void bts_authorization()
{
	if (current_state == bbc_state) {
		if (bbc_authorized) {
			// if bbc is authorized, show 0
			set_zero();
		} else {
			// if not authorized, show 1
			set_one();
		}
		gpio_set_level(SEG_1, 1);
		gpio_set_level(SEG_2, 0);
		gpio_set_level(SEG_3, 0);
	} else if (current_state == throttle_state) {
		if (throttle_authorized) {
			set_zero();
		} else {
			set_one();
		}
		gpio_set_level(SEG_1, 0);
		gpio_set_level(SEG_2, 1);
		gpio_set_level(SEG_3, 0);
	} else if (current_state == steer_state) {
		if (steer_authorized) {
			set_zero();
		} else {
			set_one();
		}
		gpio_set_level(SEG_1, 0);
		gpio_set_level(SEG_2, 0);
		gpio_set_level(SEG_3, 1);
	}
	current_state = (current_state + 1) % 3;
}

ember_rate_funcs_S module_rf = {
	.call_init  = init_led,
	.call_1Hz   = bts_authorization,
	.call_100Hz = sup_100Hz,
};

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
