#include "sup.h"

#include "driver/gpio.h"
#include "firmware-base/state-machine.h"
#include "soc/gpio_num.h"
#include <ember_taskglue.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <opencan_rx.h>
#include <opencan_templates.h>
#include <opencan_tx.h>

typedef enum {
	PIN_SEG_A = GPIO_NUM_13,
	PIN_SEG_B = GPIO_NUM_4,
	PIN_SEG_C = GPIO_NUM_7,
	PIN_SEG_D = GPIO_NUM_8,
	PIN_SEG_E = GPIO_NUM_9,
	PIN_SEG_F = GPIO_NUM_12,
	PIN_SEG_1 = GPIO_NUM_1,
	PIN_SEG_2 = GPIO_NUM_2,
	PIN_SEG_3 = GPIO_NUM_5,
} seven_segment_pin_t;

static void display_authorization();
static void init_led();
static void sup_100Hz();
static void init_pin(seven_segment_pin_t pin);
static void set_pins(char bitfield);


static bool bbc_authorized;
static bool throttle_authorized;
static bool steer_authorized;

static const char ZERO = 0x3F;
static const char ONE  = 0x06;

static enum {
	BBC_STATE,
	THROTTLE_STATE,
	STEER_STATE,
	STATES_TOTAL
} CURRENT_STATE = BBC_STATE;

gpio_config_t pin_init = {.mode = GPIO_MODE_OUTPUT,
	.pin_bit_mask		= (1ULL << PIN_SEG_A) | (1ULL << PIN_SEG_B)
		| (1ULL << PIN_SEG_C) | (1ULL << PIN_SEG_D)
		| (1ULL << PIN_SEG_E) | (1ULL << PIN_SEG_F)
		| (1ULL << PIN_SEG_1) | (1ULL << PIN_SEG_2)
		| (1ULL << PIN_SEG_3),
	.pull_down_en = 0,
	.pull_up_en   = 0,
	.intr_type    = GPIO_INTR_DISABLE};

static void init_led()
{
	gpio_config(&pin_init);
	set_pins(0x3F);
	gpio_set_level(PIN_SEG_1, 1);
	gpio_set_level(PIN_SEG_2, 1);
	gpio_set_level(PIN_SEG_3, 1);
}

static void set_pins(char bitfield)
{
	gpio_set_level(PIN_SEG_A, !((bitfield >> 0) & 1));
	gpio_set_level(PIN_SEG_B, !((bitfield >> 1) & 1));
	gpio_set_level(PIN_SEG_C, !((bitfield >> 2) & 1));
	gpio_set_level(PIN_SEG_D, !((bitfield >> 3) & 1));
	gpio_set_level(PIN_SEG_E, !((bitfield >> 4) & 1));
	gpio_set_level(PIN_SEG_F, !((bitfield >> 5) & 1));
}

static void display_authorization()
{
	if (CURRENT_STATE == BBC_STATE) {
		if (bbc_authorized) {
			// if bbc is authorized, show 0
			set_pins(ZERO);
		} else {
			// if not authorized, show 1
			set_pins(ONE);
		}
		gpio_set_level(PIN_SEG_1, 1);
		gpio_set_level(PIN_SEG_2, 0);
		gpio_set_level(PIN_SEG_3, 0);
	} else if (CURRENT_STATE == THROTTLE_STATE) {
		if (throttle_authorized) {
			set_pins(ZERO);
		} else {
			set_pins(ONE);
		}
		gpio_set_level(PIN_SEG_1, 0);
		gpio_set_level(PIN_SEG_2, 1);
		gpio_set_level(PIN_SEG_3, 0);
	} else if (CURRENT_STATE == STEER_STATE) {
		if (steer_authorized) {
			set_pins(ZERO);
		} else {
			set_pins(ONE);
		}
		gpio_set_level(PIN_SEG_1, 0);
		gpio_set_level(PIN_SEG_2, 0);
		gpio_set_level(PIN_SEG_3, 1);
	}
	CURRENT_STATE = (CURRENT_STATE + 1) % STATES_TOTAL;
}

ember_rate_funcs_S module_rf = {
	.call_init  = init_led,
	.call_1kHz  = display_authorization,
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
