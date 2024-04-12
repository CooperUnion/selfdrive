#include "throttle.h"

#include "firmware-base/state-machine.h"
#include "pedal.h"
#include <driver/gpio.h>
#include <ember_common.h>
#include <ember_taskglue.h>
#include <opencan_rx.h>
#include <opencan_tx.h>

#define MODE_OM_PIN GPIO_NUM_40

static bool relay_state;

static void control_relay(bool cmd);
static void throttle_init();
static void throttle_100Hz();

ember_rate_funcs_S module_rf = {
	.call_init  = throttle_init,
	.call_100Hz = throttle_100Hz,
};

/*
 * Initialize the mode control pin as a GPIO and enable the DACs.
 *
 * Note that the DACs are on, but do not have valid throttle output levels set.
 *
 * The relay is currently closed. We must set valid levels before closing it.
 */
static void throttle_init()
{
	gpio_config(&(gpio_config_t){
		.pin_bit_mask = BIT64(MODE_OM_PIN),
		.mode	      = GPIO_MODE_OUTPUT,
	});

	control_relay(0);

	enable_pedal_output();
}

static void throttle_100Hz()
{
	bool throttle_authorized = CANRX_is_message_SUP_Authorization_ok()
		&& CANRX_get_SUP_throttleAuthorized()
		&& CANRX_is_message_CTRL_VelocityCommand_ok();

	float32_t cmd;

	if (throttle_authorized) {
		cmd = ((float32_t) CANRX_get_CTRL_throttlePercent()) / 100.0;
		base_request_state(SYS_STATE_DBW_ACTIVE);
	} else {
		cmd = 0.0;
		base_request_state(SYS_STATE_IDLE);
	}

	control_relay(throttle_authorized);
	set_pedal_output(cmd);
}

/*
 * Open or close the relay.
 */
static void control_relay(bool cmd)
{
	gpio_set_level(MODE_OM_PIN, cmd);
	relay_state = cmd;
}

void CANTX_populate_THROTTLE_AccelData(
	struct CAN_Message_THROTTLE_AccelData * const m)
{
	m->THROTTLE_throttleADutyCycle = current_thr_A_dutyCycle();
	m->THROTTLE_throttleFDutyCycle = current_thr_F_dutyCycle();
	m->THROTTLE_percent	       = current_pedal_percent() * 100;
	m->THROTTLE_relayState	       = relay_state;
}
