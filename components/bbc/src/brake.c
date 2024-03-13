#include "brake.h"

#include "firmware-base/state-machine.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <ember_common.h>
#include <ember_taskglue.h>
#include <opencan_rx.h>
#include <opencan_tx.h>

#define CMD_MAX 0.6

#define PWM_PIN GPIO_NUM_37

#define PWM_FREQUENCY	    1000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION	    10

#define CMD2DUTY(cmd) ((cmd) * ((1 << PWM_RESOLUTION) - 1))

static ledc_timer_config_t pwm_timer = {
    .speed_mode	     = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num	     = LEDC_TIMER_0,
    .freq_hz	     = PWM_FREQUENCY,
};

static ledc_channel_config_t pwm_channel = {
    .gpio_num	= PWM_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel	= LEDC_CHANNEL_0,
    .intr_type	= LEDC_INTR_DISABLE,
    .timer_sel	= LEDC_TIMER_0,
    .duty	= PWM_INIT_DUTY_CYCLE,
};

static void brake_init();
static void brake_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init	= brake_init,
    .call_100Hz = brake_100Hz,
};

static void brake_init()
{
	ledc_timer_config(&pwm_timer);
	ledc_channel_config(&pwm_channel);
}

static void brake_100Hz()
{
	static float32_t prv_cmd;

	bool brake_authorized = CANRX_is_message_SUP_Authorization_ok()
	    && CANRX_get_SUP_brakeAuthorized()
	    && CANRX_is_message_OM_VelocityCommand_ok();

	float cmd;

	if (brake_authorized) {
		cmd = ((float32_t) CANRX_get_OM_brakePercent()) / 100.0;
		base_request_state(SYS_STATE_DBW_ACTIVE);
	} else {
		cmd = 0.0;
		base_request_state(SYS_STATE_IDLE);
	}

	if (cmd > CMD_MAX) cmd = CMD_MAX;

	// low-pass filter
	const float32_t alpha = 0.1;
	cmd		      = prv_cmd + (alpha * (cmd - prv_cmd));

	prv_cmd = cmd;

	pwm_channel.duty = CMD2DUTY(cmd);
	ledc_channel_config(&pwm_channel);
}

void CANTX_populate_BBC_BrakeData(struct CAN_Message_BBC_BrakeData * const m)
{
	m->BBC_dutyCycle = pwm_channel.duty;

	m->BBC_percent = (float32_t) pwm_channel.duty
	    / (float32_t) (1 << PWM_RESOLUTION) * 100;
}
