#include "ctrl.h"

#include "firmware-base/state-machine.h"
#include <driver/gpio.h>
#include <ember_common.h>
#include <ember_taskglue.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <opencan_rx.h>
#include <opencan_tx.h>
#include <selfdrive/pid.h>

#define ENCODER0_CHAN_A 3
#define ENCODER0_CHAN_B 4
#define ENCODER1_CHAN_A 36
#define ENCODER1_CHAN_B 35

#define ESP_INTR_FLAG_DEFAULT 0

#define ENCODER_MAX_TICKS	   400	// slightly over 5mph
#define ENCODER_TICKS_PER_ROTATION 30536

#define WHEEL_CIRCUMFERENCE_M 1.7954

#define METERS_PER_TICK \
	((float) WHEEL_CIRCUMFERENCE_M / (float) ENCODER_TICKS_PER_ROTATION)

#define AVERAGE_TICKS_SAMPLES 4

#define KP    0.35
#define KI    0.035
#define KD    0.42
#define SIGMA 1.00

#define PID_LOWER_LIMIT	   -5
#define PID_UPPER_LIMIT	   2
#define PID_DEADBAND_LOWER 0
#define PID_DEADBAND_UPPER 0

static float acceleration_to_throttle_percent(float acceleration);
static void  ctrl_init();
static void  ctrl_100Hz();
static void  calculate_average_velocity(
	 int16_t left_delta, int16_t right_delta);
static void encoder0_chan_a(void *arg);
static void encoder0_chan_b(void *arg);
static void encoder1_chan_a(void *arg);
static void encoder1_chan_b(void *arg);
static void velocity_control(
	float desired_velocity, float desired_acceleration);

static volatile uint16_t pulse_cnt[2];
static bool		 speed_alarm;
static uint8_t		 brake_percent;
static uint8_t		 throttle_percent;

static float average_velocity;
static float desired_acceleration;

static selfdrive_pid_t pid;
static bool	       setpoint_reset;

ember_rate_funcs_S module_rf = {
	.call_init  = ctrl_init,
	.call_100Hz = ctrl_100Hz,
};

static void ctrl_init()
{
	gpio_set_direction(ENCODER0_CHAN_A, GPIO_MODE_INPUT);
	gpio_set_direction(ENCODER0_CHAN_B, GPIO_MODE_INPUT);
	gpio_set_direction(ENCODER1_CHAN_A, GPIO_MODE_INPUT);
	gpio_set_direction(ENCODER1_CHAN_B, GPIO_MODE_INPUT);

	gpio_set_intr_type(ENCODER0_CHAN_A, GPIO_INTR_ANYEDGE);
	gpio_set_intr_type(ENCODER0_CHAN_B, GPIO_INTR_ANYEDGE);
	gpio_set_intr_type(ENCODER1_CHAN_A, GPIO_INTR_ANYEDGE);
	gpio_set_intr_type(ENCODER1_CHAN_B, GPIO_INTR_ANYEDGE);

	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

	gpio_isr_handler_add(ENCODER0_CHAN_A, encoder0_chan_a, NULL);
	gpio_isr_handler_add(ENCODER0_CHAN_B, encoder0_chan_b, NULL);
	gpio_isr_handler_add(ENCODER1_CHAN_A, encoder1_chan_a, NULL);
	gpio_isr_handler_add(ENCODER1_CHAN_B, encoder1_chan_b, NULL);

	selfdrive_pid_init(&pid,
		KP,
		KI,
		KD,
		0.01,
		PID_LOWER_LIMIT,
		PID_UPPER_LIMIT,
		SIGMA);

	selfdrive_pid_set_deadbands(
		&pid, PID_DEADBAND_LOWER, PID_DEADBAND_UPPER);
}

static void ctrl_100Hz()
{
	static uint16_t prv_pulse_cnt[2];

	const uint16_t cur_pulse_cnt[2] = {pulse_cnt[0], pulse_cnt[1]};

	const int16_t left_delta  = cur_pulse_cnt[0] - prv_pulse_cnt[0];
	const int16_t right_delta = cur_pulse_cnt[1] - prv_pulse_cnt[1];

	prv_pulse_cnt[0] = cur_pulse_cnt[0];
	prv_pulse_cnt[1] = cur_pulse_cnt[1];

	calculate_average_velocity(left_delta, right_delta);

	// check if we're over the speed limit and
	// go into the ESTOP state if that's the case
	if ((base_get_state() == SYS_STATE_DBW_ACTIVE)
		&& ((ABS(left_delta) >= ENCODER_MAX_TICKS)
			|| (ABS(right_delta) >= ENCODER_MAX_TICKS))) {
		brake_percent	 = 0;
		throttle_percent = 0;

		speed_alarm = true;
		base_request_state(SYS_STATE_ESTOP);

		return;
	}

	speed_alarm = false;

	/*
	 * We want to set brake and throttle percentages based on either a
	 * raw or normal velocity command.  Ideally, a raw velocity command
	 * would not be sent when a regular velocity command is being sent
	 * and vice-versa.  To avoid this situation, we give the raw
	 * velocity command priority when setting percentages.
	 */
	if (CANRX_is_message_DBW_RawVelocityCommand_ok()) {
		base_request_state(SYS_STATE_DBW_ACTIVE);

		taskDISABLE_INTERRUPTS();
		brake_percent	 = CANRX_get_DBW_brakePercent();
		throttle_percent = CANRX_get_DBW_throttlePercent();
		taskENABLE_INTERRUPTS();

		return;
	}

	if (CANRX_is_message_DBW_VelocityCommand_ok()) {
		base_request_state(SYS_STATE_DBW_ACTIVE);

		float current_velocity = average_velocity;
		float desired_velocity = CANRX_get_DBW_linearVelocity();

		if (setpoint_reset) {
			selfdrive_pid_setpoint_reset(
				&pid, desired_velocity, current_velocity);
			setpoint_reset = false;
		}

		desired_acceleration = selfdrive_pid_step(
			&pid, desired_velocity, current_velocity);

		velocity_control(desired_velocity, desired_acceleration);

		return;
	}

	base_request_state(SYS_STATE_IDLE);

	brake_percent	 = 0;
	throttle_percent = 0;
	setpoint_reset	 = true;
}

static float acceleration_to_brake_percent(float acceleration)
{
	return -36.60 * acceleration;
}

static float acceleration_to_throttle_percent(float acceleration)
{
	return 40.97 * acceleration;
}

static void calculate_average_velocity(int16_t left_delta, int16_t right_delta)
{
	static size_t  index;
	static int32_t average_ticks_sum;
	static int32_t average_ticks_buf[AVERAGE_TICKS_SAMPLES];

	// remove stale value
	average_ticks_sum -= average_ticks_buf[index];

	average_ticks_buf[index]  = (left_delta + right_delta) / 2;
	average_ticks_sum	 += average_ticks_buf[index];

	index = (index + 1) % AVERAGE_TICKS_SAMPLES;

	int16_t ticks_10ms = average_ticks_sum / AVERAGE_TICKS_SAMPLES;


	// * 100 is to go from 10ms to 1s
	average_velocity = ticks_10ms * METERS_PER_TICK * 100;
}

static void IRAM_ATTR encoder0_chan_a(void *arg)
{
	(void) arg;

	const uint32_t chan_a = gpio_get_level(ENCODER0_CHAN_A);
	const uint32_t chan_b = gpio_get_level(ENCODER0_CHAN_B);

	if (chan_a) {
		if (chan_b) {
			--pulse_cnt[0];
		} else {
			++pulse_cnt[0];
		}
	} else {
		if (chan_b) {
			++pulse_cnt[0];
		} else {
			--pulse_cnt[0];
		}
	}
}

static void IRAM_ATTR encoder0_chan_b(void *arg)
{
	(void) arg;

	const uint32_t chan_a = gpio_get_level(ENCODER0_CHAN_A);
	const uint32_t chan_b = gpio_get_level(ENCODER0_CHAN_B);

	if (chan_b) {
		if (chan_a) {
			++pulse_cnt[0];
		} else {
			--pulse_cnt[0];
		}
	} else {
		if (chan_a) {
			--pulse_cnt[0];
		} else {
			++pulse_cnt[0];
		}
	}
}

static void IRAM_ATTR encoder1_chan_a(void *arg)
{
	(void) arg;

	const uint32_t chan_a = gpio_get_level(ENCODER1_CHAN_A);
	const uint32_t chan_b = gpio_get_level(ENCODER1_CHAN_B);

	if (chan_a) {
		if (chan_b) {
			--pulse_cnt[1];
		} else {
			++pulse_cnt[1];
		}
	} else {
		if (chan_b) {
			++pulse_cnt[1];
		} else {
			--pulse_cnt[1];
		}
	}
}

static void IRAM_ATTR encoder1_chan_b(void *arg)
{
	(void) arg;

	const uint32_t chan_a = gpio_get_level(ENCODER1_CHAN_A);
	const uint32_t chan_b = gpio_get_level(ENCODER1_CHAN_B);

	if (chan_b) {
		if (chan_a) {
			++pulse_cnt[1];
		} else {
			--pulse_cnt[1];
		}
	} else {
		if (chan_a) {
			--pulse_cnt[1];
		} else {
			++pulse_cnt[1];
		}
	}
}

static void velocity_control(
	float desired_velocity, float desired_acceleration)
{
	// we'd like to stop, or so i hope
	if (!desired_velocity) {
		brake_percent	 = 100;
		throttle_percent = 0;

		return;
	}

	if (desired_acceleration > 0) {
		brake_percent	 = 0;
		throttle_percent = acceleration_to_throttle_percent(
			desired_acceleration);
	} else {
		brake_percent
			= acceleration_to_brake_percent(desired_acceleration);
		throttle_percent = 0;
	}
}

void CANRX_onRxCallback_DBW_SetCTRLVelocityGains(
	const struct CAN_TMessageRaw_PIDGains * const raw,
	const struct CAN_TMessage_PIDGains * const    dec)
{
	(void) raw;

	pid.kp = dec->gainKp;
	pid.ki = dec->gainKi;
	pid.kd = dec->gainKd;
}

void CANTX_populate_CTRL_Alarms(struct CAN_Message_CTRL_Alarms * const m)
{
	m->CTRL_alarmsRaised = speed_alarm;
	m->CTRL_speedAlarm   = speed_alarm;
}

void CANTX_populate_CTRL_ControllerData(
	struct CAN_Message_CTRL_ControllerData * const m)
{
	m->CTRL_averageVelocity	    = average_velocity;
	m->CTRL_desiredAcceleration = desired_acceleration;
}

void CANTX_populateTemplate_ControllerGains(
	struct CAN_TMessage_PIDGains * const m)
{
	m->gainKp = pid.kp;
	m->gainKi = pid.ki;
	m->gainKd = pid.kd;
}

void CANTX_populateTemplate_VelocityCommand(
	struct CAN_TMessage_DBWVelocityCommand * const m)
{
	m->brakePercent	   = brake_percent;
	m->throttlePercent = throttle_percent;
}

void CANTX_populateTemplate_EncoderData(
	struct CAN_TMessage_EncoderData * const m)
{
	m->encoderLeft	= pulse_cnt[0];
	m->encoderRight = pulse_cnt[1];
}
