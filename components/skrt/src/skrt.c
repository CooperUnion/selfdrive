#include "skrt.h"

#include "firmware-base/state-machine.h"
#include <driver/gpio.h>
#include <ember_common.h>
#include <ember_taskglue.h>
#include <esp_attr.h>
#include <opencan_rx.h>
#include <opencan_tx.h>
#include <selfdrive/pid.h>

#include <math.h>
#include <string.h>

#define KP 1.750
#define TS 0.001

#define PID_UPPER_LIMIT 10
#define PID_LOWER_LIMIT -10

#define ENCODER2DEG_SLOPE	 0.0029
#define ENCODER2DEG_SLOPE_OFFSET 0.0446

#define RAD2DEG(ang) (ang * (180.0 / M_PI))

static void  steer_init();
static void  steer_100Hz();
static float encoder2deg(void);
static bool  odrive_calibration_needed(void);

static struct {
	bool odrive_calibration : 1;
} alarm;

enum {
	IDLE,
	FULL_CALIBRATION_SEQUENCE,
	CLOSED_LOOP_CONTROL,
} odrive_state = IDLE;

selfdrive_pid_t pid;

enum {
	READY,
	CALIBRATING,
	NEEDS_CALIBRATION,
} steer_state = READY;

float velocity;

ember_rate_funcs_S module_rf = {
    .call_init	= steer_init,
    .call_100Hz = steer_100Hz,
};

static void steer_init()
{
	selfdrive_pid_init(&pid,
	    KP,
	    0,
	    0,
	    TS,
	    PID_LOWER_LIMIT,
	    PID_UPPER_LIMIT,
	    0);
}

static void steer_100Hz()
{
	bool calibration_needed = odrive_calibration_needed();

	alarm.odrive_calibration = calibration_needed;

	bool steer_authorized = CANRX_is_message_DBW_SteeringCommand_ok()
	    && CANRX_is_message_SOUP_Authorization_ok()
	    && CANRX_is_message_WHL_AbsoluteEncoder_ok()
	    && CANRX_is_node_ODRIVE_ok() && CANRX_get_SOUP_skrtAuthorized();

	if (!steer_authorized) {
		base_request_state(SYS_STATE_IDLE);

		velocity = 0;

		if (odrive_state != IDLE) {
			odrive_state = IDLE;
			CANTX_doTx_SKRT_ODriveRequestState();
		}

		return;
	}

	base_request_state(SYS_STATE_DBW_ACTIVE);

	// clear errors before calibrating
	if (calibration_needed) {
		CANTX_doTx_SKRT_ODriveClearErrors();
		steer_state = NEEDS_CALIBRATION;

		return;
	}

	// we only want to calibrate when DBW is active
	if (steer_state == NEEDS_CALIBRATION) {
		odrive_state = FULL_CALIBRATION_SEQUENCE;
		CANTX_doTx_SKRT_ODriveRequestState();

		steer_state = CALIBRATING;

		return;
	}

	if (steer_state == CALIBRATING) {
		// TODO: ideally add a timeout to also reboot the ODrive
		if (CANRX_get_ODRIVE_axisState() != CAN_ODRIVE_AXISSTATE_IDLE)
			return;

		steer_state = READY;
	}

	float encoder_deg = encoder2deg();

	if (odrive_state != CLOSED_LOOP_CONTROL) {
		odrive_state = CLOSED_LOOP_CONTROL;
		CANTX_doTx_SKRT_ODriveRequestState();

		selfdrive_pid_setpoint_reset(&pid,
		    CANRX_get_DBW_steeringAngle(),
		    encoder_deg);
	}

	velocity = selfdrive_pid_step(&pid,
	    RAD2DEG(CANRX_get_DBW_steeringAngle()),
	    encoder_deg);
}

static float encoder2deg(void)
{
	uint16_t raw_ticks = CANRX_get_WHL_encoder();

	// unfortunately, OpenCAN doesn't support
	// big-endian messages at the moment...
	int16_t ticks
	    = ((raw_ticks & 0xff00) >> 8) | ((raw_ticks & 0xff) << 8);

	float deg = (ENCODER2DEG_SLOPE * ticks) + ENCODER2DEG_SLOPE_OFFSET;

	return deg;
}

static bool odrive_calibration_needed(void)
{
	bool calibration_needed = false;

	calibration_needed
	    |= CANRX_get_ODRIVE_axisError() != CAN_ODRIVE_AXISERROR_NONE;
	calibration_needed |= CANRX_get_ODRIVE_motorErrorAlarm();
	calibration_needed |= CANRX_get_ODRIVE_encoderErrorAlarm();
	calibration_needed |= CANRX_get_ODRIVE_controllerErrorAlarm();

	return calibration_needed;
}

void CANTX_populate_SKRT_Alarms(struct CAN_Message_SKRT_Alarms * const m)
{
	m->SKRT_alarmsRaised	       = alarm.odrive_calibration;
	m->SKRT_odriveCalibrationAlarm = alarm.odrive_calibration;
}

void CANTX_populate_SKRT_ODriveClearErrors(uint8_t * const data,
    uint8_t * const					   len)
{
	(void) data;
	(void) len;
}

void CANTX_populate_SKRT_ODriveRequestState(
    struct CAN_Message_SKRT_ODriveRequestState * const m)
{
	switch (odrive_state) {
		case IDLE:
			m->SKRT_odriveRequestState
			    = CAN_SKRT_ODRIVEREQUESTSTATE_IDLE;
			break;

		case FULL_CALIBRATION_SEQUENCE:
			m->SKRT_odriveRequestState
			    = CAN_SKRT_ODRIVEREQUESTSTATE_FULL_CALIBRATION_SEQUENCE;
			break;

		case CLOSED_LOOP_CONTROL:
			m->SKRT_odriveRequestState
			    = CAN_SKRT_ODRIVEREQUESTSTATE_CLOSED_LOOP_CONTROL;
			break;

		default:
			m->SKRT_odriveRequestState
			    = CAN_SKRT_ODRIVEREQUESTSTATE_UNDEFINED;
			break;
	}
}

void CANTX_populate_SKRT_ODriveVelocity(
    struct CAN_Message_SKRT_ODriveVelocity * const m)
{
	// unfortunately, OpenCAN doesn't support
	// IEEE754 signals at the moment...
	memcpy(&m->SKRT_odriveVelocity, &velocity, sizeof(velocity));

	m->SKRT_odriveTorqueFeedForward = 0;
}

void CANTX_populate_SKRT_SteeringData(
    struct CAN_Message_SKRT_SteeringData * const m)
{
	switch (steer_state) {
		case READY:
			m->SKRT_state = CAN_SKRT_STATE_READY;
			break;

		case CALIBRATING:
			m->SKRT_state = CAN_SKRT_STATE_CALIBRATING;
			break;

		case NEEDS_CALIBRATION:
			m->SKRT_state = CAN_SKRT_STATE_NEEDS_CALIBRATION;
			break;
	}
}
