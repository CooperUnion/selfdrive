#include "steer.h"

#include <driver/gpio.h>
#include <esp_attr.h>
#include <math.h>
#include <string.h>

#include "cuber_base.h"
#include "ember_common.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_tx.h"
#include "pid.h"

// ######        DEFINES        ###### //

#define KP 1.750
#define TS 0.001

#define ENCODER2DEG_SLOPE        0.0029
#define ENCODER2DEG_SLOPE_OFFSET 0.0446

#define RAD2DEG(ang) (ang * (180 / M_PI))

// ######      PROTOTYPES       ###### //

static float encoder2deg(void);
static bool odrive_calibration_needed(void);

// ######     PRIVATE DATA      ###### //

static struct {
    bool odrive_calibration : 1;
} alarm;

enum {
    IDLE,
    FULL_CALIBRATION_SEQUENCE,
    CLOSED_LOOP_CONTROL,
} odrive_state = IDLE;

pid_S pid;

enum {
    READY,
    CALIBRATING,
    NEEDS_CALIBRATION,
} steer_state = READY;

float velocity;

// ######    RATE FUNCTIONS     ###### //

static void steer_init();
static void steer_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init  = steer_init,
    .call_100Hz = steer_100Hz,
};

static void steer_init()
{
    pid_init(&pid, KP, 0, 0, TS, 0, 0, 0);
}

static void steer_100Hz()
{
    bool calibration_needed = odrive_calibration_needed();

    alarm.odrive_calibration = calibration_needed;

    if (calibration_needed && steer_state != CALIBRATING)
        steer_state = NEEDS_CALIBRATION;

    bool steer_authorized =
        CANRX_is_message_DBW_SteeringCommand_ok() &&
        CANRX_is_message_SUP_Authorization_ok() &&
        CANRX_is_message_WHL_AbsoluteEncoder_ok() &&
        CANRX_is_node_ODRIVE_ok() &&
        CANRX_get_SUP_steerAuthorized();

    if (!steer_authorized) {
        base_request_state(CUBER_SYS_STATE_IDLE);

        velocity = 0;

        odrive_state = IDLE;
        CANTX_doTx_STEER_ODriveRequestState();

        // reset our PID controller
        // for the next time we use it
        pid_setpoint_reset(&pid, 0, 0);

        return;
    }

    base_request_state(CUBER_SYS_STATE_DBW_ACTIVE);

    // we only want to calibrate when DBW is active
    if (steer_state == NEEDS_CALIBRATION) {
        odrive_state = FULL_CALIBRATION_SEQUENCE;
        CANTX_doTx_STEER_ODriveRequestState();

        steer_state = CALIBRATING;

        return;
    }

    if (steer_state == CALIBRATING) {
        // TODO: ideally add a timeout to also reboot the ODrive
        if (CANRX_get_ODRIVE_axisState() != CAN_ODRIVE_AXISSTATE_IDLE)
            return;

        steer_state = READY;
    }

    if (steer_state != READY) {
        odrive_state = IDLE;
        CANTX_doTx_STEER_ODriveRequestState();

        return;
    }

    odrive_state = CLOSED_LOOP_CONTROL;
    CANTX_doTx_STEER_ODriveRequestState();

    velocity = pid_step(
        &pid,
        RAD2DEG(CANRX_get_DBW_steeringAngle()),
        encoder2deg());
}

// ######   PRIVATE FUNCTIONS   ###### //

static float encoder2deg(void)
{
    uint16_t raw_ticks = CANRX_get_WHL_encoder();

    // unfortunately, OpenCAN doesn't support
    // big-endian messages at the moment...
    uint16_t ticks = ((raw_ticks & 0xff00) << 8) | (raw_ticks & 0xff);

    float deg = (ENCODER2DEG_SLOPE * ticks) + ENCODER2DEG_SLOPE_OFFSET;

    return deg;
}

static bool odrive_calibration_needed(void)
{
    bool calibration_needed = false;

    calibration_needed |= CANRX_get_ODRIVE_axisError() != CAN_ODRIVE_AXISERROR_NONE;
    calibration_needed |= CANRX_get_ODRIVE_motorErrorAlarm();
    calibration_needed |= CANRX_get_ODRIVE_encoderErrorAlarm();
    calibration_needed |= CANRX_get_ODRIVE_controllerErrorAlarm();

    return calibration_needed;
}

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //

void CANTX_populate_STEER_Alarms(struct CAN_Message_STEER_Alarms * const m)
{
    m->STEER_alarmsRaised           = alarm.odrive_calibration;
    m->STEER_odriveCalibrationAlarm = alarm.odrive_calibration;
}

void CANTX_populate_STEER_ODriveClearErrors(uint8_t * const data, uint8_t * const len)
{
    (void) data;
    (void) len;
}

void CANTX_populate_STEER_ODriveControllerMode(struct CAN_Message_STEER_ODriveControllerMode * const m)
{
    m->STEER_odriveControlMode = CAN_STEER_ODRIVECONTROLMODE_VELOCITY_CONTROL;
    m->STEER_odriveInputMode   = CAN_STEER_ODRIVEINPUTMODE_PASSTHROUGH;
}

void CANTX_populate_STEER_ODriveRequestState(struct CAN_Message_STEER_ODriveRequestState * const m)
{
    switch (odrive_state) {
        case IDLE:
            m->STEER_odriveRequestState = CAN_STEER_ODRIVEREQUESTSTATE_IDLE;
            break;

        case FULL_CALIBRATION_SEQUENCE:
            m->STEER_odriveRequestState = CAN_STEER_ODRIVEREQUESTSTATE_FULL_CALIBRATION_SEQUENCE;
            break;

        case CLOSED_LOOP_CONTROL:
            m->STEER_odriveRequestState = CAN_STEER_ODRIVEREQUESTSTATE_CLOSED_LOOP_CONTROL;
            break;

        default:
            m->STEER_odriveRequestState = CAN_STEER_ODRIVEREQUESTSTATE_UNDEFINED;
            break;
    }
}

void CANTX_populate_STEER_ODriveVelocity(struct CAN_Message_STEER_ODriveVelocity * const m)
{
    // unfortunately, OpenCAN doesn't support
    // IEEE754 signals at the moment...
    memcpy(&m->STEER_odriveVelocity, &velocity, sizeof(velocity));

    m->STEER_odriveTorqueFeedForward = 0;
}

void CANTX_populate_STEER_SteeringData(struct CAN_Message_STEER_SteeringData * const m)
{
    m->STEER_angle             = 0;
    m->STEER_encoderTimeoutSet = 0;
    m->STEER_oDriveConnected   = 0;

    switch (steer_state) {
        case READY:
            m->STEER_state = CAN_STEER_STATE_READY;
            break;

        case CALIBRATING:
            m->STEER_state = CAN_STEER_STATE_CALIBRATING;
            break;

        case NEEDS_CALIBRATION:
            m->STEER_state = CAN_STEER_STATE_NEEDS_CALIBRATION;
            break;
    }
}
