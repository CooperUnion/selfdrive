#include "steer.h"

#include <driver/gpio.h>
#include <esp_attr.h>

#include "cuber_base.h"
#include "ember_common.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

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

enum {
    READY,
    CALIBRATING,
    NEEDS_CALIBRATION,
} steer_state = READY;

// ######    RATE FUNCTIONS     ###### //

static void steer_init();
static void steer_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init  = steer_init,
    .call_100Hz = steer_100Hz,
};

static void steer_init()
{
}

static void steer_100Hz()
{
    bool calibration_needed = odrive_calibration_needed();

    alarm.odrive_calibration = calibration_needed;

    if (!calibration_needed)
        steer_state = READY;
    else if (steer_state != CALIBRATING)
        steer_state = NEEDS_CALIBRATION;

    if (steer_state == NEEDS_CALIBRATION) {
        odrive_state = FULL_CALIBRATION_SEQUENCE;
        steer_state  = CALIBRATING;

        CANTX_doTx_STEER_ODriveClearErrors();
        CANTX_doTx_STEER_ODriveRequestState();
    }
}

// ######   PRIVATE FUNCTIONS   ###### //

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
    m->STEER_odriveVelocity          = 0;
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
