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

void CANTX_populate_STEER_ODriveControllerMode(struct CAN_Message_STEER_ODriveControllerMode * const m)
{
    m->STEER_odriveControlMode = CAN_STEER_ODRIVECONTROLMODE_VELOCITY_CONTROL;
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
}
