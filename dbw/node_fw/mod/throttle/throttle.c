#include "throttle.h"
#include "pedal.h"

#include <driver/gpio.h>

#include "base/base.h"
#include "common.h"
#include "io/can.h"
#include "module_types.h"
#include "sys/task_glue.h"

/* Define firmware module identity for the entire build. */
const enum firmware_module_types FIRMWARE_MODULE_IDENTITY = MOD_THROTTLE;

// ######        DEFINES        ###### //

#define MODE_CTRL_PIN 16

#define ENCODER_TIMEOUT_US 20000

// ~5MPH
#define ENCODER_MAX_TICKS 85

// ######     PRIVATE DATA      ###### //

static bool relay_state;

static uint32_t can_Encoder_Data_delta_ms;

// ######          CAN          ###### //

struct CAN_dbwNode_Accel_Data_t CAN_Accel; // used by pedal.c; not static

static const can_outgoing_t can_Accel_Data_cfg = {
    .id = CAN_DBWNODE_ACCEL_DATA_FRAME_ID,
    .extd = CAN_DBWNODE_ACCEL_DATA_IS_EXTENDED,
    .dlc = CAN_DBWNODE_ACCEL_DATA_LENGTH,
    .pack = CAN_dbwNode_Accel_Data_pack,
};

static struct CAN_dbwNode_Accel_Cmd_t CAN_Accel_Cmd;

static const can_incoming_t can_Accel_Cmd_cfg = {
    .id = CAN_DBWNODE_ACCEL_CMD_FRAME_ID,
    .out = &CAN_Accel_Cmd,
    .unpack = CAN_dbwNode_Accel_Cmd_unpack,
};

static struct CAN_dbwNode_Encoder_Data_t CAN_Encoder_Data;

static const can_incoming_t can_Encoder_Data_cfg = {
    .id = CAN_DBWNODE_ENCODER_DATA_FRAME_ID,
    .out = &CAN_Encoder_Data,
    .unpack = CAN_dbwNode_Encoder_Data_unpack,
    .delta_ms = &can_Encoder_Data_delta_ms,
};

// ######      PROTOTYPES       ###### //

static void control_relay(bool cmd);

// ######    RATE FUNCTIONS     ###### //

static void throttle_init();
static void throttle_10Hz();

const struct rate_funcs module_rf = {
    .call_init = throttle_init,
    .call_10Hz = throttle_10Hz,
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
    gpio_pad_select_gpio(MODE_CTRL_PIN);
    gpio_set_direction(MODE_CTRL_PIN, GPIO_MODE_OUTPUT);
    control_relay(0);

    enable_pedal_output();

    can_register_incoming_msg(can_Accel_Cmd_cfg);
    can_register_incoming_msg(can_Encoder_Data_cfg);
}

static void throttle_10Hz()
{
    // make sure we didn't E-STOP
    if (
        (ABS(CAN_Encoder_Data.Encoder0) >= ENCODER_MAX_TICKS) ||
        (ABS(CAN_Encoder_Data.Encoder1) >= ENCODER_MAX_TICKS) ||
        (can_Encoder_Data_delta_ms >= ENCODER_TIMEOUT_US)
    ) base_set_state_estop();

    // set the relay from the CAN data
    control_relay(
        CAN_Accel_Cmd.ModeCtrl &&
        base_dbw_currently_active()
    );
    CAN_Accel.RelayState = relay_state;

    // get the pedal output from the CAN data
    float32_t cmd = CAN_dbwNode_Accel_Cmd_ThrottleCmd_decode(CAN_Accel_Cmd.ThrottleCmd);
    set_pedal_output(cmd); // sets CAN feedback data too

    // send CAN feedback message
    can_send_iface(&can_Accel_Data_cfg, &CAN_Accel);
}

// ######   PRIVATE FUNCTIONS   ###### //

/*
 * Open or close the relay.
 */
static void control_relay(bool cmd)
{
    gpio_set_level(MODE_CTRL_PIN, cmd);
    relay_state = cmd;
}

// ######   PUBLIC FUNCTIONS    ###### //
