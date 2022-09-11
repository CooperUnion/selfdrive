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

#define CMD_TIMEOUT_MS 200

// ######     PRIVATE DATA      ###### //

static bool relay_state;

// ######          CAN          ###### //

struct CAN_dbwNode_Accel_Data_t CAN_Accel; // used by pedal.c; not static
static struct CAN_dbwNode_Vel_Cmd_t CAN_Vel_Cmd;

// ######      PROTOTYPES       ###### //

static void control_relay(bool cmd);

// ######    RATE FUNCTIONS     ###### //

static void throttle_init();
static void throttle_100Hz();

const struct rate_funcs module_rf = {
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
    gpio_pad_select_gpio(MODE_CTRL_PIN);
    gpio_set_direction(MODE_CTRL_PIN, GPIO_MODE_OUTPUT);
    control_relay(0);

    enable_pedal_output();

    can_register_incoming_msg(&CAN_dbwNode_Vel_Cmd_info_S, &CAN_Vel_Cmd);
}

static void throttle_100Hz()
{
    if (base_dbw_active() && !can_Vel_Cmd_cfg.recieved) {
        static uint64_t prv_delta_ms;
        static bool     set;

        if (set) {
            if (can_Vel_Cmd_cfg.delta_ms - prv_delta_ms >= CMD_TIMEOUT_MS)
                base_set_state_estop(CAN_dbwESTOP_Reason_TIMEOUT_CHOICE);
        } else {
            prv_delta_ms = can_Vel_Cmd_cfg.delta_ms;
            set = true;
        }
    }

    if (
        base_dbw_active() &&
        can_Vel_Cmd_cfg.recieved &&
        (can_Vel_Cmd_cfg.delta_ms >= CMD_TIMEOUT_MS)
    )
        base_set_state_estop(CAN_dbwESTOP_Reason_TIMEOUT_CHOICE);

    // set the relay from the CAN data
    control_relay(base_dbw_active());
    CAN_Accel.RelayState = relay_state;

    // get the pedal output from the CAN data
    float32_t cmd = ((float32_t) CAN_Vel_Cmd.ThrottlePercent) / 100.0;
    set_pedal_output(cmd); // sets CAN feedback data too

    // send CAN feedback message
    can_send_iface(&CAN_dbwNode_Accel_Data_info_S, &CAN_Accel);
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
