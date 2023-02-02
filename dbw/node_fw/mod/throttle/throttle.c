#include "throttle.h"
#include "pedal.h"

#include <driver/gpio.h>

#include "common.h"
#include "cuber_base.h"
#include "cuber_nodetypes.h"
#include "ember_taskglue.h"

#include "opencan_rx.h"
#include "opencan_tx.h"

/* Define firmware module identity for the entire build. */
const enum cuber_node_types CUBER_NODE_IDENTITY = NODE_THROTTLE;

// ######        DEFINES        ###### //

#define MODE_CTRL_PIN 16
#define ENCODER_MAX_TICKS  85     // slightly over 5MPH

// ######     PRIVATE DATA      ###### //

static bool relay_state;

// ######      PROTOTYPES       ###### //

static void control_relay(bool cmd);

// ######    RATE FUNCTIONS     ###### //

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
    gpio_pad_select_gpio(MODE_CTRL_PIN);
    gpio_set_direction(MODE_CTRL_PIN, GPIO_MODE_OUTPUT);
    control_relay(0);

    enable_pedal_output();
}

static void throttle_100Hz()
{
    if (!CANRX_is_node_CTRL_ok()) {
        base_set_state_estop(0 /* placeholder */);
    }

    static uint16_t prv_left_ticks;
    static uint16_t prv_right_ticks;

    uint16_t left_ticks  = CANRX_get_CTRL_encoderLeft();
    uint16_t right_ticks = CANRX_get_CTRL_encoderRight();

    int16_t left_delta  = left_ticks - prv_left_ticks;
    int16_t right_delta = right_ticks - prv_right_ticks;

    if (ABS(left_delta)  > ENCODER_MAX_TICKS) base_set_state_estop(0);
    if (ABS(right_delta) > ENCODER_MAX_TICKS) base_set_state_estop(0);

    prv_left_ticks  = left_ticks;
    prv_right_ticks = right_ticks;

    /* set the relay based on whether DBW is active */
    control_relay(!base_estoped() && CANRX_is_node_DBW_ok());

    /* todo: set the cmd to 0 if DBW is not active, just in case the relay fails */
    float32_t cmd = ((float32_t) CANRX_get_DBW_throttlePercent()) / 100.0;

    set_pedal_output(cmd); // sets CAN feedback data too
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

// ######        CAN TX         ###### //

void CANTX_populate_THROTTLE_AccelData(struct CAN_Message_THROTTLE_AccelData * const m) {
    m->THROTTLE_throttleACmd = 0; // just leaving these off for now
    m->THROTTLE_throttleFCmd = 0; // just leaving these off for now
    m->THROTTLE_percent = current_pedal_percent();
    m->THROTTLE_relayState = relay_state;
}
