#include "fnr.h"

#include <driver/gpio.h>

#include "ember_common.h"
#include "cuber_base.h"
#include "ember_taskglue.h"

#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

#define MODE_CTRL_PIN GPIO_NUM_5
// 1-A, 6-B, 3-C, 4-D
#define DBW_CTRL1_PIN GPIO_NUM_6
#define DBW_CTRL3_PIN GPIO_NUM_7
#define DBW_CTRL4_PIN GPIO_NUM_8
#define DBW_CTRL6_PIN GPIO_NUM_9

// ######     PRIVATE DATA      ###### //

static enum relay_states state = MANUAL;

// ######      PROTOTYPES       ###### //

static void control_relay(bool auth);
static void set_drive_state(enum drive_states state);

// ######    RATE FUNCTIONS     ###### //

static void fnr_init();
static void fnr_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init  = fnr_init,
    .call_100Hz = fnr_100Hz,
};

static void fnr_init()
{
    gpio_config(&(gpio_config_t){
        .pin_bit_mask
            = BIT64(MODE_CTRL_PIN)
            | BIT64(DBW_CTRL1_PIN)
            | BIT64(DBW_CTRL3_PIN)
            | BIT64(DBW_CTRL4_PIN)
            | BIT64(DBW_CTRL6_PIN),
        .mode = GPIO_MODE_OUTPUT,
    });

    set_drive_state(NEUTRAL);
    control_relay(0);
}

static void fnr_100Hz()
{
    bool fnr_authorized =
        CANRX_get_SUP_fnrAuthorized() &&
            (CANRX_is_message_DBW_VelocityCommand_ok() ||
                CANRX_is_message_DBW_RawVelocityCommand_ok());

    if (!fnr_authorized) {
        state = MANUAL;
        control_relay(fnr_authorized);
        base_request_state(CUBER_SYS_STATE_IDLE);

        return;
    }

    state = DBW;
    base_request_state(CUBER_SYS_STATE_DBW_ACTIVE);
    // prioritizing raw velocity message over normal one
    /*
    if (CANRX_is_message_DBW_RawVelocityCommand_ok())
    {
        switch (CANRX_get_DBW_fnr()) {
            case CAN_T_DBWVELOCITYCOMMAND_FNR_NEUTRAL:
                set_drive_state(NEUTRAL);
                break;

            case CAN_T_DBWVELOCITYCOMMAND_FNR_FORWARD:
               set_drive_state(FORWARD);
                break;

            case CAN_T_DBWVELOCITYCOMMAND_FNR_REVERSE:
                set_drive_state(REVERSE);
                break;
        }

        return;
    }
    */
    // make this more compact
    if (CANRX_get_DBW_linearVelocity() > 0)
    {
        set_drive_state(FORWARD);
        return;
    }
    if (CANRX_get_DBW_linearVelocity() < 0)
    {
        set_drive_state(REVERSE);
        return;
    }
    set_drive_state(NEUTRAL);
}
// ######   PRIVATE FUNCTIONS   ###### //

// Open or close the relay.
static void control_relay(bool auth)
{
    gpio_set_level(MODE_CTRL_PIN, auth);
}

// Truth Table for FNR
//  A   B   C   D   Position
//  0   1   1   1   Forward
//  0   0   0   0   Neutral
//  1   0   1   0   Reverse

static void set_drive_state(enum drive_states state) {
    switch (state) {
        case FORWARD:
            gpio_set_level(DBW_CTRL1_PIN, 0); // A
            gpio_set_level(DBW_CTRL3_PIN, 1); // C
            gpio_set_level(DBW_CTRL4_PIN, 1); // D
            gpio_set_level(DBW_CTRL6_PIN, 1); // B
            break;

        case NEUTRAL:
            gpio_set_level(DBW_CTRL1_PIN, 0); // A
            gpio_set_level(DBW_CTRL3_PIN, 0); // C
            gpio_set_level(DBW_CTRL4_PIN, 0); // D
            gpio_set_level(DBW_CTRL6_PIN, 0); // B
            break;

        case REVERSE:
            gpio_set_level(DBW_CTRL1_PIN, 1); // A
            gpio_set_level(DBW_CTRL3_PIN, 1); // C
            gpio_set_level(DBW_CTRL4_PIN, 0); // D
            gpio_set_level(DBW_CTRL6_PIN, 0); // B
            break;
    }
}

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //

void CANTX_populate_FNR_FNRData(struct CAN_Message_FNR_FNRData * const m) {
    switch (state) {
        case MANUAL:
            m->FNR_relayState = CAN_FNR_RELAYSTATE_MANUAL;
            break;

        case DBW:
            m->FNR_relayState = CAN_FNR_RELAYSTATE_DBW;
            break;
    }
}
