#include "cuber_base.h"

#include <driver/gpio.h>
#include <rom/rtc.h>

#include "common.h"
#include "cuber_nodetypes.h"
#include "ember_can.h"
#include "ember_can_callbacks.h"
#include "ember_taskglue.h"
#include "libgitrev.h"

// ######        DEFINES        ###### //

#define LED1_PIN 32
#define LED2_PIN 33

#define DBW_ACTIVE_TIMEOUT_MS 200

// ######      PROTOTYPES       ###### //

static void set_status_LEDs();

// ######     PRIVATE DATA      ###### //

enum system_states {
    SYS_STATE_UNDEF = 0,
    SYS_STATE_INIT,
    SYS_STATE_IDLE,
    SYS_STATE_DBW_ACTIVE,
    SYS_STATE_LOST_CAN,
    SYS_STATE_BAD,
    SYS_STATE_ESTOP,
};

static enum system_states system_state = SYS_STATE_UNDEF;

static bool wdt_trigger;

// ######          CAN          ###### //

static struct CAN_NodeStatus_t CAN_Status;

static can_outgoing_t can_Status_cfg = {
    .id = CAN_NODESTATUS_FRAME_ID,
    .extd = CAN_NODESTATUS_IS_EXTENDED,
    .dlc = CAN_NODESTATUS_LENGTH,
    .pack = CAN_NodeStatus_pack,
};

static struct CAN_DBW_ESTOP_t CAN_DBW_ESTOP;

static can_outgoing_t can_DBW_ESTOP_cfg = {
    .id = CAN_DBW_ESTOP_FRAME_ID,
    .extd = CAN_DBW_ESTOP_IS_EXTENDED,
    .dlc = CAN_DBW_ESTOP_LENGTH,
    .pack = CAN_DBW_ESTOP_pack,
};

static struct CAN_DBW_Active_t CAN_DBW_Active;

static can_incoming_t can_DBW_Active_cfg = {
    .id = CAN_DBW_ACTIVE_FRAME_ID,
    .out = &CAN_DBW_Active,
    .unpack = CAN_DBW_Active_unpack,
};

static struct CAN_DBW_ESTOP_t CAN_DBW_ESTOP_in;

static can_incoming_t can_DBW_ESTOP_in_cfg = {
    .id = CAN_DBW_ESTOP_FRAME_ID,
    .out = &CAN_DBW_ESTOP_in,
    .unpack = CAN_DBW_ESTOP_unpack,
};

static struct CAN_NodeInfo_t CAN_Info;

static can_outgoing_t can_NodeInfo_cfg = {
    .id = CAN_NODEINFO_FRAME_ID,
    .extd = CAN_NODEINFO_IS_EXTENDED,
    .dlc = CAN_NODEINFO_LENGTH,
    .pack = CAN_NodeInfo_pack,
};

// ######    RATE FUNCTIONS     ###### //

static void base_init();
static void base_1Hz();
static void base_10Hz();
static void base_100Hz();

ember_rate_funcs_S base_rf = {
    .call_init  = base_init,
    .call_1Hz   = base_1Hz,
    .call_10Hz  = base_10Hz,
    .call_100Hz = base_100Hz,
};

static void base_init()
{
    system_state = SYS_STATE_IDLE;

    gpio_pad_select_gpio(LED1_PIN);
    gpio_pad_select_gpio(LED2_PIN);

    gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(LED1_PIN, 0);
    gpio_set_level(LED2_PIN, 0);

    // set up dbwNode_Info
    CAN_Info.gitHash     = GITREV_BUILD_REV;
    CAN_Info.gitDirty    = GITREV_BUILD_DIRTY;
    can_NodeInfo_cfg.id += CUBER_NODE_IDENTITY;

    // set up dbwNode_Status
    can_Status_cfg.id += CUBER_NODE_IDENTITY;

    const RESET_REASON reason = rtc_get_reset_reason(0);
    CAN_Status.esp32ResetReasonCode = reason;

    switch (reason) {
        case POWERON_RESET:
            CAN_Status.resetReason = CAN_NodeStatus_resetReason_POWERON_CHOICE;
            break;

        case RTCWDT_RTC_RESET:
            CAN_Status.resetReason = CAN_NodeStatus_resetReason_WATCHDOG_RESET_CHOICE;
            break;

        default:
            CAN_Status.resetReason = CAN_NodeStatus_resetReason_UNKNOWN_CHOICE;
            break;
    }

    can_register_incoming_msg(&can_DBW_Active_cfg);
    can_register_incoming_msg(&can_DBW_ESTOP_in_cfg);
}

static void base_1Hz()
{
    can_send_iface(&can_NodeInfo_cfg, &CAN_Info);
}

static void base_10Hz()
{
    set_status_LEDs();
}

static void base_100Hz()
{
    if (
        CAN_DBW_Active.active &&
        (can_DBW_Active_cfg.delta_ms < DBW_ACTIVE_TIMEOUT_MS) &&
        (system_state == SYS_STATE_IDLE)
    ) {
        system_state = SYS_STATE_DBW_ACTIVE;
    } else if (
        (!CAN_DBW_Active.active || (can_DBW_Active_cfg.delta_ms >= DBW_ACTIVE_TIMEOUT_MS)) &&
        (system_state == SYS_STATE_DBW_ACTIVE)
    ) {
        system_state = SYS_STATE_IDLE;
    }

    if (can_DBW_ESTOP_in_cfg.recieved) system_state = SYS_STATE_ESTOP;

    switch (system_state) {
        case SYS_STATE_IDLE:
            CAN_Status.sysStatus = CAN_NodeStatus_sysStatus_IDLE_CHOICE;
            break;

        case SYS_STATE_DBW_ACTIVE:
            CAN_Status.sysStatus = CAN_NodeStatus_sysStatus_ACTIVE_CHOICE;
            break;

        case SYS_STATE_ESTOP:
            CAN_Status.sysStatus = CAN_NodeStatus_sysStatus_ESTOP_CHOICE;
            break;

        default:
            CAN_Status.sysStatus = CAN_NodeStatus_sysStatus_UNHEALTHY_CHOICE;
            break;
    }

    can_send_iface(&can_Status_cfg, &CAN_Status);

    CAN_Status.counter++;
}

// ######   PRIVATE FUNCTIONS   ###### //

/*
 * Blink the status LEDs according to system state.
 * Timings are intended for 10Hz.
 */
static void set_status_LEDs() {
    static bool led1_state, led2_state;
    static uint timer;

    switch (system_state) {
        case SYS_STATE_UNDEF:
            if (!(timer % 2)) {
                led1_state = !led1_state;
                led2_state = !led2_state;
            }
            break;

        case SYS_STATE_IDLE:
            led1_state = 1;
            if (!(timer % 1)) {
                led2_state = !led2_state;
            }
            break;

        case SYS_STATE_DBW_ACTIVE:
            led1_state = 1;
            if (!(timer % 2)) {
                led2_state = !led2_state;
            }
            break;

        case SYS_STATE_LOST_CAN:
            led2_state = 1;
            if (!(timer % 2)) {
                led1_state = !led1_state;
            }
            break;

        case SYS_STATE_ESTOP:
            led2_state = 1;
            led1_state = !led1_state;
            break;

        default:
            led1_state = !led1_state;
            led2_state = !led2_state;
            break;
    }

    if (wdt_trigger) {
        led1_state = 1;
        led2_state = 1;
    }

    gpio_set_level(LED1_PIN, led1_state);
    gpio_set_level(LED2_PIN, led2_state);

    timer++;
}

// ######   PUBLIC FUNCTIONS    ###### //

bool base_dbw_active(void)
{
    return system_state == SYS_STATE_DBW_ACTIVE;
}


void ember_can_callback_notify_lost_can(void)
{
    system_state = SYS_STATE_LOST_CAN;
}


void base_set_state_estop(uint8_t choice)
{
    system_state = SYS_STATE_ESTOP;

    CAN_DBW_ESTOP.src = CAN_DBW_ESTOP_src_NODE_CHOICE;
    CAN_DBW_ESTOP.reason = choice;

    can_send_iface(&can_DBW_ESTOP_cfg, &CAN_DBW_ESTOP);
}


void base_set_wdt_trigger(void)
{
    wdt_trigger = true;
}
