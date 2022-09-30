#include "base/base.h"

#include <driver/gpio.h>
#include <rom/rtc.h>

#include "common.h"
#include "module_types.h"
#include "io/can.h"
#include "sys/task_glue.h"

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

static struct CAN_DBW_NodeStatus_t CAN_Status;

static can_outgoing_t can_Status_cfg = {
    .id = CAN_DBW_NODESTATUS_FRAME_ID,
    .extd = CAN_DBW_NODESTATUS_IS_EXTENDED,
    .dlc = CAN_DBW_NODESTATUS_LENGTH,
    .pack = CAN_DBW_NodeStatus_pack,
};

static struct CAN_dbwESTOP_t CAN_DBW_ESTOP;

static can_outgoing_t can_DBW_ESTOP_cfg = {
    .id = CAN_DBWESTOP_FRAME_ID,
    .extd = CAN_DBWESTOP_IS_EXTENDED,
    .dlc = CAN_DBWESTOP_LENGTH,
    .pack = CAN_dbwESTOP_pack,
};

static struct CAN_dbwActive_t CAN_DBW_Active;

static can_incoming_t can_DBW_Active_cfg = {
    .id = CAN_DBWACTIVE_FRAME_ID,
    .out = &CAN_DBW_Active,
    .unpack = CAN_dbwActive_unpack,
};

static struct CAN_dbwESTOP_t CAN_DBW_ESTOP_in;

static can_incoming_t can_DBW_ESTOP_in_cfg = {
    .id = CAN_DBWESTOP_FRAME_ID,
    .out = &CAN_DBW_ESTOP_in,
    .unpack = CAN_dbwESTOP_unpack,
};

static struct CAN_DBW_NodeInfo_t CAN_Info;

// ######    RATE FUNCTIONS     ###### //

static void base_init();
static void base_10Hz();
static void base_100Hz();

const struct rate_funcs base_rf = {
    .call_init  = base_init,
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

    can_Status_cfg.id += FIRMWARE_MODULE_IDENTITY;

    const RESET_REASON reason = rtc_get_reset_reason(0);
    CAN_Status.esp32ResetReasonCode = reason;

    switch (reason) {
        case POWERON_RESET:
            CAN_Status.resetReason = CAN_DBW_NodeStatus_resetReason_POWERON_CHOICE;
            break;

        case RTCWDT_RTC_RESET:
            CAN_Status.resetReason = CAN_DBW_NodeStatus_resetReason_WATCHDOG_RESET_CHOICE;
            break;

        default:
            CAN_Status.resetReason = CAN_DBW_NodeStatus_resetReason_UNKNOWN_CHOICE;
            break;
    }

    can_register_incoming_msg(&can_DBW_Active_cfg);
    can_register_incoming_msg(&can_DBW_ESTOP_in_cfg);
}

static void base_10Hz()
{
    set_status_LEDs();
}

static void base_100Hz()
{
    if (
        CAN_DBW_Active.Active &&
        (can_DBW_Active_cfg.delta_ms < DBW_ACTIVE_TIMEOUT_MS) &&
        (system_state == SYS_STATE_IDLE)
    ) {
        system_state = SYS_STATE_DBW_ACTIVE;
    } else if (
        (!CAN_DBW_Active.Active || (can_DBW_Active_cfg.delta_ms >= DBW_ACTIVE_TIMEOUT_MS)) &&
        (system_state == SYS_STATE_DBW_ACTIVE)
    ) {
        system_state = SYS_STATE_IDLE;
    }

    if (can_DBW_ESTOP_in_cfg.recieved) system_state = SYS_STATE_ESTOP;

    switch (system_state) {
        case SYS_STATE_IDLE:
            CAN_Status.systemStatus = CAN_DBW_NodeStatus_systemStatus_IDLE_CHOICE;
            break;

        case SYS_STATE_DBW_ACTIVE:
            CAN_Status.systemStatus = CAN_DBW_NodeStatus_systemStatus_ACTIVE_CHOICE;
            break;

        case SYS_STATE_ESTOP:
            CAN_Status.systemStatus = CAN_DBW_NodeStatus_systemStatus_ESTOP_CHOICE;
            break;

        default:
            CAN_Status.systemStatus = CAN_DBW_NodeStatus_systemStatus_UNHEALTHY_CHOICE;
            break;
    }

    can_send_iface(&can_Status_cfg, &CAN_Status);

    CAN_Status.counter++;
}

// ######   PRIVATE FUNCTIONS   ###### //

/*
 * Blink the status LEDs according to system state.
 *
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

bool base_dbw_active()
{
    return system_state == SYS_STATE_DBW_ACTIVE;
}


void base_set_state_lost_can()
{
    system_state = SYS_STATE_LOST_CAN;
}


void base_set_state_estop(uint8_t choice)
{
    system_state = SYS_STATE_ESTOP;

    CAN_DBW_ESTOP.Source = CAN_dbwESTOP_Source_NODE_CHOICE;
    CAN_DBW_ESTOP.Reason = choice;

    can_send_iface(&can_DBW_ESTOP_cfg, &CAN_DBW_ESTOP);
}


void base_set_wdt_trigger()
{
    wdt_trigger = true;
}
