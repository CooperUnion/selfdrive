#include "base/base.h"

#include <driver/gpio.h>
#include <rom/rtc.h>

#include "common.h"
#include "module_types.h"
#include "io/can.h"
#include "libgitrev.h"
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

static struct CAN_dbwNode_Status_t CAN_Status;

static can_outgoing_t can_Status_cfg = {
    .id = CAN_DBWNODE_STATUS_FRAME_ID,
    .extd = CAN_DBWNODE_STATUS_IS_EXTENDED,
    .dlc = CAN_DBWNODE_STATUS_LENGTH,
    .pack = CAN_dbwNode_Status_pack,
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

static struct CAN_dbwNode_Info_t CAN_Info;

static can_outgoing_t can_dbwNode_Info_cfg = {
    .id = CAN_DBWNODE_INFO_FRAME_ID,
    .extd = CAN_DBWNODE_INFO_IS_EXTENDED,
    .dlc = CAN_DBWNODE_INFO_LENGTH,
    .pack = CAN_dbwNode_Info_pack,
};

// ######    RATE FUNCTIONS     ###### //

static void base_init();
static void base_1Hz();
static void base_10Hz();
static void base_100Hz();

const struct rate_funcs base_rf = {
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
    CAN_Info.Githash = GITREV_BUILD_REV;
    CAN_Info.GitDirty = GITREV_BUILD_DIRTY;
    can_dbwNode_Info_cfg.id += FIRMWARE_MODULE_IDENTITY;

    // set up dbwNode_Status
    can_Status_cfg.id += FIRMWARE_MODULE_IDENTITY;

    const RESET_REASON reason = rtc_get_reset_reason(0);
    CAN_Status.Esp32ResetReasonCode = reason;

    switch (reason) {
        case POWERON_RESET:
            CAN_Status.ResetReason = CAN_dbwNode_Status_ResetReason_POWERON_CHOICE;
            break;

        case RTCWDT_RTC_RESET:
            CAN_Status.ResetReason = CAN_dbwNode_Status_ResetReason_WATCHDOG_RESET_CHOICE;
            break;

        default:
            CAN_Status.ResetReason = CAN_dbwNode_Status_ResetReason_UNKNOWN_CHOICE;
            break;
    }

    can_register_incoming_msg(&can_DBW_Active_cfg);
    can_register_incoming_msg(&can_DBW_ESTOP_in_cfg);
}

static void base_1Hz()
{
    can_send_iface(&can_dbwNode_Info_cfg, &CAN_Info);
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
            CAN_Status.SystemStatus = CAN_dbwNode_Status_SystemStatus_IDLE_CHOICE;
            break;

        case SYS_STATE_DBW_ACTIVE:
            CAN_Status.SystemStatus = CAN_dbwNode_Status_SystemStatus_ACTIVE_CHOICE;
            break;

        case SYS_STATE_ESTOP:
            CAN_Status.SystemStatus = CAN_dbwNode_Status_SystemStatus_ESTOP_CHOICE;
            break;

        default:
            CAN_Status.SystemStatus = CAN_dbwNode_Status_SystemStatus_UNHEALTHY_CHOICE;
            break;
    }

    can_send_iface(&can_Status_cfg, &CAN_Status);

    CAN_Status.Counter++;
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
