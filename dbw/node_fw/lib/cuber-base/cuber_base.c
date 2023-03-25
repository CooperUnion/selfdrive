#include "cuber_base.h"

#include <driver/gpio.h>
#include <rom/rtc.h>

#include "common.h"
#include "ember_taskglue.h"
#include "libgitrev.h"
#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

#define LED1_PIN 32
#define LED2_PIN 33

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

static RESET_REASON reset_reason;

// ######    RATE FUNCTIONS     ###### //

static void base_init();
static void base_10Hz();

ember_rate_funcs_S base_rf = {
    .call_init  = base_init,
    .call_10Hz  = base_10Hz,
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

    reset_reason = rtc_get_reset_reason(0);
}

static void base_10Hz()
{
    set_status_LEDs();
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

void base_set_state_idle(void)
{
    system_state = SYS_STATE_IDLE;
}

void base_set_state_dbw_active(void)
{
    system_state = SYS_STATE_DBW_ACTIVE;
}

void base_set_state_estop(void)
{
    system_state = SYS_STATE_ESTOP;
}

void base_set_wdt_trigger(void)
{
    wdt_trigger = true;
}

void ember_can_callback_notify_lost_can(void)
{
    system_state = SYS_STATE_LOST_CAN;
}

// ######         CAN RX         ###### //

void CANRX_onRxCallback_DBW_ESTOP(
    const struct CAN_MessageRaw_DBW_ESTOP * const raw,
    const struct CAN_Message_DBW_ESTOP * const dec)
{
    (void) raw;

    system_state = SYS_STATE_ESTOP;
}

// ######         CAN TX         ###### //

void CANTX_populateTemplate_NodeStatus(struct CAN_TMessage_DBWNodeStatus * const m)
{
    switch (system_state) {
        case SYS_STATE_IDLE:
            m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_IDLE;
            break;
        case SYS_STATE_DBW_ACTIVE:
            m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_ACTIVE;
            break;
        case SYS_STATE_ESTOP:
            m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
            break;
        default:
            m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_UNHEALTHY;
            break;
    }

    static typeof(m->counter) counter;
    m->counter = counter++;

    switch (reset_reason) {
        case POWERON_RESET:
            m->resetReason = CAN_T_DBWNODESTATUS_RESETREASON_POWERON;
            break;
        case RTCWDT_RTC_RESET:
            m->resetReason = CAN_T_DBWNODESTATUS_RESETREASON_WATCHDOG_RESET;
            break;
        default:
            m->resetReason = CAN_T_DBWNODESTATUS_RESETREASON_UNKNOWN;
            break;
    }

    m->esp32ResetReasonCode = reset_reason;
}

void CANTX_populateTemplate_NodeInfo(struct CAN_TMessage_DBWNodeInfo * const m)
{
    m->gitHash = GITREV_BUILD_REV;
    m->gitDirty = GITREV_BUILD_DIRTY;
    m->eepromIdentity = 0; // no eeprom identity at the moment
}
