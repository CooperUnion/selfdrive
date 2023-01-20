#include "cuber_base.h"

#include <driver/gpio.h>
#include <rom/rtc.h>

#include "common.h"
#include "cuber_nodetypes.h"
#include "ember_taskglue.h"
#include "libgitrev.h"
#include "opencan_rx.h"
#include "opencan_tx.h"

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

static RESET_REASON reset_reason;

// ######          CAN          ###### //

void CANTX_populate_TEST_NodeStatus(struct CAN_Message_TEST_NodeStatus * const m)
{
    switch (system_state) {
        case SYS_STATE_IDLE:
            m->TEST_sysStatus = CAN_TEST_SYSSTATUS_IDLE;
            break;
        case SYS_STATE_DBW_ACTIVE:
            m->TEST_sysStatus = CAN_TEST_SYSSTATUS_ACTIVE;
            break;
        case SYS_STATE_ESTOP:
            m->TEST_sysStatus = CAN_TEST_SYSSTATUS_ESTOP;
            break;
        default:
            m->TEST_sysStatus = CAN_TEST_SYSSTATUS_UNHEALTHY;
            break;
    }

    static uint8_t counter;
    m->TEST_counter = counter++;

    switch (reset_reason) {
        case POWERON_RESET:
            m->TEST_resetReason = CAN_TEST_RESETREASON_POWERON;
            break;
        case RTCWDT_RTC_RESET:
            m->TEST_resetReason = CAN_TEST_RESETREASON_WATCHDOG_RESET;
            break;
        default:
            m->TEST_resetReason = CAN_TEST_RESETREASON_UNKNOWN;
            break;
    }

    m->TEST_esp32ResetReasonCode = reset_reason;
}

void CANTX_populate_TEST_NodeInfo(struct CAN_Message_TEST_NodeInfo * const m)
{
    m->TEST_gitHash = GITREV_BUILD_REV;
    m->TEST_gitDirty = GITREV_BUILD_DIRTY;
    m->TEST_eepromIdentity = 0; // no eeprom identity at the moment
}

// ######    RATE FUNCTIONS     ###### //

static void base_init();
static void base_10Hz();
static void base_100Hz();

ember_rate_funcs_S base_rf = {
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

    reset_reason = rtc_get_reset_reason(0);
}

static void base_10Hz()
{
    set_status_LEDs();
}

static void base_100Hz()
{
    // we should be able to do this soon
    /* if (!CANRX_is_node_DBW_ok())
    {
        system_state = SYS_STATE_ESTOP;
    }
    else */ if (CANRX_get_DBW_active())
    {
        if (system_state == SYS_STATE_IDLE)
        {
            system_state = SYS_STATE_DBW_ACTIVE;
        }
        else
        {
            // keep current system state
        }
    }
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
    // we'll have oneshot message support soon

    system_state = SYS_STATE_ESTOP;

    // CAN_DBW_ESTOP.src = CAN_DBW_ESTOP_src_NODE_CHOICE;
    // CAN_DBW_ESTOP.reason = choice;

    // can_send_iface(&can_DBW_ESTOP_cfg, &CAN_DBW_ESTOP);
}


void base_set_wdt_trigger(void)
{
    wdt_trigger = true;
}
