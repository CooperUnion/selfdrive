#include "base/base.h"

#include <driver/gpio.h>

#include "common.h"
#include "module_types.h"
#include "io/can.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

#define LED1_PIN 32
#define LED2_PIN 33

// ######      PROTOTYPES       ###### //

static void set_status_LEDs();

// ######     PRIVATE DATA      ###### //

enum system_states {
    SYS_STATE_UNDEF = 0,
    SYS_STATE_INIT,
    SYS_STATE_GOOD,
    SYS_STATE_LOST_CAN,
    SYS_STATE_BAD,
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

static struct CAN_dbwNode_Info_t CAN_Info;

// ######    RATE FUNCTIONS     ###### //

static void base_init();
static void base_10Hz();
static void base_100Hz();

const struct rate_funcs base_rf = {
    .call_init = base_init,
    .call_10Hz = base_10Hz,
    .call_1Hz = base_100Hz,
};

static void base_init()
{
    system_state = SYS_STATE_GOOD;

    gpio_pad_select_gpio(LED1_PIN);
    gpio_pad_select_gpio(LED2_PIN);

    gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(LED1_PIN, 0);
    gpio_set_level(LED2_PIN, 0);

    can_Status_cfg.id += FIRMWARE_MODULE_IDENTITY;
}

static void base_10Hz()
{
    set_status_LEDs();

    CAN_Status.Counter++;
}

static void base_100Hz()
{
    switch (system_state) {
        case SYS_STATE_GOOD:
            CAN_Status.SystemStatus = CAN_dbwNode_Status_SystemStatus_HEALTHY_CHOICE;
            break;

        default:
            CAN_Status.SystemStatus = CAN_dbwNode_Status_SystemStatus_UNHEALTHY_CHOICE;
            break;
    }

    can_send_iface(&can_Status_cfg, &CAN_Status);
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

        case SYS_STATE_GOOD:
            led1_state = 1;
            if (!(timer % 1)) {
                led2_state = !led2_state;
            }
            break;

        case SYS_STATE_LOST_CAN:
            led2_state = 1;
            if (!(timer % 2)) {
                led1_state = !led1_state;
            }
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

void base_set_state_lost_can()
{
    system_state = SYS_STATE_LOST_CAN;
}


void base_set_state_good()
{
    system_state = SYS_STATE_GOOD;
}


void base_set_wdt_trigger()
{
    wdt_trigger = true;
}
