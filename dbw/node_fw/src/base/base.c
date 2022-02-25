#include "base/base.h"

#include <driver/gpio.h>

#include "common.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

#define LED1_PIN 32
#define LED2_PIN 33

// ######      PROTOTYPES       ###### //

static void base_init();

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

// ######    RATE FUNCTIONS     ###### //

static void base_init();
static void base_10Hz();

struct rate_funcs base_rf = {
    .call_init = base_init,
    .call_10Hz = base_10Hz,
};

static void base_init() {
    system_state = SYS_STATE_GOOD;

    gpio_pad_select_gpio(LED1_PIN);
    gpio_pad_select_gpio(LED2_PIN);

    gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
}

static void base_10Hz() {
    // blink LEDs according to state
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
            if (!(timer % 5)) {
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

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

void base_set_state_lost_can() {
    system_state = SYS_STATE_LOST_CAN;
}

void base_set_state_good() {
    system_state = SYS_STATE_GOOD;
}

void base_set_wdt_trigger() {
    wdt_trigger = true;
}
