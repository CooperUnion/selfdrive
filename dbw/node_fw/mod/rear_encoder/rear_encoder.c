#include "rear_encoder.h"

#include <esp_attr.h>
#include <driver/gpio.h>

#include "ember_common.h"
#include "cuber_base.h"
#include "cuber_nodetypes.h"
#include "ember_taskglue.h"
#include "opencan_tx.h"

/* Define firmware module identity for the entire build. */
const enum cuber_node_types CUBER_NODE_IDENTITY = NODE_REAR_ENCODER;

// ######        DEFINES        ###### //

#define ENCODER0_CHAN_A 27
#define ENCODER0_CHAN_B 26
#define ENCODER1_CHAN_A 17
#define ENCODER1_CHAN_B 0

#define ESP_INTR_FLAG_DEFAULT 0

#define ENCODER_MAX_TICKS  85     // slightly over 5MPH
#define ENCODER_TIMEOUT_US 20000

// ######      PROTOTYPES       ###### //

static void encoder0_chan_a(void *arg);
static void encoder0_chan_b(void *arg);
static void encoder1_chan_a(void *arg);
static void encoder1_chan_b(void *arg);

// ######     PRIVATE DATA      ###### //

static volatile int64_t pulse_cnt[2];

// ######    RATE FUNCTIONS     ###### //

static void encoder_init();
static void encoder_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init  = encoder_init,
    .call_100Hz = encoder_100Hz,
};

static void encoder_init()
{
    gpio_set_direction(ENCODER0_CHAN_A, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER0_CHAN_B, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER1_CHAN_A, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER1_CHAN_B, GPIO_MODE_INPUT);

    gpio_set_intr_type(ENCODER0_CHAN_A, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER0_CHAN_B, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER1_CHAN_A, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER1_CHAN_B, GPIO_INTR_ANYEDGE);

    gpio_pullup_en(ENCODER0_CHAN_A);
    gpio_pullup_en(ENCODER0_CHAN_B);
    gpio_pullup_en(ENCODER1_CHAN_A);
    gpio_pullup_en(ENCODER1_CHAN_B);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    gpio_isr_handler_add(ENCODER0_CHAN_A, encoder0_chan_a, NULL);
    gpio_isr_handler_add(ENCODER0_CHAN_B, encoder0_chan_b, NULL);
    gpio_isr_handler_add(ENCODER1_CHAN_A, encoder1_chan_a, NULL);
    gpio_isr_handler_add(ENCODER1_CHAN_B, encoder1_chan_b, NULL);
}

static void encoder_100Hz()
{
    // check if we're over speed and trigger estop if so
    const int32_t left_ticks = pulse_cnt[0];
    const int32_t right_ticks = pulse_cnt[1];

    if ((ABS(left_ticks) >= ENCODER_MAX_TICKS) ||
        (ABS(right_ticks) >= ENCODER_MAX_TICKS)) {
        base_set_state_estop(0 /* placeholder */);
    }
}

// ######   PRIVATE FUNCTIONS   ###### //

static void IRAM_ATTR encoder0_chan_a(void *arg)
{
    const uint32_t chan_a = gpio_get_level(ENCODER0_CHAN_A);
    const uint32_t chan_b = gpio_get_level(ENCODER0_CHAN_B);

    if (chan_a) {
        if (chan_b) {
            --pulse_cnt[0];
        } else {
            ++pulse_cnt[0];
        }
    } else {
        if (chan_b) {
            ++pulse_cnt[0];
        } else {
            --pulse_cnt[0];
        }
    }
}

static void IRAM_ATTR encoder0_chan_b(void *arg)
{
    const uint32_t chan_a = gpio_get_level(ENCODER0_CHAN_A);
    const uint32_t chan_b = gpio_get_level(ENCODER0_CHAN_B);

    if (chan_b) {
        if (chan_a) {
            ++pulse_cnt[0];
        } else {
            --pulse_cnt[0];
        }
    } else {
        if (chan_a) {
            --pulse_cnt[0];
        } else {
            ++pulse_cnt[0];
        }
    }
}

static void IRAM_ATTR encoder1_chan_a(void *arg)
{
    const uint32_t chan_a = gpio_get_level(ENCODER1_CHAN_A);
    const uint32_t chan_b = gpio_get_level(ENCODER1_CHAN_B);

    if (chan_a) {
        if (chan_b) {
            --pulse_cnt[1];
        } else {
            ++pulse_cnt[1];
        }
    } else {
        if (chan_b) {
            ++pulse_cnt[1];
        } else {
            --pulse_cnt[1];
        }
    }
}

static void IRAM_ATTR encoder1_chan_b(void *arg)
{
    const uint32_t chan_a = gpio_get_level(ENCODER1_CHAN_A);
    const uint32_t chan_b = gpio_get_level(ENCODER1_CHAN_B);

    if (chan_b) {
        if (chan_a) {
            ++pulse_cnt[1];
        } else {
            --pulse_cnt[1];
        }
    } else {
        if (chan_a) {
            --pulse_cnt[1];
        } else {
            ++pulse_cnt[1];
        }
    }
}

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //

void CANTX_populateTemplate_EncoderData(struct CAN_TMessage_EncoderData * const m) {
    m->encoderLeft = pulse_cnt[0];
    m->encoderRight = pulse_cnt[1];
}
