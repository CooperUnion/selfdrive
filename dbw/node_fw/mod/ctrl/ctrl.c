#include "ctrl.h"

#include <esp_attr.h>
#include <driver/gpio.h>

#include "ember_common.h"
#include "cuber_base.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

#define ENCODER0_CHAN_A 26
#define ENCODER0_CHAN_B 27
#define ENCODER1_CHAN_A 17
#define ENCODER1_CHAN_B 0

#define ESP_INTR_FLAG_DEFAULT 0

#define ENCODER_MAX_TICKS 600  // slightly over 5MPH

// ######      PROTOTYPES       ###### //

static void encoder0_chan_a(void *arg);
static void encoder0_chan_b(void *arg);
static void encoder1_chan_a(void *arg);
static void encoder1_chan_b(void *arg);

// ######     PRIVATE DATA      ###### //

static volatile uint16_t pulse_cnt[2];
static bool speed_alarm;
static uint8_t brake_percent;
static uint8_t throttle_percent;

// ######    RATE FUNCTIONS     ###### //

static void ctrl_init();
static void ctrl_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init  = ctrl_init,
    .call_100Hz = ctrl_100Hz,
};

static void ctrl_init()
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

static void ctrl_100Hz()
{
    static uint16_t prv_pulse_cnt[2];

    const uint16_t cur_pulse_cnt[2] = {pulse_cnt[0], pulse_cnt[1]};

    const int16_t left_delta  = cur_pulse_cnt[0] - prv_pulse_cnt[0];
    const int16_t right_delta = cur_pulse_cnt[1] - prv_pulse_cnt[1];

    prv_pulse_cnt[0] = cur_pulse_cnt[0];
    prv_pulse_cnt[1] = cur_pulse_cnt[1];

    // check if we're over the speed limit and
    // go into the ESTOP state if that's the case
    if (
        (ABS(left_delta)  >= ENCODER_MAX_TICKS) ||
        (ABS(right_delta) >= ENCODER_MAX_TICKS))
    {
        brake_percent    = 0;
        throttle_percent = 0;

        speed_alarm = true;
        base_set_state_estop();

        return;
    }

    speed_alarm = false;

    /*
     * We want to set brake and throttle percentages based on either a
     * raw or normal velocity command.  Ideally, a raw velocity command
     * would not be sent when a regular velocity command is being sent
     * and vice-versa.  To avoid this situation, we give the raw
     * velocity command priority when setting percentages.
     */

    if (CANRX_is_message_DBW_RawVelocityCommand_ok()) {
        brake_percent    = CANRX_get_DBW_brakePercent();
        throttle_percent = CANRX_get_DBW_throttlePercent();
        base_set_state_dbw_active();

        return;
    }

    if (CANRX_is_message_DBW_VelocityCommand_ok()) {
        // TODO: set brake and throttle percentages
        // using a PID controller

        brake_percent    = 0;
        throttle_percent = 0;
        base_set_state_dbw_active();

        return;
    }

    brake_percent    = 0;
    throttle_percent = 0;
    base_set_state_idle();
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

void CANTX_populate_CTRL_Alarms(struct CAN_Message_CTRL_Alarms * const m)
{
    m->CTRL_alarmsRaised = speed_alarm;
    m->CTRL_speedAlarm   = speed_alarm;
}

void CANTX_populate_CTRL_VelocityCommand(struct CAN_Message_CTRL_VelocityCommand * const m)
{
    m->CTRL_brakePercent    = brake_percent;
    m->CTRL_throttlePercent = throttle_percent;
}

void CANTX_populateTemplate_EncoderData(struct CAN_TMessage_EncoderData * const m)
{
    m->encoderLeft  = pulse_cnt[0];
    m->encoderRight = pulse_cnt[1];
}
