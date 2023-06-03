#include "ctrl.h"

#include <esp_attr.h>
#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ember_common.h"
#include "cuber_base.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

#define ENCODER0_CHAN_A  4
#define ENCODER0_CHAN_B  3
#define ENCODER1_CHAN_A 36
#define ENCODER1_CHAN_B 35

#define ESP_INTR_FLAG_DEFAULT 0

#define ENCODER_MAX_TICKS           600  // slightly over 5MPH
#define ENCODER_TICKS_PER_ROTATION 4000

#define WHEEL_CIRCUMFERENCE_M 1.899156

#define METERS_PER_TICK (WHEEL_CIRCUMFERENCE_M / ENCODER_TICKS_PER_ROTATION)

#define AVERAGE_TICKS_SAMPLES 4

// ######      PROTOTYPES       ###### //

static void calculate_average_velocity(int16_t left_delta, int16_t right_delta);
static void encoder0_chan_a(void *arg);
static void encoder0_chan_b(void *arg);
static void encoder1_chan_a(void *arg);
static void encoder1_chan_b(void *arg);

// ######     PRIVATE DATA      ###### //

static volatile uint16_t pulse_cnt[2];
static bool speed_alarm;
static uint8_t brake_percent;
static uint8_t throttle_percent;

static float average_velocity;

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

    calculate_average_velocity(left_delta, right_delta);

    // check if we're over the speed limit and
    // go into the ESTOP state if that's the case
    if (
        (ABS(left_delta)  >= ENCODER_MAX_TICKS) ||
        (ABS(right_delta) >= ENCODER_MAX_TICKS))
    {
        brake_percent    = 0;
        throttle_percent = 0;

        speed_alarm = true;
        base_request_state(CUBER_SYS_STATE_ESTOP);

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
        taskDISABLE_INTERRUPTS();
        brake_percent    = CANRX_get_DBW_brakePercent();
        throttle_percent = CANRX_get_DBW_throttlePercent();
        taskENABLE_INTERRUPTS();

        base_request_state(CUBER_SYS_STATE_DBW_ACTIVE);
        return;
    }

    if (CANRX_is_message_DBW_VelocityCommand_ok()) {
        // TODO: set brake and throttle percentages
        // using a PID controller

        brake_percent    = 0;
        throttle_percent = 0;

        base_request_state(CUBER_SYS_STATE_DBW_ACTIVE);
        return;
    }

    brake_percent    = 0;
    throttle_percent = 0;
    base_request_state(CUBER_SYS_STATE_IDLE);
}

// ######   PRIVATE FUNCTIONS   ###### //

static void calculate_average_velocity(int16_t left_delta, int16_t right_delta)
{
    static size_t  index;
    static int32_t average_ticks_sum;
    static int32_t average_ticks_buf[AVERAGE_TICKS_SAMPLES];

    // remove stale value
    average_ticks_sum -= average_ticks_buf[index];

    average_ticks_buf[index] = (left_delta + right_delta) / 2;
    average_ticks_sum += average_ticks_buf[index];

    index = (index + 1) % AVERAGE_TICKS_SAMPLES;

    int16_t ticks = average_ticks_sum / AVERAGE_TICKS_SAMPLES;

    // magic scaling factor of 10 here, no idea why
    average_velocity = ticks * METERS_PER_TICK / 0.1;
}

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

void CANTX_populateTemplate_VelocityCommand(struct CAN_TMessage_DBWVelocityCommand * const m)
{
    m->brakePercent    = brake_percent;
    m->throttlePercent = throttle_percent;
}

void CANTX_populateTemplate_EncoderData(struct CAN_TMessage_EncoderData * const m)
{
    m->encoderLeft  = pulse_cnt[0];
    m->encoderRight = pulse_cnt[1];
}
