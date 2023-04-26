#include "ctrl.h"

#include <esp_attr.h>
#include <driver/gpio.h>

#include "ember_common.h"
#include "cuber_base.h"
#include "pid.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_tx.h"
#include <string.h>

// ######        DEFINES        ###### //

#define ENCODER0_CHAN_A 26
#define ENCODER0_CHAN_B 27
#define ENCODER1_CHAN_A 17
#define ENCODER1_CHAN_B 0

#define ESP_INTR_FLAG_DEFAULT 0

#define ENCODER_MAX_TICKS 600  // slightly over 5MPH

#define ENCODER_TICKS_PER_ROTATION 4000
#define WHEEL_CIRCUMFERENCE_M      1.899156
#define TICKS_PER_M                (WHEEL_CIRCUMFERENCE_M / ENCODER_TICKS_PER_ROTATION)

#define ACCEL_TO_PEDAL_SLOPE_MAPPING        15.4
#define BRAKE_TO_PEDAL_SLOPE_MAPPING        -58.03
#define BRAKE_TO_PEDAL_SLOPE_MAPPING_OFFSET -11.33

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
static uint8_t linear_velocity;
static float vel_hist[4];
static uint8_t vel_index;
static float vel_filtered;

static pid_S *pid;

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

    vel_index = 0;
    memset(vel_hist, 0, sizeof(vel_hist));
}

static void ctrl_100Hz()
{
    static uint16_t prv_pulse_cnt[2];

    const uint16_t cur_pulse_cnt[2] = {pulse_cnt[0], pulse_cnt[1]};

    const int16_t left_delta  = cur_pulse_cnt[0] - prv_pulse_cnt[0];
    const int16_t right_delta = cur_pulse_cnt[1] - prv_pulse_cnt[1];

    prv_pulse_cnt[0] = cur_pulse_cnt[0];
    prv_pulse_cnt[1] = cur_pulse_cnt[1];

    // check if we're over the speed limit and trigger estop if so
    if (
        (ABS(left_delta)  >= ENCODER_MAX_TICKS) ||
        (ABS(right_delta) >= ENCODER_MAX_TICKS)
    ) {
        speed_alarm = true;
        base_set_state_estop(0 /* placeholder */);
    } else {
        speed_alarm = false;
    }

    if (CANRX_is_node_DBW_ok() && base_dbw_active()) {
        brake_percent    = CANRX_get_DBW_brakePercent();
        throttle_percent = CANRX_get_DBW_throttlePercent();
        linear_velocity = CANRX_get_DBW_linearVelocity();
    } else {
        brake_percent    = 0;
        throttle_percent = 0;
        linear_velocity = 0;
    }

    float kp = 0;
    float ki = 0;
    float kd = 0;
    float ts = 0;
    float upper_lim = 0;
    float lower_lim = 0;
    float sigma = 0;

    //vel_filtered = actua_vel;
    vel_hist[vel_index] = TICKS_PER_M * (pulse_cnt[0] + pulse_cnt[1]) / 0.01;
    vel_filtered = (vel_hist[0] + vel_hist[1] + vel_hist[2] + vel_hist[3]) / 4.0;

    // double check what's happening with pointers here
    pid_init(pid, kp, ki, kd, ts, upper_lim, lower_lim, sigma);
    // accel_des pid.step(target_vel, actual_vel)
    // target_vel = linear_velocity
    float actual_vel = vel_filtered; // Fred needs to make
    float accel_des = step(pid, linear_velocity, actual_vel);

    // call :pedal_ctrl (actual_vel, target_vel, accel_des)
    //vel_des = target_vel = linear_velocity
    //vel_act = actual_vel
    if (linear_velocity == 0.0) {
        throttle_percent = 0;
        brake_percent = 50;
    }
    else if (actual_vel < 0) {
        if (actual_vel > -0.5) {
            if ( (accel_des > 0) && (linear_velocity > 0)) {
                throttle_percent = ACCEL_TO_PEDAL_SLOPE_MAPPING * accel_des;
                brake_percent = 0;
            } else {
                throttle_percent = 0;
                brake_percent = (BRAKE_TO_PEDAL_SLOPE_MAPPING * accel_des) + BRAKE_TO_PEDAL_SLOPE_MAPPING_OFFSET;
            }
        }
        else {
            throttle_percent = 0;
            brake_percent = 50;
        }
    }
    else if (actual_vel >= 0) {
        if (accel_des > 0) {
            throttle_percent = ACCEL_TO_PEDAL_SLOPE_MAPPING * accel_des;
            brake_percent = 0;
        }
        else if (accel_des <= 0) {
            throttle_percent = 0;
            brake_percent = (BRAKE_TO_PEDAL_SLOPE_MAPPING * accel_des) + BRAKE_TO_PEDAL_SLOPE_MAPPING_OFFSET;
        }
    }
    else {
        throttle_percent = 0;
        brake_percent = 50;
    }


    vel_index = (vel_index + 1) % 4;
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
