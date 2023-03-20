#include "brake.h"

#include <driver/ledc.h>

#include "ember_common.h"
#include "cuber_base.h"
#include "cuber_nodetypes.h"
#include "ember_taskglue.h"

#include "opencan_rx.h"
#include "opencan_tx.h"

/* Define firmware module identity for the entire build. */
const enum cuber_node_types CUBER_NODE_IDENTITY = NODE_BRAKE;

// ######        DEFINES        ###### //

#define MAX_BRAKE_POWER 0.60

#define PWM_PIN 16
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 1000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION 16
#define MAX_DUTY 65535

// ######      PROTOTYPES       ###### //

static void init_pwm(ledc_timer_config_t pwm_timer, ledc_channel_config_t pwm_channel);
static float32_t clip_brake_cmd(ledc_channel_config_t pwm_channel, float32_t cmd);
static uint32_t convert_brake_cmd_to_duty(float32_t cmd);

// ######     PRIVATE DATA      ###### //

static ledc_timer_config_t pwm_timer = {
    .speed_mode = 0,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = 0,
    .freq_hz = PWM_FREQUENCY,
};

static ledc_channel_config_t pwm_channel = {
    .gpio_num = PWM_PIN,
    .speed_mode = 0,
    .channel = PWM_CHANNEL,
    .intr_type = 0,
    .timer_sel = 0,
    .duty = PWM_INIT_DUTY_CYCLE,
};

// ######    RATE FUNCTIONS     ###### //

static void brake_init();
static void brake_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init = brake_init,
    .call_100Hz = brake_100Hz,
};

/*
 * Initialize pwm struct and attach output to GPIO pin
 *
 * Begin receiving incoming commands to the brake node over CAN
 */
static void brake_init()
{
    init_pwm(pwm_timer, pwm_channel);
}

/*
 * Get the target brake percent from the CAN data
 *
 * Set the brake output to the target percentage
 *
 * Update and send CAN message
 */
static void brake_100Hz()
{
    static float32_t prev_cmd;

    bool dbw_active = base_dbw_active();

    // check now even though base also checks
    if (dbw_active && !CANRX_is_node_DBW_ok()) {
        base_set_state_estop(0 /* dummy value, API will change */);
    }

    float32_t cmd = (dbw_active)
        ? ((float32_t) CANRX_get_DBW_brakePercent()) / 100.0
        : 0.0;

    cmd = clip_brake_cmd(pwm_channel, cmd);

    // low-pass filter
    const float32_t alpha = 0.1;
    cmd = prev_cmd + (alpha * (cmd - prev_cmd));

    prev_cmd = cmd;

    // output pwm
    const uint32_t duty_cmd = convert_brake_cmd_to_duty(cmd);

    pwm_channel.duty = duty_cmd;
    ledc_channel_config(&pwm_channel);
}

// ######   PRIVATE FUNCTIONS   ###### //

/*
 * Sets up the pwm pin to output from specified GPIO pin
 */
static void init_pwm(ledc_timer_config_t pwm_timer, ledc_channel_config_t pwm_channel)
{
    ledc_timer_config(&pwm_timer);
    ledc_channel_config(&pwm_channel);
}

/*
 * Clip input brake percentage
 */
static float32_t clip_brake_cmd(ledc_channel_config_t pwm_channel, float32_t cmd)
{
    // clip command at 60% (May be too high since hydrastar was disconnected from the rear tires)
    if (cmd > MAX_BRAKE_POWER) {
        cmd = MAX_BRAKE_POWER;
    } else if (cmd < 0) {
        cmd = 0.0;
    }

    return cmd;
}

static uint32_t convert_brake_cmd_to_duty(float32_t cmd)
{
    return cmd * MAX_DUTY;
}

// ######   PUBLIC FUNCTIONS    ###### //

// ######         CAN TX         ###### //

void CANTX_populate_BRAKE_BrakeData(struct CAN_Message_BRAKE_BrakeData * const m)
{
    m->BRAKE_frequency = pwm_timer.freq_hz;
    m->BRAKE_resolution = pwm_timer.duty_resolution;
    m->BRAKE_percent = 100 * ((float32_t) pwm_channel.duty / (float32_t) MAX_DUTY);
    m->BRAKE_dutyCycle = pwm_channel.duty;
}
