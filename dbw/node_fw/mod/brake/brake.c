#include "brake.h"

#include <driver/ledc.h>

#include "base/base.h"
#include "common.h"
#include "io/can.h"
#include "module_types.h"
#include "sys/task_glue.h"

/* Define firmware module identity for the entire build. */
const enum firmware_module_types FIRMWARE_MODULE_IDENTITY = MOD_BRAKE;

// ######        DEFINES        ###### //

#define MAX_BRAKE_POWER 0.60

#define PWM_PIN 16
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 1000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION 16
#define MAX_DUTY 65535

#define CMD_TIMEOUT_MS 200

// ######      PROTOTYPES       ###### //

static void init_pwm(ledc_timer_config_t pwm_timer, ledc_channel_config_t pwm_channel);
static void send_brake_feedback(ledc_timer_config_t pwm_timer, ledc_channel_config_t pwm_channel);
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

// ######          CAN          ###### //

static struct CAN_dbwNode_Brake_Data_t CAN_Brake;

static const can_outgoing_t can_Brake_Data_cfg = {
    .id = CAN_DBWNODE_BRAKE_DATA_FRAME_ID,
    .extd = CAN_DBWNODE_BRAKE_DATA_IS_EXTENDED,
    .dlc = CAN_DBWNODE_BRAKE_DATA_LENGTH,
    .pack = CAN_dbwNode_Brake_Data_pack,
};

static struct CAN_dbwNode_Vel_Cmd_t CAN_Vel_Cmd;

static can_incoming_t can_Vel_Cmd_cfg = {
    .id = CAN_DBWNODE_VEL_CMD_FRAME_ID,
    .out = &CAN_Vel_Cmd,
    .unpack = CAN_dbwNode_Vel_Cmd_unpack,
};

// ######    RATE FUNCTIONS     ###### //

static void brake_init();
static void brake_100Hz();

struct rate_funcs module_rf = {
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

    can_register_incoming_msg(&can_Vel_Cmd_cfg);
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

    /*
    if (base_dbw_active() && !can_Vel_Cmd_cfg.recieved) {
        static uint64_t prv_delta_ms;
        static bool     set;

        if (set) {
            if (can_Vel_Cmd_cfg.delta_ms - prv_delta_ms >= CMD_TIMEOUT_MS)
                base_set_state_estop(CAN_dbwESTOP_Reason_TIMEOUT_CHOICE);
        } else {
            prv_delta_ms = can_Vel_Cmd_cfg.delta_ms;
            set = true;
        }
    }

    if (
        base_dbw_active() &&
        can_Vel_Cmd_cfg.recieved &&
        (can_Vel_Cmd_cfg.delta_ms >= CMD_TIMEOUT_MS)
    )
        base_set_state_estop(CAN_dbwESTOP_Reason_TIMEOUT_CHOICE);
    */

    float32_t cmd = (base_dbw_active())
        ? ((float32_t) CAN_Vel_Cmd.BrakePercent) / 100.0
        : 0.0;

    cmd = clip_brake_cmd(pwm_channel, cmd);

    const float32_t alpha = 0.1;
    cmd = prev_cmd + (alpha * (cmd - prev_cmd));

    prev_cmd = cmd;

    // output pwm
    const uint32_t duty_cmd = convert_brake_cmd_to_duty(cmd);

    pwm_channel.duty = duty_cmd;
    ledc_channel_config(&pwm_channel);

    send_brake_feedback(pwm_timer, pwm_channel);
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
 * Soft start so there are no voltage spikes
 *
 * Update and send CAN message as the brake percentage  increments
 */
static void send_brake_feedback(ledc_timer_config_t pwm_timer, ledc_channel_config_t pwm_channel)
{
    // update and send message
    CAN_Brake.Frequency = pwm_timer.freq_hz;
    CAN_Brake.Resolution = pwm_timer.duty_resolution;
    CAN_Brake.Percent = 100 * ((float32_t) pwm_channel.duty / (float32_t) MAX_DUTY);
    CAN_Brake.DutyCycle = pwm_channel.duty;

    can_send_iface(&can_Brake_Data_cfg, &CAN_Brake);
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
