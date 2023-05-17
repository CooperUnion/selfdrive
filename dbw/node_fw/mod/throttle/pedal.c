#include "pedal.h"
#include "throttle.h"

#include <driver/ledc.h>

#include "ember_common.h"

// ######        DEFINES        ###### //
// init -> enable_pedal_output()
// 100hz -> set_pedal_output(cmd)

// two seperate pins
// PWM 1 = GPIO 36
// PWM 2 = GPIO 35
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 1000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION 10
#define MAX_DUTY 1024

// ######     PRIVATE DATA      ###### //
static ledc_timer_config_t pwm1_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = 0,
    .freq_hz = PWM_FREQUENCY,
};

static ledc_channel_config_t pwm1_channel = {
    .gpio_num = 36,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = PWM_CHANNEL,
    .intr_type = 0,
    .timer_sel = 0,
    .duty = PWM_INIT_DUTY_CYCLE,
};

static ledc_timer_config_t pwm2_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = 0,
    .freq_hz = PWM_FREQUENCY,
};

static ledc_channel_config_t pwm2_channel = {
    .gpio_num = 35,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = PWM_CHANNEL,
    .intr_type = 0,
    .timer_sel = 0,
    .duty = PWM_INIT_DUTY_CYCLE,
};

/*
 * Helper struct to define the low and high voltage of each output.
 */
struct throttle_output {
    float32_t low_voltage;
    float32_t high_voltage;
};

static const struct throttle_output thr_A = {0.5f, 2.5f};
static const struct throttle_output thr_F = {1.5f, 4.5f};

static float32_t current_percent;

// ######      PROTOTYPES       ###### //
static void init_pwm(ledc_timer_config_t pwm_timer, ledc_channel_config_t pwm_channel);
static uint32_t voltage_to_pwm(float32_t v);
static uint32_t convert_throttle_command(struct throttle_output t, float32_t p);

// ######   PRIVATE FUNCTIONS   ###### //

static void init_pwm(ledc_timer_config_t pwm_timer, ledc_channel_config_t pwm_channel)
{
    ledc_timer_config(&pwm_timer);
    ledc_channel_config(&pwm_channel);
}

/*
 * Convert voltage out of 3.3V maximum to a PWM command (0-2^^PWM_RESOLUTION)
 */
static uint32_t voltage_to_pwm(float32_t v)
{
    return (v / 3.3f) * (2<<(PWM_RESOLUTION-1));
}

/*
 * Convert throttle percentage command to PWM command (0-2^^PWM_RESOLUTION)
 */
static uint32_t convert_throttle_command(struct throttle_output t, float32_t p)
{
    if (p < 0.0f || p > 1.0f) {
        return voltage_to_pwm(t.low_voltage);
    }

    const float32_t voltage = t.low_voltage + (p * (t.high_voltage - t.low_voltage));
    return voltage_to_pwm(voltage);
}

static uint32_t convert_throttle_cmd_to_duty(uint32_t cmd)
{
    return cmd * MAX_DUTY;
}

// ######   PUBLIC FUNCTIONS    ###### //

/*
 * Enable the pedal output DACs.
 */
void enable_pedal_output()
{
    init_pwm(pwm1_timer, pwm1_channel);
    init_pwm(pwm2_timer, pwm2_channel);
}

/*
 * Set the pedal outputs according to the given percentage command (0.00 - 1.00).
 */
void set_pedal_output(float32_t cmd)
{
    // clip command at 50%
    if (cmd > 0.50) {
        cmd = 0.50;
    }

    const uint32_t thr_F_cmd = convert_throttle_command(thr_F, cmd);
    const uint32_t thr_A_cmd = convert_throttle_command(thr_A, cmd);

    current_percent = cmd;

    const uint32_t thr_F_duty_cmd = convert_throttle_cmd_to_duty(thr_F_cmd);
    const uint32_t thr_A_duty_cmd = convert_throttle_cmd_to_duty(thr_A_cmd);

    // set pwm1
    ledc_set_duty(pwm1_timer.speed_mode, pwm1_channel.channel, thr_A_duty_cmd);
    ledc_update_duty(pwm1_timer.speed_mode, pwm1_channel.channel);

    // set pwm2
    ledc_set_duty(pwm2_timer.speed_mode, pwm2_channel.channel, thr_F_duty_cmd);
    ledc_update_duty(pwm2_timer.speed_mode, pwm2_channel.channel);
}

float32_t current_pedal_percent(void) {
    return current_percent;
}