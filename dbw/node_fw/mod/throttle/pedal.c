#include "pedal.h"
#include "throttle.h"

#include <driver/gpio.h>
#include <driver/ledc.h>

#include "ember_common.h"

// ######        DEFINES        ###### //

#define CMD_MAX 0.5

#define GPIO_PWM_A GPIO_NUM_36
#define GPIO_PWM_F GPIO_NUM_35

#define PWM_FREQUENCY       36000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION      10

// ######     PRIVATE DATA      ###### //

static ledc_timer_config_t pwm_a_timer = {
    .speed_mode      = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num       = LEDC_TIMER_0,
    .freq_hz         = PWM_FREQUENCY,
};

static ledc_channel_config_t pwm_a_channel = {
    .gpio_num   = GPIO_PWM_A,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_0,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = PWM_INIT_DUTY_CYCLE,
};

static ledc_timer_config_t pwm_f_timer = {
    .speed_mode      = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num       = LEDC_TIMER_0,
    .freq_hz         = PWM_FREQUENCY,
};

static ledc_channel_config_t pwm_f_channel = {
    .gpio_num   = GPIO_PWM_F,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_1,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = PWM_INIT_DUTY_CYCLE,
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
static uint32_t voltage_to_duty_cycle(float32_t v);
static uint32_t convert_throttle_command(struct throttle_output t, float32_t p);

// ######   PRIVATE FUNCTIONS   ###### //

static void init_pwm(ledc_timer_config_t pwm_timer, ledc_channel_config_t pwm_channel)
{
    ledc_timer_config(&pwm_timer);
    ledc_channel_config(&pwm_channel);
}

/*
 * Convert voltage out of 3.3V maximum to a duty cycle (0--(2^PWM_RESOLUTION)-1).
 */
static uint32_t voltage_to_duty_cycle(float32_t v)
{
    return (v / 3.3f) * ((1 << PWM_RESOLUTION) - 1);
}

/*
 * Convert throttle percentage command to a voltage and then a duty cycle.
 */
static uint32_t convert_throttle_command(struct throttle_output t, float32_t p)
{
    if (p < 0.0f || p > 1.0f) {
        return voltage_to_duty_cycle(t.low_voltage);
    }

    const float32_t voltage = t.low_voltage + (p * (t.high_voltage - t.low_voltage));
    return voltage_to_duty_cycle(voltage);
}

// ######   PUBLIC FUNCTIONS    ###### //

/*
 * Enable the pedal output DACs.
 */
void enable_pedal_output()
{
    init_pwm(pwm_a_timer, pwm_a_channel);
    init_pwm(pwm_f_timer, pwm_f_channel);
}

/*
 * Set the pedal outputs according to the given percentage command (0.00 - 1.00).
 */
void set_pedal_output(float32_t cmd)
{
    if (cmd > CMD_MAX) cmd = CMD_MAX;

    const uint32_t thr_F_dutyCycle = convert_throttle_command(thr_F, cmd);
    const uint32_t thr_A_dutyCycle = convert_throttle_command(thr_A, cmd);

    current_percent = cmd;

    ledc_set_duty(pwm_a_timer.speed_mode, pwm_a_channel.channel, thr_A_dutyCycle);
    ledc_update_duty(pwm_a_timer.speed_mode, pwm_a_channel.channel);

    ledc_set_duty(pwm_f_timer.speed_mode, pwm_f_channel.channel, thr_F_dutyCycle);
    ledc_update_duty(pwm_f_timer.speed_mode, pwm_f_channel.channel);
}

float32_t current_pedal_percent(void) {
    return current_percent;
}
