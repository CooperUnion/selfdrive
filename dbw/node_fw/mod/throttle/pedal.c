#include "pedal.h"
#include "throttle.h"

#include <driver/dac.h>

#include "common.h"

// ######        DEFINES        ###### //

// ######     PRIVATE DATA      ###### //

/*
 * Helper struct to define the low and high voltage of each output.
 */
struct throttle_output {
    float32_t low_voltage;
    float32_t high_voltage;
};

static const struct throttle_output thr_A = {0.5f, 2.5f};
static const struct throttle_output thr_F = {1.5f, 4.5f};

// ######      PROTOTYPES       ###### //

static uint8_t voltage_to_pwm(float32_t v);
static uint8_t convert_throttle_command(struct throttle_output t, float32_t p);

// ######   PRIVATE FUNCTIONS   ###### //

/*
 * Convert voltage out of 3.3V maximum to a DAC/PWM command (0-255).
 */
static uint8_t voltage_to_pwm(float32_t v)
{
    return (v / 3.3f) * 255;
}

/*
 * Convert throttle percentage command to DAC command (0-255).
 */
static uint8_t convert_throttle_command(struct throttle_output t, float32_t p)
{
    if (p < 0.0f || p > 1.0f) {
        return voltage_to_pwm(t.low_voltage);
    }

    const float32_t voltage = t.low_voltage + (p * (t.high_voltage - t.low_voltage));
    return voltage_to_pwm(voltage);
}

// ######   PUBLIC FUNCTIONS    ###### //

/*
 * Enable the pedal output DACs.
 */
void enable_pedal_output()
{
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);
}

/*
 * Set the pedal outputs according to the given percentage command (0.00 - 1.00).
 */
void set_pedal_output(float32_t cmd)
{
    // clip command at 50%
    if (cmd > 0.50) {
        cmd = 0.50;
    } else if (cmd < 0.0) {
        cmd = 0.0;
    }

    const uint8_t thr_F_cmd = convert_throttle_command(thr_F, cmd);
    const uint8_t thr_A_cmd = convert_throttle_command(thr_A, cmd);

    dac_output_voltage(DAC_CHANNEL_1, thr_F_cmd);
    dac_output_voltage(DAC_CHANNEL_2, thr_A_cmd);

    CAN_Accel.ThrottleACmd = thr_A_cmd;
    CAN_Accel.ThrottleFCmd = thr_F_cmd;
    CAN_Accel.Percent = cmd * 100;
}
