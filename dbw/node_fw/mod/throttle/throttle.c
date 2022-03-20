#include "throttle.h"

#include <driver/dac.h>
#include <driver/gpio.h>

#include "common.h"
#include "io/can.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

#define MODE_CTRL_PIN 16

// ######     PRIVATE DATA      ###### //

struct throttle_output {
    float32_t low_voltage;
    float32_t high_voltage;
};

static const struct throttle_output thr_A = {0.5f, 2.5f};
static const struct throttle_output thr_F = {1.5f, 4.5f};

// ######      PROTOTYPES       ###### //

static void throttle_init();
static void throttle_1Hz();

static uint8_t voltage_to_pwm(float32_t v);
static uint8_t convert_throttle_command(struct throttle_output t, float32_t p);

// ######    RATE FUNCTIONS     ###### //

struct rate_funcs module_rf = {
    .call_init = throttle_init,
    .call_1Hz = throttle_1Hz,
};

static void throttle_init()
{
    gpio_pad_select_gpio(MODE_CTRL_PIN);
    gpio_set_direction(MODE_CTRL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODE_CTRL_PIN, 0);

    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);
}

static void throttle_1Hz()
{
    static bool state;
    static uint timer;
    timer++;

    if (!(timer % 5)) {
        gpio_set_level(MODE_CTRL_PIN, state);
        state = !state;
    }

    const float32_t cmd = 0.0f;
    const uint8_t thr_F_cmd = convert_throttle_command(thr_F, cmd);
    const uint8_t thr_A_cmd = convert_throttle_command(thr_A, cmd);

    dac_output_voltage(DAC_CHANNEL_1, thr_F_cmd);
    dac_output_voltage(DAC_CHANNEL_2, thr_A_cmd);

    const twai_message_t msg = {
        .extd = 0,
        .identifier = 0x03,
        .data_length_code = 2,
        .data = {thr_F_cmd, thr_A_cmd},
    };

    can_send_msg(&msg);
}

// ######   PRIVATE FUNCTIONS   ###### //

static uint8_t voltage_to_pwm(float32_t v) {
    return (v / 3.3f) * 255;
}

static uint8_t convert_throttle_command(struct throttle_output t, float32_t p) {
    if (p < 0.0f || p > 1.0f) {
        return voltage_to_pwm(t.low_voltage);
    }

    const float32_t voltage = t.low_voltage + (p * (t.high_voltage - t.low_voltage));
    return voltage_to_pwm(voltage);
}

// ######   PUBLIC FUNCTIONS    ###### //

