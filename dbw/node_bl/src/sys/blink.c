#include "sys/blink.h"

#include <driver/gpio.h>
#include <driver/timer.h>
#include <stdbool.h>

#include "sys/timing.h"

// ######        DEFINES        ###### //

#define LED1_PIN 32
#define LED2_PIN 33

#define PULSE_TIMEOUT_US 100000

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static uint64_t blink_cur_timer_val;
static uint64_t blink_prv_timer_val;

static bool led1_state;
static bool led2_state;

// ######          CAN          ###### //

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

esp_err_t blink_init(void)
{
    esp_err_t err;

    gpio_pad_select_gpio(LED1_PIN);
    gpio_pad_select_gpio(LED2_PIN);

    err = gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) return err;
    err = gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) return err;

    err = gpio_set_level(LED1_PIN, led1_state);
    if (err != ESP_OK) return err;
    err = gpio_set_level(LED2_PIN, led2_state);
    if (err != ESP_OK) return err;

    err = timer_get_counter_value(TIMING_GROUP, US_TIMER, &blink_prv_timer_val);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t blink_pulse(void)
{
    static unsigned int pulses;

    esp_err_t err;

    err = timer_get_counter_value(TIMING_GROUP, US_TIMER, &blink_cur_timer_val);
    if (err != ESP_OK) return err;

    if (blink_cur_timer_val - blink_prv_timer_val < PULSE_TIMEOUT_US)
        return ESP_OK;

    blink_prv_timer_val = blink_cur_timer_val;
    ++pulses;

    led1_state = !led1_state;
    err = gpio_set_level(LED1_PIN, led1_state);
    if (err != ESP_OK) return err;

    if (pulses % 2) return ESP_OK;

    led2_state = !led2_state;
    err = gpio_set_level(LED2_PIN, led2_state);
    if (err != ESP_OK) return err;

    return ESP_OK;
}
