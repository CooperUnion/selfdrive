#include <driver/gpio.h>

#include "ember_taskglue.h"
#include "node_pins.h"

// ######        DEFINES        ###### //

#define LED1_PIN NODE_BOARD_PIN_LED1
#define LED2_PIN NODE_BOARD_PIN_LED2

// ######      PROTOTYPES       ###### //

static void leds_init(void);
static void leds_10Hz(void);

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

// ######    RATE FUNCTIONS     ###### //

ember_rate_funcs_S leds_rf = {
    .call_init = leds_init,
    .call_10Hz = leds_10Hz,
};

static void leds_init(void) {
    gpio_config(&(gpio_config_t){
        .pin_bit_mask = BIT64(LED1_PIN) | BIT64(LED2_PIN),
        .mode = GPIO_MODE_OUTPUT,
    });

    gpio_set_level(LED1_PIN, 0);
    gpio_set_level(LED2_PIN, 0);
}

static void leds_10Hz(void) {
    static bool led1;
    static bool led2;

    led1 = !led1;
    led2 = !led2;

    gpio_set_level(LED1_PIN, led1);
    gpio_set_level(LED2_PIN, led2);
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
