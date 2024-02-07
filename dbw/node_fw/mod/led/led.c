#include "led.h"

#include <driver/gpio.h>

#include "ember_common.h"
#include "ember_taskglue.h"

// ######        DEFINES        ###### //
#define LED_GPIO 18

// ######      PROTOTYPES       ###### //
static void led_init();
static void led_1Hz();

// ######     PRIVATE DATA      ###### //
static int LED_STATUS;


// ######    RATE FUNCTIONS     ###### //
ember_rate_funcs_S module_rf = {
    .call_init = led_init,
    .call_1Hz = led_1Hz,
};

static void led_init() {
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

static void led_1Hz() {
    LED_STATUS = !LED_STATUS;
    gpio_set_level(LED_GPIO, LED_STATUS);
}
// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //
