#include "led.h"

#include <driver/gpio.h>

#include "ember_common.h"
#include "ember_taskglue.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

static void led_init();
static void led_1Hz();

// ######     PRIVATE DATA      ###### //

// ######    RATE FUNCTIONS     ###### //

/*
 * This struct is filled the pointers to the rate functions.
 *
 * hello_blink_rt is accessed using extern in module_list.h, which is read by
 * tasking.c, which runs the rate tasks. This is not declared as static because
 * of the above.
 *
 * Note that we just don't fill out the fields for the rates we don't need.
 * That's okay - the C standard guarantees they will be set to NULL.
 *
 * Note that it's perfectly okay that the rate functions are declared as static.
 */
ember_rate_funcs_S module_rf = {
    .call_init = led_init,
    .call_1Hz = led_1Hz,
};

/*
 * Initializes the GPIO for the LED.
 *
 * In the future, we should probably centralize GPIO initialization.
 */
static void led_init()
{

}

/*
 * Toggles the LED on each call.
 */
static void led_1Hz()
{

}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
