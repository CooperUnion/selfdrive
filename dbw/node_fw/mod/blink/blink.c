#include "blink.h"

#include <driver/gpio.h>

#include "common.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

static void blink_init();
static void blink_1Hz();

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
struct rate_funcs module_rf = {
    .call_init = blink_init,
    .call_1Hz = blink_1Hz,
};

/*
 * Initializes the GPIO for the LED.
 *
 * In the future, we should probably centralize GPIO initialization.
 */
static void blink_init()
{

}

/*
 * Toggles the LED on each call.
 */
static void blink_1Hz()
{

}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
