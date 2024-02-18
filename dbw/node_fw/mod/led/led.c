// ######        LIBRARIES        ###### //
#include "led.h"

#include <driver/gpio.h>

#include "ember_common.h"
#include "ember_taskglue.h"

// ######        DEFINES        ###### //
//define important constants/macros to be used within the program
//these are available globally within the program

#define LED_GPIO 18 //use a GPIO pin to blink the LED (ensure it is one that is not being used)

// ######      PROTOTYPES       ###### //
//declare non-rate based functions in the program

static void led_init();
static void led_1Hz();

// ######     PRIVATE DATA      ###### //
//define any variables that will not be used globally and only within the program

static int LED_STATUS = 0; //initialize the LED status to off


// ######    RATE FUNCTIONS     ###### //
//Rate functions can run at 1Hz, 10Hz, 100Hz, or 1000Hz (1kHz)
// rate functions can be declared as static because they are only used in this program

ember_rate_funcs_S module_rf = {
    .call_init = led_init,
    .call_1Hz = led_1Hz,
    //.call_10Hz = led_10hz.
    //.call_100Hz = led_100hz,
    //.call_1000Hz = led_1000hz,
};

//Create respective functions for each element defined within the struct above

static void led_init()
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT); //gpio pin, input/output
}

static void led_1Hz()
{
    LED_STATUS = !LED_STATUS; //toggle the LED status
    gpio_set_level(LED_GPIO, LED_STATUS); //set the LED to the new status
}

// ######   PRIVATE FUNCTIONS   ###### //
//functions intended for use only within this .c file

// ######   PUBLIC FUNCTIONS    ###### ///
//functions intended for use within and outside of this .c file

// ######   CAN TX    ###### ///
