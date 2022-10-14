#include "sys/task_glue.h"

#include <driver/gpio.h>

// ######        DEFINES        ###### //

#define GPIO_LED1 GPIO_NUM_32
#define GPIO_LED2 GPIO_NUM_33

// ######      PROTOTYPES       ###### //

static void bl_init(void);
static void bl_10Hz(void);

static void set_up_leds(void);

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

// ######    RATE FUNCTIONS     ###### //

const struct rate_funcs bl_rf = {
    .call_init = bl_init,
    .call_10Hz = bl_10Hz,
};

static void bl_init(void)
{
    set_up_leds();
    gpio_set_level(GPIO_LED1, 1);
    gpio_set_level(GPIO_LED2, 1);
}

static void bl_10Hz(void)
{
    static bool led;
    gpio_set_level(GPIO_LED2, led);
    led = !led;
}

// ######   PRIVATE FUNCTIONS   ###### //

static void set_up_leds(void)
{
    gpio_pad_select_gpio(GPIO_LED1);
    gpio_pad_select_gpio(GPIO_LED2);

    gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_LED1, 0);
    gpio_set_level(GPIO_LED2, 0);
}

// ######   PUBLIC FUNCTIONS    ###### //


