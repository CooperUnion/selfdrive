#include "can.h"

#include <driver/twai.h>
#include <stdio.h>

#include "common.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

#define CAN_TX_GPIO 19
#define CAN_RX_GPIO 18

// ######      PROTOTYPES       ###### //

static void can_init();
static void can_10Hz_ping();

// ######     PRIVATE DATA      ###### //

// ######    RATE FUNCTIONS     ###### //

struct rate_funcs can_rf = {
    .call_init = can_init,
    .call_10Hz = can_10Hz_ping,
};

static void can_init()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    gpio_pad_select_gpio(32);
    gpio_set_direction(32, GPIO_MODE_OUTPUT);
    gpio_set_level(32, 0);

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        // ok
    } else {
        // not ok
    }

    if (twai_start() == ESP_OK) {
        // ok
    } else {
        // not ok
    }
}

static void can_10Hz_ping()
{
    static uint8_t count = 0;

    twai_message_t message;
    message.extd = 1;

    message.identifier = 0xAAAA;

    message.data[0] = 0;
    message.data[1] = 0;
    message.data[2] = 0;
    message.data[3] = count++;

    esp_err_t r = twai_transmit(&message, pdMS_TO_TICKS(5));

    if (r == ESP_OK) {
        gpio_set_level(32, 0);
    } else {
        gpio_set_level(32, 1);
    }
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
