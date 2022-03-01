#include "can.h"

#include <esp_ota_ops.h>
#include <driver/twai.h>
#include <stdio.h>

#include "base/base.h"
#include "common.h"
#include "sys/task_glue.h"
#include "sys/watchdog.h"

#include "can_gen.h"

// ######        DEFINES        ###### //

#define CAN_TX_GPIO 19
#define CAN_RX_GPIO 18

// ######      PROTOTYPES       ###### //

static void can_send_msg(const twai_message_t *message);

// ######     PRIVATE DATA      ###### //

static bool can_ready;

static can_incoming_t in_msgs[25];
static uint in_msgs_count = 0;

// ######    RATE FUNCTIONS     ###### //

static void can_init();
static void can_100Hz();

const struct rate_funcs can_rf = {
    .call_init = can_init,
    .call_100Hz = can_100Hz,
};

static void can_init()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        can_ready = 1;
    } else {
        can_ready = 0;
    }

    if (twai_start() == ESP_OK) {
        can_ready &= 1;
    } else {
        can_ready &= 0;
    }
}


static void can_100Hz()
{
    for (;;) {
        twai_message_t msg;
        esp_err_t result = twai_receive(&msg, 0);

        switch (result) {
            case ESP_ERR_TIMEOUT: // no messages in queue
                return;

            case ESP_OK:
                for (uint i = 0; i < in_msgs_count; i++) {
                    if (in_msgs[i].id == msg.identifier) {
                        in_msgs[i].unpack(in_msgs[i].out, msg.data, msg.data_length_code);
                    }
                }
                break;

            default:
                return; // error
        }
    }
}

// ######   PRIVATE FUNCTIONS   ###### //

static void can_send_msg(const twai_message_t *message)
{
    esp_err_t r = twai_transmit(message, 0);

    if (r == ESP_OK) {
        base_set_state_good();
    } else {
        base_set_state_lost_can();
        // attempt recovery?
    }
}

// ######   PUBLIC FUNCTIONS    ###### //

void can_register_incoming_msg(const can_incoming_t cfg)
{
    in_msgs[in_msgs_count] = cfg;
    in_msgs_count++;
}

void can_send_iface(const can_outgoing_t *i, const void *s)
{
    twai_message_t msg = {
        .identifier = i->id,
        .extd = i->extd,
        .data_length_code = i->dlc,
    };

    i->pack(msg.data, s, 8);

    can_send_msg(&msg);
}

