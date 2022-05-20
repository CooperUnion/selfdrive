#include "io/can.h"

#include <driver/timer.h>
#include <driver/twai.h>
#include <esp_err.h>
#include <stdbool.h>

// ######        DEFINES        ###### //

#define CAN_TX_GPIO 19
#define CAN_RX_GPIO 18

#define CAN_MAX_IN_MSGS 24

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static can_incoming_t in_msgs[CAN_MAX_IN_MSGS];
static unsigned int   in_msgs_cnt;

// ######          CAN          ###### //

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

esp_err_t can_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err;

    err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) return err;

    err = twai_start();
    if (err != ESP_OK) return err;

    const timer_config_t timer_config = {
        .alarm_en    = TIMER_ALARM_DIS,
        .counter_en  = TIMER_PAUSE,
        .intr_type   = TIMER_INTR_NONE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_DIS,
        .divider     = 80,  // 1E-6 * 80MHz (microseconds)
    };
    err = timer_init(CAN_TIMER_GROUP, CAN_TIMER, &timer_config);
    if (err != ESP_OK) return err;

    err = timer_start(CAN_TIMER_GROUP, CAN_TIMER);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t can_poll(void)
{
    esp_err_t      err;
    twai_message_t msg;

    while (true) {
        err = twai_receive(&msg, 0);

        if (err == ESP_OK) {
            for (uint i = 0; i < in_msgs_cnt; i++) {
                if (in_msgs[i].id == msg.identifier) {
                    in_msgs[i].unpack(in_msgs[i].out, msg.data, msg.data_length_code);
                    if (in_msgs[i].timer_val) timer_get_counter_value(TIMER_GROUP_1, TIMER_0, in_msgs[i].timer_val);
                }
            }
            continue;
        }

        return err;
    }
}

void can_register_incoming_msg(const can_incoming_t cfg)
{
    in_msgs[in_msgs_cnt] = cfg;
    in_msgs_cnt++;
}
