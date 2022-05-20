#include "io/can.h"

#include <driver/timer.h>
#include <driver/twai.h>
#include <esp_err.h>

// ######        DEFINES        ###### //

#define CAN_TX_GPIO 19
#define CAN_RX_GPIO 18

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

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
