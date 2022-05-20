#include "io/can.h"

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

    return ESP_OK;
}
