#include "sys/timing.h"

#include <driver/timer.h>
#include <esp_err.h>

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

esp_err_t timing_init(void)
{
    esp_err_t err;

    const timer_config_t timer_config = {
        .alarm_en    = TIMER_ALARM_DIS,
        .counter_en  = TIMER_PAUSE,
        .intr_type   = TIMER_INTR_NONE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_DIS,
        .divider     = 80,  // 1E-6 * 80MHz (microseconds)
    };
    err = timer_init(TIMING_GROUP, US_TIMER, &timer_config);
    if (err != ESP_OK) return err;

    err = timer_start(TIMING_GROUP, US_TIMER);
    if (err != ESP_OK) return err;

    return ESP_OK;
}
