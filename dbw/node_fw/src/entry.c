#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include <stdbool.h>

#define BLINK_GPIO 2

SemaphoreHandle_t sem_1Hz = NULL;

// the 1hz in the name here just implies that we will call this at 1hz,
// not that it inherently runs at 1hz
void blink_1Hz()
{
    static bool state = 0;
    gpio_set_level(BLINK_GPIO, state);
    state = !state;
}

void tasks_1Hz()
{
    for (;;)
    {
        if (xSemaphoreTake(sem_1Hz, portMAX_DELAY) == pdTRUE)
        {
            blink_1Hz();
        }
    }
}

bool IRAM_ATTR task_granter(void* unused)
{
    static uint32_t tick_count = 0;

    tick_count++; // check value and release corresponding semaphores

    if (!(tick_count % 1000)) // 1Hz
    {
        xSemaphoreGiveFromISR(sem_1Hz, NULL);
        tick_count = 0; // reset because this is the least frequent rate
    }

    return true; // indicate that we want to yield, maybe change this later
}

void app_main()
{
    sem_1Hz = xSemaphoreCreateBinary();
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    timer_config_t granter_cfg = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80, // 80MHz/80 = 1MHz timer rate
    };

    timer_init(0, 0, &granter_cfg);
    timer_set_counter_value(0, 0, 0);
    timer_set_alarm_value(0, 0, 1000); // alarm at timer rate / 1000 = 1MHz/1000=1kHz
    timer_enable_intr(0, 0);
    timer_isr_callback_add(0, 0, task_granter, NULL, ESP_INTR_FLAG_IRAM);
    timer_start(0, 0);

    TaskHandle_t tasks_1Hz_handle;
    xTaskCreatePinnedToCore(tasks_1Hz, "1_HZ", 2000, 0, 0, &tasks_1Hz_handle, 0);
}
