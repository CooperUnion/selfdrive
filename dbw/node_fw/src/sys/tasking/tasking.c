#include "sys/tasking/tasking.h"

#include <driver/timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "common.h"
#include "sys/tasking/module_list.h"

// ######        DEFINES        ###### //

#define TIMER_GROUP 0, 0

// ######      PROTOTYPES       ###### //

static void create_tasking_interrupt();
static bool IRAM_ATTR task_granter(void* unused);

// ######     PRIVATE DATA      ###### //

// todo: change to task notifications
static SemaphoreHandle_t sem_1Hz = NULL;
static SemaphoreHandle_t sem_10Hz = NULL;
static SemaphoreHandle_t sem_100Hz = NULL;
static SemaphoreHandle_t sem_1kHz = NULL;

// ######   PRIVATE FUNCTIONS   ###### //

/*
 * This is an interrupt triggered by a timer (set up in entry.c) at 1kHz. It
 * maintains a tick count and gives semaphores to the rate runners based on how
 * many ticks have passed. This means every 1, 10, 100, 1000 ticks, we give the
 * semaphores for the 1kHz, 100Hz, 10Hz, and 1Hz tasks respectively.
 */
static bool IRAM_ATTR task_granter(void* unused)
{
    static uint32_t tick_count = 0;

    // check value and give corresponding semaphores

    tick_count++;

    if (!(tick_count % 1)) { // 1kHz (redundant logic)
        xSemaphoreGiveFromISR(sem_1kHz, NULL);
    }

    if (!(tick_count % 10)) { // 100Hz
        xSemaphoreGiveFromISR(sem_100Hz, NULL);
    }

    if (!(tick_count % 100)) { // 10Hz
        xSemaphoreGiveFromISR(sem_10Hz, NULL);
    }

    if (!(tick_count % 1000)) { // 1Hz
        xSemaphoreGiveFromISR(sem_1Hz, NULL);
        tick_count = 0; // reset because this is the least frequent rate
    }

    return true; // indicate that we want to yield, maybe change this later
}

/*
 * Set up (and start!) the 1kHz timer interrupt task_granter().
 *
 * The module rate functions will begin to run!
 */
static void create_tasking_interrupt()
{
    const timer_config_t granter_cfg = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80, // 80MHz/80 = 1MHz timer rate
    };

    timer_init(TIMER_GROUP, &granter_cfg);
    timer_set_counter_value(TIMER_GROUP, 0);
    timer_set_alarm_value(TIMER_GROUP, 1000); // alarm at timer rate / 1000 = 1MHz/1000=1kHz
    timer_enable_intr(TIMER_GROUP);
    timer_isr_callback_add(TIMER_GROUP, task_granter, NULL, ESP_INTR_FLAG_IRAM);

    timer_start(TIMER_GROUP);
}

/**********************
 * Module rate runners.
 **********************/

static void modules_init()
{
    for (uint i = 0; i < ARRAY_SIZE(task_list); i++) {
        if (task_list[i]->call_init)
            task_list[i]->call_init();
    }
}

static void module_runner_1Hz()
{
    for (;;) {
        if (xSemaphoreTake(sem_1Hz, portMAX_DELAY) == pdTRUE) {
            for (uint i = 0; i < ARRAY_SIZE(task_list); i++) {
                if (task_list[i]->call_1Hz)
                    task_list[i]->call_1Hz();
            }
        }
    }
}

static void module_runner_10Hz()
{
    for (;;) {
        if (xSemaphoreTake(sem_10Hz, portMAX_DELAY) == pdTRUE) {
            for (uint i = 0; i < ARRAY_SIZE(task_list); i++) {
                if (task_list[i]->call_10Hz)
                    task_list[i]->call_10Hz();
            }
        }
    }
}

static void module_runner_100Hz()
{
    for (;;) {
        if (xSemaphoreTake(sem_100Hz, portMAX_DELAY) == pdTRUE) {
            for (uint i = 0; i < ARRAY_SIZE(task_list); i++) {
                if (task_list[i]->call_100Hz)
                    task_list[i]->call_100Hz();
            }
        }
    }
}

static void module_runner_1kHz()
{
    for (;;) {
        if (xSemaphoreTake(sem_1kHz, portMAX_DELAY) == pdTRUE) {
            for (uint i = 0; i < ARRAY_SIZE(task_list); i++) {
                if (task_list[i]->call_1kHz)
                    task_list[i]->call_1kHz();
            }
        }
    }
}

// ######   PUBLIC FUNCTIONS    ###### //

/*
 * Tasking setup and initialization of all modules.
 *
 * ! Take care to make sure the watchdog is active before we call this, because
 * otherwise the module inits could hang the system without triggering a reset.
 */
void tasking_init()
{
    // maybe make the semaphores and tasks static in the future
    sem_1Hz = xSemaphoreCreateBinary();
    sem_10Hz = xSemaphoreCreateBinary();
    sem_100Hz = xSemaphoreCreateBinary();
    sem_1kHz = xSemaphoreCreateBinary();

    static TaskHandle_t module_runner_1Hz_handle;
    static TaskHandle_t module_runner_10Hz_handle;
    static TaskHandle_t module_runner_100Hz_handle;
    static TaskHandle_t module_runner_1kHz_handle;

    xTaskCreatePinnedToCore(module_runner_1Hz, "1_HZ", 2000, 0, 3, &module_runner_1Hz_handle, 0);
    xTaskCreatePinnedToCore(module_runner_10Hz, "10_HZ", 2000, 0, 2, &module_runner_10Hz_handle, 0);
    xTaskCreatePinnedToCore(module_runner_100Hz, "100_HZ", 2000, 0, 1, &module_runner_100Hz_handle, 0);
    xTaskCreatePinnedToCore(module_runner_1kHz, "1k_HZ", 2000, 0, 0, &module_runner_1kHz_handle, 0);

    modules_init();
    create_tasking_interrupt();
}
