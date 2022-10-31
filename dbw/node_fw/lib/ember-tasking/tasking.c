#include "tasking.h"
#include "ember_taskglue.h"

#include <driver/timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "common.h"
#include "watchdog.h"

// ######        DEFINES        ###### //

#define TIMER_GROUP 0, 0
#define TASK_STACK_SIZE 4000

// ######      PROTOTYPES       ###### //

static void create_tasking_interrupt();

static IRAM_ATTR void module_runner_1Hz();
static IRAM_ATTR void module_runner_10Hz();
static IRAM_ATTR void module_runner_100Hz();
static IRAM_ATTR void module_runner_1kHz();

static bool IRAM_ATTR task_granter(void* unused);

// ######     PRIVATE DATA      ###### //

// todo: change to task notifications
static SemaphoreHandle_t sem_1Hz = NULL;
static SemaphoreHandle_t sem_10Hz = NULL;
static SemaphoreHandle_t sem_100Hz = NULL;
static SemaphoreHandle_t sem_1kHz = NULL;

extern ember_rate_funcs_S* ember_task_list[];
extern const size_t ember_task_count;

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

    task_wdt_servicer();

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
 * Set up the 1kHz timer interrupt task_granter(), but don't start it yet.
 * 
 * After this, it's ready to go - call timer_start(TIMER_GROUP) to start it.
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
}

/**********************
 * Module rate runners.
 **********************/

static IRAM_ATTR void module_runner_1Hz()
{
    for (;;) {
        if (xSemaphoreTake(sem_1Hz, portMAX_DELAY) == pdTRUE) {
            for (uint i = 0; i < ember_task_count; i++) {
                if (ember_task_list[i]->call_1Hz)
                    ember_task_list[i]->call_1Hz();
            }
            task_1Hz_wdt_kick();
        }
    }
}

static IRAM_ATTR void module_runner_10Hz()
{
    for (;;) {
        if (xSemaphoreTake(sem_10Hz, portMAX_DELAY) == pdTRUE) {
            for (uint i = 0; i < ember_task_count; i++) {
                if (ember_task_list[i]->call_10Hz)
                    ember_task_list[i]->call_10Hz();
            }
            task_10Hz_wdt_kick();
        }
    }
}

static IRAM_ATTR void module_runner_100Hz()
{
    for (;;) {
        if (xSemaphoreTake(sem_100Hz, portMAX_DELAY) == pdTRUE) {
            for (uint i = 0; i < ember_task_count; i++) {
                if (ember_task_list[i]->call_100Hz)
                    ember_task_list[i]->call_100Hz();
            }
            task_100Hz_wdt_kick();
        }
    }
}

static IRAM_ATTR void module_runner_1kHz()
{
    for (;;) {
        if (xSemaphoreTake(sem_1kHz, portMAX_DELAY) == pdTRUE) {
            for (uint i = 0; i < ember_task_count; i++) {
                if (ember_task_list[i]->call_1kHz)
                    ember_task_list[i]->call_1kHz();
            }
            task_1kHz_wdt_kick();
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

    // we need to give more thought to the priorities here, I think
    xTaskCreatePinnedToCore(module_runner_1Hz, "1_HZ", TASK_STACK_SIZE, 0, 0, &module_runner_1Hz_handle, 0);
    xTaskCreatePinnedToCore(module_runner_10Hz, "10_HZ", TASK_STACK_SIZE, 0, 0, &module_runner_10Hz_handle, 0);
    xTaskCreatePinnedToCore(module_runner_100Hz, "100_HZ", TASK_STACK_SIZE, 0, 0, &module_runner_100Hz_handle, 0);
    xTaskCreatePinnedToCore(module_runner_1kHz, "1k_HZ", TASK_STACK_SIZE, 0, 0, &module_runner_1kHz_handle, 0);

    create_tasking_interrupt();
}

/*
 * Run the call_init()'s.
 */
void modules_init()
{
    for (uint i = 0; i < ember_task_count; i++) {
        if (ember_task_list[i]->call_init)
            ember_task_list[i]->call_init();
    }
}

/*
 * Enable the tasking timer interrupt. Rate functions will start to run!
 */
void tasking_begin()
{
    timer_start(TIMER_GROUP);
}
