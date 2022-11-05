#include "watchdog.h"

#include <freertos/FreeRTOS.h>
#include <hal/wdt_hal.h>
#include <soc/rtc.h>

#include "common.h"

/*
 * watchdog - Save the system if all else fails
 */

// ######        DEFINES        ###### //

// note on init timeout: init currently takes <1ms
// note: anything <25ms or so will screw with JTAG
// and you'll have a bad time
#define INIT_TIMEOUT_MS 25
#define FINAL_TIMEOUT_MS 25
#define FW_UPDATE_TIMEOUT_MS 1000

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static volatile bool task_1Hz_checkin;
static volatile bool task_10Hz_checkin;
static volatile bool task_100Hz_checkin;
static volatile bool task_1kHz_checkin;

static wdt_hal_context_t wdt_ctx;

// ######   PRIVATE FUNCTIONS   ###### //

/*
 * Kick hardware watchdog.
 */
static IRAM_ATTR void kick_rtc_watchdog()
{
    wdt_hal_write_protect_disable(&wdt_ctx);
    wdt_hal_feed(&wdt_ctx);
    wdt_hal_write_protect_enable(&wdt_ctx);
}

// ######   PUBLIC FUNCTIONS    ###### //

/*
 * Set up and activate the rtc watchdog with the given timeout. Note that there
 * can only be one watchdog active at a time - this function will override any
 * previous active watchdog.
 *
 * Refer to:
 * https://github.com/espressif/esp-idf/blob/master/components/hal/include/hal/wdt_hal.h
 * https://github.com/espressif/esp-idf/blob/4c3322ed6003b87422bd79fe9e67f1cb347c6e2f/components/esp_hw_support/rtc_wdt.c#L91
 */
void set_up_rtc_watchdog(uint timeout_ms)
{
    const uint32_t timeout_ticks =
        (uint32_t) ((uint64_t) rtc_clk_slow_freq_get_hz() * timeout_ms / 1000);

    // it's okay to call wdt_hal_init() on an already-active timer. It will
    // destroy the existing context and stop all active RTC watchdogs before
    // setting up again.
    wdt_hal_init(&wdt_ctx, WDT_RWDT, 0, false);

    wdt_hal_write_protect_disable(&wdt_ctx);
    wdt_hal_config_stage(&wdt_ctx, WDT_STAGE0, timeout_ticks, WDT_STAGE_ACTION_RESET_RTC);
    wdt_hal_enable(&wdt_ctx);
    wdt_hal_write_protect_enable(&wdt_ctx);
}

void task_1Hz_wdt_kick()
{
    task_1Hz_checkin = true;
}

void task_10Hz_wdt_kick()
{
    task_10Hz_checkin = true;
}

void task_100Hz_wdt_kick()
{
    task_100Hz_checkin = true;
}

void task_1kHz_wdt_kick()
{
    task_1kHz_checkin = true;
}

/*
 * Checks task checkins and ensures timing deadlines were met. Incorrect timing
 * will latch (static bool violated) and cause the watchdog to reset the system.
 * 
 * Run this function in the 1kHz interrupt.
 */
void IRAM_ATTR task_wdt_servicer()
{
    // maintain count of ticks since last checkin/kick
    static uint tick_count_1kHz = 0;
    static uint tick_count_100Hz = 0;
    static uint tick_count_10Hz = 0;
    static uint tick_count_1Hz = 0;

    tick_count_1kHz++;
    tick_count_100Hz++;
    tick_count_10Hz++;
    tick_count_1Hz++;

    if (task_1kHz_checkin) {
        tick_count_1kHz = 0;
        task_1kHz_checkin = false;
    }

    if (task_100Hz_checkin) {
        tick_count_100Hz = 0;
        task_100Hz_checkin = false;
    }

    if (task_10Hz_checkin) {
        tick_count_10Hz = 0;
        task_10Hz_checkin = false;
    }

    if (task_1Hz_checkin) {
        tick_count_1Hz = 0;
        task_1Hz_checkin = false;
    }

    /*
     * Below, we check if any of the tasks have exceeded their timing limits.
     * For example, there are at most 1000 ticks permitted between each checkin
     * of the 1Hz task. 1001 ticks passing would mean that the 1Hz task had more
     * than 1 second between runs.
     *
     * For the 1Hz task: (1kHz watchdog tick rate)/(1Hz task rate) = 1000
     */

    static bool violated; // static so we latch forever

    violated |= tick_count_1kHz > 1;
    violated |= tick_count_100Hz > 10;
    violated |= tick_count_10Hz > 100;
    violated |= tick_count_1Hz > 1000;

    if (!violated) {
        kick_rtc_watchdog();
    }
    // implicit else is that the watchdog resets the SoC shortly after
}

/*
 * Configure the watchdog with a temporarily larger timeout so we can run the
 * taskinig init and call_init()'s and have some insurance against the system 
 * hanging while we do it.
 */
void set_up_rtc_watchdog_for_init()
{
    set_up_rtc_watchdog(INIT_TIMEOUT_MS);
}

/*
 * Configure the watchdog with the final task-running timeout. 
 *
 * You're on very strict time once this happens!
 */
void set_up_rtc_watchdog_final()
{
    set_up_rtc_watchdog(FINAL_TIMEOUT_MS);
}

void set_up_rtc_watchdog_1sec()
{
    set_up_rtc_watchdog(FW_UPDATE_TIMEOUT_MS);
}

