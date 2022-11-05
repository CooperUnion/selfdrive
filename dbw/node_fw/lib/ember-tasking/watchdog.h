#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <freertos/FreeRTOS.h>

void task_1Hz_wdt_kick();
void task_10Hz_wdt_kick();
void task_100Hz_wdt_kick();
void task_1kHz_wdt_kick();

void IRAM_ATTR task_wdt_servicer();

void set_up_rtc_watchdog_for_init();
void set_up_rtc_watchdog_final();
void set_up_rtc_watchdog_fwupdate();

void set_up_rtc_watchdog(uint32_t timeout_ms);

#endif
