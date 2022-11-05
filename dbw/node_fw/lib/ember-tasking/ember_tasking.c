#include "tasking.h"
#include "watchdog.h"


void ember_tasking_begin(void) {
  set_up_rtc_watchdog_for_init();
  tasking_init();
  modules_init();

  set_up_rtc_watchdog_final();
  tasking_begin();
}

void ember_tasking_set_1sec_watchdog(void) {
  set_up_rtc_watchdog(1000);
}

