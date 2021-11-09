#include "common.h"
#include "sys/tasking.h"
#include "sys/watchdog.h"

void app_main()
{
    /* initialization */
    set_up_rtc_watchdog_for_init();
    tasking_init();
    modules_init();

    /* begin running tasks */
    set_up_rtc_watchdog_final();
    tasking_begin();
}
