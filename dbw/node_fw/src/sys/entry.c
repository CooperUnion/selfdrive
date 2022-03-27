#include "common.h"
#include "sys/bl_tools.h"
#include "sys/tasking.h"
#include "sys/watchdog.h"

void app_main()
{
    /* set boot partition back to bootloader */
    set_up_rtc_watchdog_fwupdate();
    set_boot_partition_to_factory();

    /* initialization */
    set_up_rtc_watchdog_for_init();
    tasking_init();
    modules_init();

    /* begin running tasks */
    set_up_rtc_watchdog_final();
    tasking_begin();
}
