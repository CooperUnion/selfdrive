#include "common.h"
#include "ember_bltools.h"
#include "ember_tasking.h"
#include "module_list.inc"

#include <stdio.h>

void app_main()
{
    fprintf(stderr, "Hi :)\n");
    /* set boot partition back to bootloader */
    // ember_bltools_set_boot_partition_to_factory();

    /* begin running tasks */
    ember_tasking_begin();
}

