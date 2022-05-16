#include "module_types.h"
#include "sys/tasking.h"


const enum firmware_module_types FIRMWARE_MODULE_IDENTITY = IGVC_MODULE_TYPE;


void app_main()
{
    tasking_init();
    modules_init();

    tasking_begin();
}
