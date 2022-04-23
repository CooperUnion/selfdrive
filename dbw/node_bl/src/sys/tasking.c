#include "sys/tasking.h"

#include "common.h"
#include "sys/module_list.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static const struct rate_funcs* task_list[] = {
    &can_rf,
    &eeprom_rf,
};

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

void modules_init()
{
    for (uint i = 0; i < ARRAY_SIZE(task_list); i++) {
        if (task_list[i]->call_init)
            task_list[i]->call_init();
    }
}
