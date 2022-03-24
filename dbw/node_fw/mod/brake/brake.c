#include "brake.h"

#include "common.h"
#include "module_types.h"
#include "sys/task_glue.h"

/* Define firmware module identity for the entire build. */
const enum firmware_module_types FIRMWARE_MODULE_IDENTITY = MOD_BRAKE;

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

static void brake_init();

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

// ######    RATE FUNCTIONS     ###### //

struct rate_funcs module_rf = {
    .call_init = brake_init,
};

static void brake_init()
{

}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

