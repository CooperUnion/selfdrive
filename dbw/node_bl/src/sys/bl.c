#include "bl.h"

#include "base/base.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

static void bl_init();

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

// ######    RATE FUNCTIONS     ###### //

const struct rate_funcs bl_rf = {
    .call_init  = bl_init,
};

static void bl_init()
{
    base_set_state_bl();
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
