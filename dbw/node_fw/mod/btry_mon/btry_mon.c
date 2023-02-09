#include "btry_mon.h"

#include <driver/gpio.h>

#include "cuber_nodetypes.h"
#include "ember_taskglue.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

// ######    RATE FUNCTIONS     ###### //

static void btry_mon_init();

ember_rate_funcs_S module_rf = {
    .call_init  = btry_mon_init,
};

static void btry_mon_init()
{

}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //
