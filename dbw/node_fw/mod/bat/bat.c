#include "bat.h"

#include <esp_attr.h>
#include <driver/gpio.h>

#include "ember_common.h"
#include "cuber_base.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

static void bat_init();

// ######     PRIVATE DATA      ###### //

// ######    RATE FUNCTIONS     ###### //

ember_rate_funcs_S module_rf = {
    .call_init  = bat_init,
};

static void bat_init() {

}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //
