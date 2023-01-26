#include "pb_mon.h"

#include <driver/gpio.h>

#include "cuber_nodetypes.h"
#include "ember_taskglue.h"
#include "opencan_tx.h"

/* Define firmware module identity for the entire build. */
const enum cuber_node_types CUBER_NODE_IDENTITY = NODE_PB_MON;

// ######        DEFINES        ###### //

#define INDUCTIVE_PROX_GPIO 34
#define MAGNET_GPIO         35

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

// ######    RATE FUNCTIONS     ###### //

static void pb_mon_init();

ember_rate_funcs_S module_rf = {
    .call_init  = pb_mon_init,
};

static void pb_mon_init()
{
    gpio_set_direction(INDUCTIVE_PROX_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(MAGNET_GPIO,         GPIO_MODE_INPUT);

    gpio_pulldown_en(INDUCTIVE_PROX_GPIO);
    gpio_pulldown_en(MAGNET_GPIO);
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

// ######        CAN TX         ###### //

void CANTX_populate_PB_ParkingBrakeData(struct CAN_Message_PB_ParkingBrakeData * const m) {
    bool pb_set      = gpio_get_level(INDUCTIVE_PROX_GPIO);
    bool magnet_on   = gpio_get_level(MAGNET_GPIO);

    m->PB_pbSet           = pb_set;
    m->PB_magnetEnergized = magnet_on;
    m->PB_armedESTOP      = pb_set && magnet_on;
}
