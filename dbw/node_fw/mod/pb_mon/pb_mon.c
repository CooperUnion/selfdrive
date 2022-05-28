#include "pb_mon.h"

#include <driver/gpio.h>

#include "io/can.h"
#include "module_types.h"
#include "sys/task_glue.h"

/* Define firmware module identity for the entire build. */
const enum firmware_module_types FIRMWARE_MODULE_IDENTITY = MOD_PB_MON;

// ######        DEFINES        ###### //

#define INDUCTIVE_PROX_GPIO 34
#define MAGNET_GPIO         35

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

static struct CAN_dbwNode_PbMon_Data_t CAN_PbMon;

static const can_outgoing_t can_PbMon_Data_cfg = {
    .id = CAN_DBWNODE_PBMON_DATA_FRAME_ID,
    .extd = CAN_DBWNODE_PBMON_DATA_IS_EXTENDED,
    .dlc = CAN_DBWNODE_PBMON_DATA_LENGTH,
    .pack = CAN_dbwNode_PbMon_Data_pack,
};

// ######    RATE FUNCTIONS     ###### //

static void pb_mon_init();
static void pb_mon_100Hz();

const struct rate_funcs module_rf = {
    .call_init  = pb_mon_init,
    .call_100Hz = pb_mon_100Hz,
};

static void pb_mon_init()
{
    gpio_set_direction(INDUCTIVE_PROX_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(MAGNET_GPIO,         GPIO_MODE_INPUT);

    gpio_pulldown_en(INDUCTIVE_PROX_GPIO);
    gpio_pulldown_en(MAGNET_GPIO);
}

static void pb_mon_100Hz()
{
    CAN_PbMon.PbSet           = gpio_get_level(INDUCTIVE_PROX_GPIO);
    CAN_PbMon.MagnetEnergized = gpio_get_level(MAGNET_GPIO);

    CAN_PbMon.ArmedESTOP = CAN_PbMon.PbSet && CAN_PbMon.MagnetEnergized;

    can_send_iface(&can_PbMon_Data_cfg, &CAN_PbMon);
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
