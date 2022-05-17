#include "bl_tools.h"

#include <esp_ota_ops.h>

#include "base/base.h"
#include "common.h"
#include "io/can.h"
#include "module_types.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

static void bl_tools_init();
static void bl_tools_10Hz();

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

static struct CAN_dbwBL_Magic_Packet_t CAN_BL_Magic_Packet;

static can_incoming_t can_BL_Magic_Packet_cfg = {
    .id = CAN_DBWBL_MAGIC_PACKET_FRAME_ID,
    .out = &CAN_BL_Magic_Packet,
    .unpack = CAN_dbwBL_Magic_Packet_unpack,
};

// ######    RATE FUNCTIONS     ###### //

const struct rate_funcs bl_tools_rf = {
    .call_init  = bl_tools_init,
    .call_10Hz = bl_tools_10Hz,
};

static void bl_tools_init()
{
    can_BL_Magic_Packet_cfg.id += FIRMWARE_MODULE_IDENTITY;

    can_register_incoming_msg(can_BL_Magic_Packet_cfg);
}

static void bl_tools_10Hz()
{
    if (CAN_BL_Magic_Packet.Size && !base_dbw_currently_active()) {
        esp_restart();
    }
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

bool set_boot_partition_to_factory() {
   const esp_partition_t *factory = esp_partition_find_first(
       ESP_PARTITION_TYPE_APP,
       ESP_PARTITION_SUBTYPE_APP_FACTORY,
       NULL
    );

   if (!factory) {
       return false;
   }

   esp_err_t err = esp_ota_set_boot_partition(factory);
   if (err != ESP_OK) {
       return false;
       // log error here
   }

   return true;
}
