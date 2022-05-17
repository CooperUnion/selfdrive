#include "bl.h"

#include <stdbool.h>

#include "base/base.h"
#include "io/can.h"
#include "module_types.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

static void bl_init();
static void bl_10Hz();

// ######     PRIVATE DATA      ###### //

static bool ota_ready = true;

// ######          CAN          ###### //

static struct CAN_dbwBL_Magic_Packet_t CAN_BL_Magic_Packet;

static can_incoming_t can_BL_Magic_Packet_cfg = {
    .id = CAN_DBWBL_MAGIC_PACKET_FRAME_ID,
    .out = &CAN_BL_Magic_Packet,
    .unpack = CAN_dbwBL_Magic_Packet_unpack,
};

static struct CAN_dbwBL_Data_Frame_t CAN_BL_Data_Frame;

static can_incoming_t can_BL_Data_Frame_cfg = {
    .id = CAN_DBWBL_DATA_FRAME_FRAME_ID,
    .out = &CAN_BL_Data_Frame,
    .unpack = CAN_dbwBL_Data_Frame_unpack,
};

static struct CAN_dbwBL_Metadata_t CAN_BL_Metadata;

static can_outgoing_t can_BL_Metadata_cfg = {
    .id = CAN_DBWBL_METADATA_FRAME_ID,
    .extd = CAN_DBWBL_METADATA_IS_EXTENDED,
    .dlc = CAN_DBWBL_METADATA_LENGTH,
    .pack = CAN_dbwBL_Metadata_pack,
};

// ######    RATE FUNCTIONS     ###### //

const struct rate_funcs bl_rf = {
    .call_init  = bl_init,
    .call_10Hz  = bl_10Hz,
};

static void bl_init()
{
    base_set_state_bl();

    can_BL_Magic_Packet_cfg.id += FIRMWARE_MODULE_IDENTITY;
    can_BL_Data_Frame_cfg.id   += FIRMWARE_MODULE_IDENTITY;
    can_BL_Metadata_cfg.id     += FIRMWARE_MODULE_IDENTITY;

    can_register_incoming_msg(can_BL_Magic_Packet_cfg);
    can_register_incoming_msg(can_BL_Data_Frame_cfg);
}

static void bl_10Hz()
{
    if (ota_ready && CAN_BL_Magic_Packet.Size) {
        ota_ready = false;
        CAN_BL_Metadata.Ready = 1;
        can_send_iface(&can_BL_Metadata_cfg, &CAN_BL_Metadata);
    }
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
