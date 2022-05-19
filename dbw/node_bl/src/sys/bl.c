#include "bl.h"

#include <esp_ota_ops.h>
#include <stdbool.h>
#include <stdint.h>

#include "base/base.h"
#include "io/can.h"
#include "module_types.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

#define TIMEOUT_MS 2000

// ######      PROTOTYPES       ###### //

static void bl_init();
static void bl_10Hz();
static void bl_1kHz();

// ######     PRIVATE DATA      ###### //

static bool ota_ready;

static uint32_t can_BL_Magic_Packet_delta_ms;
static uint32_t can_BL_Data_Frame_delta_ms;

static const esp_partition_t *ota_partition;

// ######          CAN          ###### //

static struct CAN_dbwBL_Magic_Packet_t CAN_BL_Magic_Packet;

static can_incoming_t can_BL_Magic_Packet_cfg = {
    .id = CAN_DBWBL_MAGIC_PACKET_FRAME_ID,
    .out = &CAN_BL_Magic_Packet,
    .unpack = CAN_dbwBL_Magic_Packet_unpack,
    .delta_ms = &can_BL_Magic_Packet_delta_ms,
};

static struct CAN_dbwBL_Data_Frame_t CAN_BL_Data_Frame;

static can_incoming_t can_BL_Data_Frame_cfg = {
    .id = CAN_DBWBL_DATA_FRAME_FRAME_ID,
    .out = &CAN_BL_Data_Frame,
    .unpack = CAN_dbwBL_Data_Frame_unpack,
    .delta_ms = &can_BL_Data_Frame_delta_ms,
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
    .call_init = bl_init,
    .call_10Hz = bl_10Hz,
    .call_1kHz = bl_1kHz,
};

static void bl_init()
{
    base_set_state_bl();

    can_BL_Magic_Packet_cfg.id += FIRMWARE_MODULE_IDENTITY;
    can_BL_Data_Frame_cfg.id   += FIRMWARE_MODULE_IDENTITY;
    can_BL_Metadata_cfg.id     += FIRMWARE_MODULE_IDENTITY;

    can_register_incoming_msg(can_BL_Magic_Packet_cfg);
    can_register_incoming_msg(can_BL_Data_Frame_cfg);

    ota_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_OTA_0,
        NULL
    );
}

static void bl_10Hz()
{
    if (!CAN_BL_Magic_Packet.Size) {
        if (can_BL_Magic_Packet_delta_ms >= TIMEOUT_MS) {
            // if our OTA partition is invalid changing the boot
            // partition will fail and we'll boot into the bootloader
            // anyways
            esp_ota_set_boot_partition(ota_partition);
            esp_restart();
        }
    } else if (!ota_ready) {
        ota_ready = true;
        CAN_BL_Metadata.Ready = 1;
        CAN_BL_Metadata.Done  = 0;
        CAN_BL_Metadata.Abort = 0;
        CAN_BL_Metadata.ACK   = 0;
        can_send_iface(&can_BL_Metadata_cfg, &CAN_BL_Metadata);
    }
}

static void bl_1kHz()
{
    if (!ota_ready) return;
}

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //
