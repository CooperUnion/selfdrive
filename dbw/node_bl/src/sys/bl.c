#include "sys/bl.h"

#include <driver/timer.h>
#include <esp_err.h>
#include <esp_ota_ops.h>
#include <esp_system.h>

#include "io/can.h"
#include "sys/timing.h"

// ######        DEFINES        ###### //

#define MAGIC_PACKET_TIMEOUT_US 2000000

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static uint64_t bl_prv_timer_val;
static uint64_t bl_cur_timer_val;

static const esp_partition_t *ota_partition;

// ######          CAN          ###### //

static struct CAN_dbwBL_Magic_Packet_t CAN_BL_Magic_Packet;

static can_incoming_t can_BL_Magic_Packet_cfg = {
    .id = CAN_DBWBL_MAGIC_PACKET_FRAME_ID,
    .out = &CAN_BL_Magic_Packet,
    .unpack = CAN_dbwBL_Magic_Packet_unpack,
};

static struct CAN_dbwBL_Metadata_t CAN_BL_Metadata;

static can_outgoing_t can_BL_Metadata_cfg = {
    .id = CAN_DBWBL_METADATA_FRAME_ID,
    .extd = CAN_DBWBL_METADATA_IS_EXTENDED,
    .dlc = CAN_DBWBL_METADATA_LENGTH,
    .pack = CAN_dbwBL_Metadata_pack,
};

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

esp_err_t bl_init(void)
{
    esp_err_t err;

    ota_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_OTA_0,
        NULL
    );
    if (!ota_partition) return ESP_FAIL;

    can_BL_Magic_Packet_cfg.id += IGVC_MODULE_TYPE;
    can_BL_Metadata_cfg.id     += IGVC_MODULE_TYPE;

    can_register_incoming_msg(can_BL_Magic_Packet_cfg);

    err = timer_get_counter_value(TIMING_GROUP, US_TIMER, &bl_prv_timer_val);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t bl_magic_wait(void)
{
    esp_err_t err;

    err = timer_get_counter_value(TIMING_GROUP, US_TIMER, &bl_cur_timer_val);
    if (err != ESP_OK) return err;

    if (!CAN_BL_Magic_Packet.Size) {
        if (bl_cur_timer_val - bl_prv_timer_val >= MAGIC_PACKET_TIMEOUT_US) {
            CAN_BL_Metadata.Ready = 0;
            CAN_BL_Metadata.Done  = 0;
            CAN_BL_Metadata.Abort = 1;
            CAN_BL_Metadata.ACK   = 0;
            can_send_iface(&can_BL_Metadata_cfg, &CAN_BL_Metadata);
            bl_restart();
        }

        return ESP_FAIL;
    }

    CAN_BL_Metadata.Ready = 1;
    CAN_BL_Metadata.Done  = 0;
    CAN_BL_Metadata.Abort = 0;
    CAN_BL_Metadata.ACK   = 0;
    err = can_send_iface(&can_BL_Metadata_cfg, &CAN_BL_Metadata);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

void bl_restart(void)
{
    // if our OTA partition is invalid changing the boot partition will
    // fail and we'll boot into the bootloader anyways
    esp_ota_set_boot_partition(ota_partition);
    esp_restart();
}
