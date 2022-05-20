#include "bl.h"

#include <esp_err.h>
#include <esp_ota_ops.h>
#include <esp_system.h>

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static const esp_partition_t *ota_partition;

// ######          CAN          ###### //

// ######   PRIVATE FUNCTIONS   ###### //

// ######   PUBLIC FUNCTIONS    ###### //

esp_err_t bl_init(void)
{
    ota_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_OTA_0,
        NULL
    );
    if (!ota_partition) return ESP_FAIL;

    return ESP_OK;
}

void bl_restart(void)
{
    // if our OTA partition is invalid changing the boot partition will
    // fail and we'll boot into the bootloader anyways
    esp_ota_set_boot_partition(ota_partition);
    esp_restart();
}
