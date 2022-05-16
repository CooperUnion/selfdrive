#include "bl_tools.h"

#include <esp_ota_ops.h>

#include "common.h"
#include "sys/task_glue.h"

// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

static void bl_tools_100Hz();

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

// ######    RATE FUNCTIONS     ###### //

const struct rate_funcs bl_tools_rf = {
    .call_100Hz = bl_tools_100Hz,
};

static void bl_tools_100Hz()
{
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
