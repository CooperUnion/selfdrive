#include "bl_tools.h"

#include <esp_ota_ops.h>

#include "common.h"

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
