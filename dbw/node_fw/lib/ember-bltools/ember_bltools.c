#include "ember_bltools.h"

#include <esp_ota_ops.h>

#include "common.h"
#include "ember_tasking.h"

static bool set_boot_partition_to_factory(void) {
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

/*
 * Set the boot partition to factory.
 *
 * NOTE: This uses ember_tasking to create a watchdog. At the moment, we will
 * not support *not* doing this.
 */
bool ember_bltools_set_boot_partition_to_factory(void) {
    ember_tasking_set_1sec_watchdog();
    return set_boot_partition_to_factory();
}
