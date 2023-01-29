#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>

#include "ember_tasking.h"
#include "module_list.inc"

#include "ember_can_callbacks.h"

void ember_can_callback_notify_lost_can() {
    /* stub */
}

void app_main() {
    /* begin running tasks */
    ember_tasking_begin();

    // for (;;) {
    //     printf("Hello :)\n");
    //     vTaskDelay(200 / portTICK_PERIOD_MS);
    // }
}
