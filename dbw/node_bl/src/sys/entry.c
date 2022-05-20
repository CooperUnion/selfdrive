#include <esp_err.h>

#include "io/can.h"
#include "sys/bl.h"

void app_main()
{
    esp_err_t err;

    err = bl_init();
    err = can_init();
}
