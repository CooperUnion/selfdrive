#include <esp_err.h>

#include "bl.h"
#include "can.h"

void app_main()
{
    esp_err_t err;

    err = bl_init();
    err = can_init();
}
