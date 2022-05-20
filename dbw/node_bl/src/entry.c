#include <esp_err.h>

#include "can.h"

void app_main()
{
    esp_err_t err;

    err = can_init();
}
