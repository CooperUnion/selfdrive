#include "strobe.h"

#include <driver/gpio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "base/base.h"
#include "common.h"
#include "io/can.h"
#include "module_types.h"
#include "sys/task_glue.h"

/* Define firmware module identity for the entire build. */
const enum firmware_module_types FIRMWARE_MODULE_IDENTITY = MOD_STROBE;

// ######        DEFINES        ###### //

#define STROBE_GPIO 34

// ######      PROTOTYPES       ###### //

static void strobe_flash();
static void strobe_solid();

// ######     PRIVATE DATA      ###### //

// ######          CAN          ###### //

static struct CAN_dbwNode_Strobe_Data_t CAN_Strobe;

static const can_outgoing_t can_Strobe_Data_cfg = {
    .id = CAN_DBWNODE_STROBE_DATA_FRAME_ID,
    .extd = CAN_DBWNODE_STROBE_DATA_IS_EXTENDED,
    .dlc = CAN_DBWNODE_STROBE_DATA_LENGTH,
    .pack = CAN_dbwNode_Strobe_Data_pack,
};

// ######    RATE FUNCTIONS     ###### //

static void strobe_init();
static void strobe_100Hz();

const struct rate_funcs module_rf = {
    .call_init  = strobe_init,
    .call_100Hz = strobe_100Hz,
};

static void strobe_init(){
    CAN_Strobe.Strobe = 0;

    for (int i=0; i<6; i++){
        gpio_set_level(STROBE_GPIO, 1);
        vTaskDelay(10);
        gpio_set_level(STROBE_GPIO, 0);
        vTaskDelay(10);
    }
}

static void strobe_100Hz(){
    if(base_dbw_currently_active() != CAN_Strobe.Strobe){
        if(base_dbw_currently_active() == 1)
            strobe_flash();
        else
            strobe_solid();
    }

    CAN_Strobe.Strobe = base_dbw_currently_active();
    can_send_iface(&can_Strobe_Data_cfg, &CAN_Strobe);
}

// ######   PRIVATE FUNCTIONS   ###### //

//solid to flashing
static void strobe_flash(){
    for (int i=0; i<1; i++){
        gpio_set_level(STROBE_GPIO, 1);
        vTaskDelay(10);
        gpio_set_level(STROBE_GPIO, 0);
        vTaskDelay(10);
    }
}

//flashing to solid
static void strobe_solid(){
    for (int i=0; i<4; i++){
        gpio_set_level(STROBE_GPIO, 1);
        vTaskDelay(10);
        gpio_set_level(STROBE_GPIO, 0);
        vTaskDelay(10);
    }
}

// ######   PUBLIC FUNCTIONS    ###### //
