#include "sup.h"

// mode 1 = Constant/Solid
// mode 2 = Rotating clockwise
// mode 3 = Flickering horizontally
// mode 4 = Flickering all over
// mode 5 = Blinking clockwise
// mode 6 = Blinking counterclockwise
// mode 7 = blinking clockwise then flicker all over
// -> back to mode 1

#include <driver/gpio.h>

#include <nvs.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "common.h"
#include "cuber_base.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_templates.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //
#define STROBE_GPIO 34 // value needs updating

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static bool brake_authorized;
static bool throttle_authorized;
static bool steer_authorized;

static nvs_handle_t out_handle;

enum strobe_states {
    STROBE_SOLID = 0,
    STROBE_BLINK = 3,
};

static enum strobe_states desired_state;
static enum strobe_states previous_state;

static bool toggle_strobe;

/*
static uint8_t solid_state                   = 1;
static uint8_t clockwise_state               = 2;
static uint8_t horiztonal_flicker_state      = 3;
static uint8_t flicker_state                 = 4;
static uint8_t clockwise_desired_state         = 5;
static uint8_t counter_clockwise_desired_state = 6;
static uint8_t blink_flick_state             = 7;
*/

// ######    RATE FUNCTIONS     ###### //
static void sup_init();
static void sup_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init = sup_init,
    .call_100Hz = sup_100Hz,
};

void strobe_light_task(void * unused) {
    (void)unused;

    uint8_t previous_gpio;
    esp_err_t get_previous_gpio = nvs_get_u8(out_handle, "strobe", &previous_gpio);

    if (previous_gpio == 0) {previous_state = STROBE_SOLID;}
    else { previous_state = STROBE_BLINK;}

    for (;;)
    {
        toggle_strobe = (previous_state != desired_state);
        if (toggle_strobe)
        {
            if (desired_state == STROBE_SOLID)
            {
                for (int i = 0; i < 4; i++) {
                    gpio_set_level(STROBE_GPIO, 1);
                    vTaskDelay(100);
                    gpio_set_level(STROBE_GPIO, 0);
                    vTaskDelay(100);
                }
                previous_gpio = 0;
                previous_state = STROBE_SOLID;
                nvs_set_u8(out_handle, "strobe", 0);
            }
            else
            {
                for (int i = 0; i < 1; i++) {
                    gpio_set_level(STROBE_GPIO, 1);
                    vTaskDelay(100);
                    gpio_set_level(STROBE_GPIO, 0);
                    vTaskDelay(100);
                }
                previous_gpio = 1;
                previous_state = STROBE_BLINK;
                nvs_set_u8(out_handle, "strobe", 1);
            }
        }
        else {
            continue;
        }
    }
}

static void sup_init()
{
    gpio_config(&(gpio_config_t){
        .pin_bit_mask = BIT64(STROBE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    });

    nvs_flash_init();
    esp_err_t init_state = nvs_open("strobe", NVS_READWRITE, &out_handle);
    nvs_set_u8(out_handle, "strobe", 0);
}

static void sup_100Hz()
{
    bool authorized;

    // BRAKE
    authorized = true;
    taskDISABLE_INTERRUPTS();
    authorized &= CANRX_is_message_DBW_VelocityCommand_ok() || CANRX_is_message_DBW_RawVelocityCommand_ok();
    authorized &= CANRX_is_node_CTRL_ok();
    authorized &= CANRX_is_node_BRAKE_ok();
    authorized &= CANRX_get_CTRL_sysStatus()  != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    authorized &= CANRX_get_BRAKE_sysStatus() != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    authorized &= !CANRX_get_CTRL_speedAlarm();
    taskENABLE_INTERRUPTS();
    brake_authorized = authorized;

    // STEER
    authorized = true;
    taskDISABLE_INTERRUPTS();
    authorized &= CANRX_is_message_DBW_VelocityCommand_ok();
    authorized &= CANRX_get_STEER_sysStatus() != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    taskENABLE_INTERRUPTS();
    steer_authorized = authorized;

    // THROTTLE
    authorized = true;
    taskDISABLE_INTERRUPTS();
    authorized &= CANRX_is_message_DBW_VelocityCommand_ok() || CANRX_is_message_DBW_RawVelocityCommand_ok();
    authorized &= CANRX_is_node_CTRL_ok();
    authorized &= CANRX_is_node_THROTTLE_ok();
    authorized &= CANRX_get_CTRL_sysStatus()     != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    authorized &= CANRX_get_THROTTLE_sysStatus() != CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
    authorized &= !CANRX_get_CTRL_speedAlarm();
    taskENABLE_INTERRUPTS();
    throttle_authorized = authorized;

    if (brake_authorized || steer_authorized || throttle_authorized) {
       base_request_state(CUBER_SYS_STATE_DBW_ACTIVE);
       desired_state = STROBE_SOLID;
    } else {
        base_request_state(CUBER_SYS_STATE_IDLE);
        desired_state = STROBE_BLINK;
    }

    static bool strobe_task_started = false;
    if (!strobe_task_started) {
        static TaskHandle_t strobe_light_handle;
        xTaskCreatePinnedToCore(strobe_light_task, "STROBE_LIGHT", 8192, 0, 3, &strobe_light_handle, 0);
        strobe_task_started = true;
    }
}

// spawn an rtos task to do this.
// make seperate rtos task (or 1hz) that is not cyclic
// and whenever you want to switch strobe light have some shared var
// that you write to write to when requesting new state
//

// only read when the tasking function starts, dont read in cyclic or init.
// ######        CAN TX         ###### //

void CANTX_populate_SUP_Authorization(
        struct CAN_Message_SUP_Authorization * const m)
{
    m->SUP_brakeAuthorized    = brake_authorized;
    m->SUP_throttleAuthorized = throttle_authorized;
    m->SUP_steerAuthorized    = steer_authorized;
}
