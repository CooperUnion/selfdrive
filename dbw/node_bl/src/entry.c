#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/twai.h>
#include <string.h>

#include "esp_ota_ops.h"

#include "can.h"
#include "can_gen.h"

// ######        DEFINES        ###### //

#define MAX_TRIGGER_WAIT_TIME_MS 1000

#define GPIO_LED1 GPIO_NUM_33
#define GPIO_LED2 GPIO_NUM_32

static uint8_t fw_image[1024 * 160];

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static struct {
    const esp_partition_t *running, *boot, *update;
} part_info;

static enum {
    BL_STATE_AWAIT_TRIGGER,
    BL_STATE_BOOT_INTO_FW,
    BL_STATE_RECIEVING_IMG,
    BL_STATE_WRITING_IMG,
    BL_STATE_FAULT,
} bl_state;

// ######          CAN          ###### //

void can_send(int id, int val)
{
    twai_message_t message;
    message.extd = 0;
    message.rtr = 0;

    message.identifier = id;

    message.data_length_code = 1;

    message.data[0] = val;
    message.data[1] = 0;
    message.data[2] = 0;
    message.data[3] = 0;

    esp_err_t r = twai_transmit(&message, 0);
}

// ######   PRIVATE FUNCTIONS   ###### //

static void init_leds(void)
{
    gpio_pad_select_gpio(GPIO_LED1);
    gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED1, 0);

    gpio_pad_select_gpio(GPIO_LED2);
    gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED2, 0);
}

static void set_led1(bool state)
{
    gpio_set_level(GPIO_LED1, state);
}

static void set_led2(bool state)
{
    gpio_set_level(GPIO_LED2, state);
}

static void populate_part_info(void)
{
    part_info.running = esp_ota_get_running_partition();
    part_info.boot = esp_ota_get_boot_partition();
    part_info.update = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
}

// ######   PUBLIC FUNCTIONS    ###### //

void app_main()
{
    can_init();
    init_leds();
    set_led1(1);

    populate_part_info();

    for (;;) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
        can_ping();

        /*
        if (part_info.running)
            can_send(0x10, part_info.running->subtype);
        if (part_info.boot)
            can_send(0x11, part_info.boot->subtype);
        if (part_info.update)
            can_send(0x12, part_info.update->subtype);
        */

        // get new firmware image
        uint32_t size = 0;
        bool led2_state = 0;
        bool skip_eligible = true;
        while (!size) {
            twai_message_t msg;
            if (twai_receive(&msg, pdTICKS_TO_MS(MAX_TRIGGER_WAIT_TIME_MS)) == ESP_OK) {
                switch (msg.identifier) {
                    case CAN_DBW_UPDATERUPDATETRIGGER_FRAME_ID:
                    {
                        skip_eligible = false;
                        gpio_set_level(GPIO_LED1, 1);
                        gpio_set_level(GPIO_LED2, 1);
                        can_send(CAN_DBW_NODEUPDATERESPONSE_FRAME_ID, 0);
                        break;
                    }
                    case CAN_DBW_UPDATERUPDATEDATA_FRAME_ID:
                    {
                        gpio_set_level(GPIO_LED2, led2_state);
                        led2_state = !led2_state;

                        uint32_t position = 0;
                        position |= msg.data[0];
                        position |= msg.data[1] << 8;

                        position *= 5;

                        memcpy(fw_image + position, msg.data + 3, 5);
                        break;
                    }
                    case CAN_DBW_UPDATERUPDATEDONE_FRAME_ID:
                    {
                        size |= msg.data[0];
                        size |= msg.data[1] << 8;
                        size |= msg.data[2] << 16;
                        size |= msg.data[3] << 24;
                        break;
                    }
                    default:
                        break;
                }
            }
            else if (skip_eligible && part_info.update) {
                goto done;
            }
        }

        gpio_set_level(GPIO_LED1, 0);
        gpio_set_level(GPIO_LED2, 1);

        esp_ota_handle_t update_handle = 0;
        esp_err_t err = esp_ota_begin(part_info.update, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);

        can_send(0x14, err);
        if (err != ESP_OK) {
            esp_ota_abort(update_handle);
            continue;
        }

        err = esp_ota_write(update_handle, fw_image, size);

        can_send(0x15, err);
        if (err != ESP_OK) {
            esp_ota_abort(update_handle);
            continue;
        }

        err = esp_ota_end(update_handle);
        can_send(0x16, err);
        if (err != ESP_OK) {
            continue;
        }

done:
        err = esp_ota_set_boot_partition(part_info.update);
        can_send(0x17, err);
        if (err != ESP_OK) {
            continue;
        }

        can_send(0x18, err);

        esp_restart();
    }
}
